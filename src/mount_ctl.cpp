#include <sys/time.h>
#include <iostream>
#include <sstream>
#include <string>
#include <mutex>
#include <thread>
#include <queue>
#include <unistd.h>

#include "camera.h"
#include "mount_ctl.h"
#include "mount.h"

#include <booster/aio/deadline_timer.h>


namespace ols {



MountControlApp::~MountControlApp()
{
}

void MountControlApp::start_driver()
{
    auto g=mi_->guard();
    if(mi_->driver_is_loaded())
        throw std::runtime_error("driver is alreay loaded");
    std::string driver = content_.get<std::string>("driver");
    std::string option = content_.get<std::string>("option");
    mi_->load_driver(driver,option);
    setup_client();
}

void MountControlApp::set_proto()
{
    auto g = mi_->guard();
    if(!mi_->mount_is_loaded())
        throw std::runtime_error("No connection to driver"); 
    auto client = mi_->client();
    std::string proto = content_.get<std::string>("proto");
    MountErrorCode e;
    if(proto == "inet")
        client->set_proto(proto_inet,e);
    else if(proto == "serial")
        client->set_proto(proto_serial,e);
    else
        throw std::runtime_error("Unsupported protocol " + proto);
    e.check();
}

void MountControlApp::set_addr()
{
    auto g=mi_->guard();
    if(!mi_->mount_is_loaded())
        throw std::runtime_error("No connection to driver"); 
    auto client = mi_->client();
    std::string proto = content_.get<std::string>("proto");
    std::string addr = content_.get<std::string>("addr");
    MountErrorCode e;
    if(proto == "inet")
        client->set_conn_string(proto_inet,addr,e);
    else if(proto == "serial")
        client->set_conn_string(proto_serial,addr,e);
    else
        throw std::runtime_error("Unsupported protocol " + proto);
    e.check();
}

void MountControlApp::set_geolocation()
{
    auto g=mi_->guard();
    auto client = check_connected();

    double lat = content_.get<double>("lat");
    double lon = content_.get<double>("lon");
    MountErrorCode e;
    client->set_lat_lon(lat,lon,e);
    e.check();
}


void MountControlApp::get_config_status()
{
    auto g=mi_->guard();
    if(!mi_->driver_is_loaded()) {
        response_["server"] = false;
        auto drivers = mi_->drivers_list();
        for(unsigned i=0;i<drivers.size();i++) {
            auto const &drv = drivers[i];
            response_["server_drivers"][i]["name"]=drv.name;
            response_["server_drivers"][i]["option"]=drv.option;
            response_["server_drivers"][i]["has_option"]=drv.has_option;
        }
    }
    else {
        response_["server"] = true;
        MountErrorCode e;
        if(!mi_->mount_is_loaded()) {
            setup_client();
        }
        auto client = mi_->client();
        bool connected = client->connected();
        response_["connected"] = connected;
        if(!connected) {
            int proto_list = client->supported_proto(e);
            e.check();
            if(proto_list & proto_inet) {
                response_["proto"]["inet"]["supported"]=true;
                response_["proto"]["inet"]["connection"] = client->get_conn_string(proto_inet,e);
                e.check();
            }
            else{
                response_["proto"]["inet"]["supported"]=false;
            }
            if(proto_list & proto_serial) {
                response_["proto"]["serial"]["supported"]=true;
                response_["proto"]["serial"]["connection"] = client->get_conn_string(proto_serial,e);
                e.check();
            }
            else {
                response_["proto"]["serial"]["supported"]=false;
            }
            MountProto proto = client->get_proto(e);
            e.check();
            if(proto == proto_inet) 
                response_["proto"]["current"] = "inet";
            else if(proto == proto_serial)
                response_["proto"]["current"] = "serial";
            else
                response_["proto"]["current"] = "none";
        }
    }
}

void MountControlApp::get_status()
{
    auto g=mi_->guard();
    auto client = check_connected();
    MountErrorCode e;
    auto pos = client->get_current(e);
    e.check();
    response_["ra"] = Mount::formatRA(pos.RA);
    response_["dec"] = Mount::formatDEC(pos.DEC);
    auto slew = client->get_slew_rate(e);
    e.check();
    response_["slew"]["max"] = slew.second;
    response_["slew"]["current"] = slew.first;
    MountTrac tr = client->get_tracking(e);
    e.check();
    std::vector<std::string> trmode{"sidereal","solar","lunar"};
    response_["tracking_mode"] = trmode[tr];
    auto lat_lon = client->get_lat_lon(e);
    e.check();
    response_["lat"] = lat_lon.first;
    response_["lon"] = lat_lon.second;
    response_["alignment"] = client->get_alignment_points(e);
    e.check();
}

void MountControlApp::reset_alignment()
{
    auto g=mi_->guard();
    auto client = check_connected();
    MountErrorCode e;
    client->reset_alignment(e);
    e.check();
}
void MountControlApp::set_tracking_mode()
{
    auto g = mi_->guard();
    auto client = check_connected();
    std::string m = content_.get<std::string>("tracking_mode");
    MountTrac t;
    if(m=="sidereal")
        t=trac_sidereal;
    else if(m=="solar")
        t=trac_solar;
    else if(m=="lunar")
        t=trac_lunar;
    else
        throw std::runtime_error("Unsuppoted mode " + m);
    MountErrorCode e;
    client->set_tracking(t,e);
    e.check();
}

void MountControlApp::setup_client()
{
    if(!mi_->mount_is_loaded()) {
        mi_->load_mount();
        auto client = mi_->client();
        std::weak_ptr<queue_type> wq = notification_queue_;
        client->set_update_callback([=](EqCoord coord,std::string const &msg) {
            queue_pointer_type q = wq.lock();
            if(!q)
                return;
            if(!msg.empty()) {
                MountControlApp::send_error_message(q,msg);
            }
            else {
                MountControlApp::send_pointing_update(q,coord);
            }
        });
        usleep(500000);
    }
}

void MountControlApp::send_error_message(queue_pointer_type q,std::string const &msg)
{
    std::shared_ptr<ErrorNotificationData> obj(new ErrorNotificationData());
    obj->source = "mount";
    obj->message = msg;
    q->push(obj);
}

void MountControlApp::send_pointing_update(queue_pointer_type q,EqCoord const &pos)
{
    std::shared_ptr<MountPositionNotification> obj(new MountPositionNotification());
    obj->RA = Mount::formatRA(pos.RA);
    obj->DEC = Mount::formatDEC(pos.DEC);
    q->push(obj);
}

void MountControlApp::connect()
{
    auto g=mi_->guard();
    if(!mi_->driver_is_loaded())
        throw std::runtime_error("Mount driver hadn't been started");
    if(!mi_->mount_is_loaded()) {
        setup_client();
    }
    MountErrorCode e;
    mi_->client()->connect(e);
    e.check();
}

void MountControlApp::go_to()
{
    auto g=mi_->guard();
    auto client = check_connected();
    MountErrorCode e;
    double RA=Mount::parseRA(content_.get<std::string>("ra"),e);
    e.check();
    double DEC=Mount::parseRA(content_.get<std::string>("dec"),e);
    e.check();
    client->go_to(EqCoord{RA,DEC},e);
    e.check();
}

Mount *MountControlApp::check_connected()
{
    if(!mi_->mount_is_loaded() || !mi_->client()->connected())
        throw std::runtime_error("Connect to mount first");
    return mi_->client();
}

void MountControlApp::sync()
{
    auto g=mi_->guard();
    auto client = check_connected();
    MountErrorCode e;
    double RA=Mount::parseRA(content_.get<std::string>("ra"),e);
    e.check();
    double DEC=Mount::parseRA(content_.get<std::string>("dec"),e);
    e.check();
    client->sync(EqCoord{RA,DEC},e);
    e.check();
}

void MountControlApp::slew()
{
    auto g=mi_->guard();
    auto client = check_connected();
    int speed = content_.get<int>("speed");
    std::string direction = content_.get<std::string>("direction");
    if(!direction.empty()) {
        MountErrorCode e;
        if(direction=="N") 
            client->slew(slew_N,speed,e);
        else if(direction == "S")
            client->slew(slew_S,speed,e);
        else if(direction == "W")
            client->slew(slew_W,speed,e);
        else if(direction == "E")
            client->slew(slew_E,speed,e);
        else
            throw std::runtime_error("Unsupoted slew direction: " + direction);
        e.check();
    }
    else {
        MountErrorCode e;
        client->slew(slew_stop,0,e);
        e.check();
    }
}



} // namespace
