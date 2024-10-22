#define INDI_AS_LIBRARY 
#ifdef INDI_AS_LIBRARY
#include "baseclient.h"
#include "basedevice.h"
#else
#include <libindi/baseclient.h>
#include <libindi/basedevice.h>
#endif

#include <sys/time.h>
#include <iostream>
#include <sstream>
#include <string>
#include <mutex>
#include <thread>
#include <queue>

#include "camera.h"
#include "mount_ctl.h"

#ifdef INDI_AS_LIBRARY
int indiserver_main(std::vector<std::string> drivers);
#endif


namespace ols {

static std::mutex last_messages_lock;
static std::list<std::string> last_messages;

#define LOG(...) do {  \
    char buf[256]; \
    int n = snprintf( buf, sizeof(buf) ,__VA_ARGS__);  \
    if(n < 255 && n > 0 && buf[n-1] == '\n')  {\
        buf[n-1] = 0; \
    } \
    { \
         std::unique_lock<std::mutex> g(last_messages_lock); \
         if(last_messages.size() > 20) \
             last_messages.pop_front(); \
        last_messages.push_back(buf); \
    } \
    BOOSTER_INFO("stacker") << buf; \
} while(0)


typedef std::unique_lock<std::recursive_mutex> guard_type;
static void log_property(char const *src,INDI::Property const &p) 
{
    char const *perms[]={"ro","wo","rw"};
    LOG("%3s %s:%s (%s) %s type %s\n",src,p.getDeviceName(), p.getName(), p.getStateAsString(), perms[p.getPermission()], p.getTypeAsString());
    switch(p.getType()) {
        case INDI_NUMBER:
            {
                INDI::PropertyNumber v(p);
                for(size_t i=0;i<v.size();i++) {
                    LOG("  %2d %s/%s val=%f min=%f max=%f step=%f\n",int(i),v[i].getName(),v[i].getLabel(),v[i].getValue(),v[i].getMin(),v[i].getMax(),v[i].getStep());
                }
            }
            break;
        case INDI_TEXT:
            {
                INDI::PropertyText v(p);
                for(size_t i=0;i<v.size();i++) {
                    LOG("  %2d %s/%s val=%s\n",int(i),v[i].getName(),v[i].getLabel(),v[i].getText());
                }
            }
            break;
        case INDI_SWITCH:
            {
                INDI::PropertySwitch v(p);
                for(size_t i=0;i<v.size();i++) {
                    LOG("  %2d %s/%s %s\n",int(i),v[i].getName(),v[i].getLabel(),(v[i].getState() == ISS_ON ? "ON " : "OFF"));
                }
            }
            break;
        case INDI_LIGHT:
            {
                INDI::PropertyLight v(p);
                for(size_t i=0;i<v.size();i++) {
                    LOG("  %2d %s/%s %s\n",int(i),v[i].getName(),v[i].getLabel(),v[i].getStateAsString());
                }
            }
            break;
        case INDI_BLOB:
            {
                INDI::PropertyBlob v(p);
                for(size_t i=0;i<v.size();i++) {
                    LOG("  %2d %s/%s ptr=%p size=%d\n",int(i),v[i].getName(),v[i].getLabel(),v[i].getBlob(),v[i].getSize());
                }
            }
            break;
        default:
            ;
    }
}


class MountClient : public INDI::BaseClient {
public:
    std::string latest_ra;
    std::string latest_dec;

    MountClient(char *port = nullptr)
    {
        if(port)
            port_ = port;
        setServer("localhost",7624);
        if(!connectServer()) {
            throw std::runtime_error("Faild to connect");
        }
    }

    void sendNewProperty(INDI::Property prop)
    {
        log_property("SND",prop);
        INDI::BaseClient::sendNewProperty(prop);
    }

    virtual void updateProperty(INDI::Property prop) override
    {
        guard_type g(lock_);
        handle(true,prop);
    }
    virtual void newProperty(INDI::Property prop) override
    {
        guard_type g(lock_);
        handle(false,prop);
    }
    void handle(bool is_new,INDI::Property prop)
    {
        log_property(is_new?"new":"upd",prop);
        if(prop.isNameMatch("DRIVER_INFO") && name_.empty()) {
            INDI::PropertyText p(prop);
            auto interface = p.findWidgetByName("DRIVER_INTERFACE");
            if(interface && atoi(interface->getText()) & INDI::BaseDevice::TELESCOPE_INTERFACE) {
                LOG("!!!!!!!!!!!!!!!!!!!!!!!!!!!! DERECTED %s\n",prop.getDeviceName());
                name_ = prop.getDeviceName();
                on_detected(prop.getBaseDevice());
            }
        }
        /*if(prop.isNameMatch("CONNECTION") && prop.isDeviceNameMatch(name_)) {
            handle_connection(prop);
        }*/
        else if(prop.isNameMatch("CONNECTION_MODE")) {
            handle_mode(prop);
        }
        else if(prop.isNameMatch("EQUATORIAL_EOD_COORD")) {
            log_ra_de(prop);
        }
        else if(prop.isNameMatch("GEOGRAPHIC_COORD")) {
            set_geolocation(prop);
        }
    }

    bool coord_def_ = false;
    void set_geolocation(INDI::PropertyNumber p)
    {
        if(coord_def_)
            return;
        auto lat = p.findWidgetByName("LAT");
        auto lon = p.findWidgetByName("LONG");
        if(!lat || !lon) {
            LOG("NO WID\n");
            return;
        }
        lat->setValue(31.0);
        lon->setValue(34.0);
        sendNewProperty(p);
        coord_def_ = true;
    }
    
    void go_to(double RA,double DEC)
    {
        guard_type g(lock_);
        /*for(auto p : device_.getProperties()) {
            log_property("GTO",p);
        }*/
        INDI::PropertySwitch on_set = device_.getProperty("ON_COORD_SET");
        if(!on_set.isValid()) {
            throw std::runtime_error("No ON_COORD_SET");
        }
        INDI::PropertyNumber coord = device_.getProperty("EQUATORIAL_EOD_COORD");
        if(!coord.isValid()) {
            throw std::runtime_error("No EQUATORIAL_EOD_COORD");
        }
        auto on_set_w = on_set.findWidgetByName("TRACK");
        auto ra_w = coord.findWidgetByName("RA");
        auto dec_w = coord.findWidgetByName("DEC");
        
        if(!on_set_w || !ra_w || !dec_w) {
            throw  std::runtime_error("No goto widgets");
        }
        if(on_set_w->getState() != ISS_ON) {
            on_set_w->setState(ISS_ON);
            sendNewProperty(on_set);
        }
        ra_w->setValue(RA);
        dec_w->setValue(DEC);
        sendNewProperty(coord);
    }
    void log_ra_de(INDI::PropertyNumber prop) 
    {
        if(!prop.isValid())
            return;
        auto ra = prop.findWidgetByName("RA");
        auto de = prop.findWidgetByName("DEC");
        if(!ra || !de)
            return;
        double RA=ra->getValue();
        double DEC=de->getValue();

        int rh = int(RA);
        int rs = int(RA*3600) % 3600;
        int rm = rs / 60;
        rs = rs % 60;
        
        char s='+';
        if(DEC < 0) {
            s='-';
            DEC=-DEC;
        }
        int dd = int(DEC);
        int ds = int(DEC*3600) % 3600;
        int dm = ds / 60;
        ds = ds % 60;
        char ra_str[256];
        char dec_str[256];
        snprintf(ra_str,sizeof(ra_str),"%02d:%02d:%02d",rh,rm,rs);
        snprintf(dec_str,sizeof(dec_str),"%c%02d:%02d:%02d",s,dd,dm,ds);
        LOG("Coordinates RA=%s DE=%s\n",ra_str,dec_str);
        latest_ra = ra_str;
        latest_dec = dec_str;
    }
    void handle_mode(INDI::PropertySwitch p)
    {
        if(!p.isValid()) {
            LOG("Property is not valid\n");
            return;
        }
        char const *wname = port_.empty() ? "CONNECTION_TCP" : "CONNECTION_SERIAL";
        printf("Connecting to %s\n",wname);
        bool update = false;
        for(unsigned i=0;i<p.count();i++) {
            if(p[i].isNameMatch(wname)) {
                if(p[i].getState() != ISS_ON) {
                    p[i].setState(ISS_ON);
                    update = true;
                }
            }
            else {
                if(p[i].getState() != ISS_OFF) {
                    p[i].setState(ISS_OFF);
                    update = true;
                }
            }
        }
        if(update) {
            sendNewProperty(p);
        }
    }
    bool is_connected()
    {
        return connected_;
    }
    void on_detected(INDI::BaseDevice d)
    {
        device_ = d;
        watchDevice(name_.c_str());
        connectDevice(name_.c_str());
    }
    void handle_connection(INDI::PropertySwitch p)
    {
        auto ptr = p.findWidgetByName("CONNECT");
        if(!ptr) {
            LOG("No CONNECT in connection\n");
            return;
        }
        if(ptr->getState() != ISS_ON) {
            LOG("Connecting to %s\n",name_.c_str());
            connectDevice(name_.c_str());
        }
        else {
            LOG("Connected to camera %s\n",name_.c_str());
            connected_ = true;
        }
    }
    void connect() {
        /*
        INDI::PropertySwitch p = device_.getProperty("CONNECTION_TYPE");
        auto atcp = p.findWidgetByName("TCP");
        auto audp = p.findWidgetByName("UDP");
        if(!audp || !atcp)
            throw std::runtime_error("Can't swithc to udp");
        atcp->setState(ISS_ON);
        audp->setState(ISS_OFF);
        sendNewProperty(p);
        */
        handle_connection(device_.getProperty("CONNECTION"));
    }


private:
    std::string port_;
    std::recursive_mutex lock_;
    bool connected_ = false;
    std::string name_;
    INDI::BaseDevice device_;
};


double parseRA(int h,int m,int s)
{
    double hours = (h + (60 * m + s) / 3600.0 );
    return hours;
}

double parseDEC(int d,int m,int s)
{
    int sig=1;
    if(d < 0) {
        sig = -1;
        d = -d;
    }
    return sig*(d + (60 * m + s) / 3600.0);
}






std::mutex MountControlApp::lock;
std::unique_ptr<MountClient> MountControlApp::client;   
bool MountControlApp::server_started;

MountControlApp::~MountControlApp()
{
}

void MountControlApp::start_driver()
{
    guard_type g(lock);
    if(server_started) {
        throw std::runtime_error("Service is alreay running");
    }
    std::string driver = content_.get<std::string>("driver");
    if(driver != "telescope_skywatcherAltAz"  && driver != "telescope_simulator")
        throw std::runtime_error("Unsupported device");
    std::string path = libdir_ + "/libindidriver_" + driver + ".so";
    std::thread main([=]() {
        indiserver_main({path});
    });
    main.detach();
    usleep(500000);
    server_started = true;
    if(!client)
        client.reset(new MountClient());
}

void MountControlApp::get_status()
{
    guard_type g(lock);
    response_["server"] = server_started;
    if(client.get()) {
        response_["client"] = client->is_connected();
        response_["ra"] = client->latest_ra;
        response_["dec"] = client->latest_dec;
    }
    else {
        response_["client"] = false;
        response_["ra"] = "N/A";
        response_["dec"] = "N/A";
    }
    {
        std::unique_lock<std::mutex> g2(last_messages_lock);
        std::vector<std::string> r(last_messages.begin(),last_messages.end());
        response_["log"] = r;
    }
}

void MountControlApp::connect()
{
    guard_type g(lock);
    if(!client)
        throw std::runtime_error("Start server first");
    client->connect();
}

void MountControlApp::go_to()
{
    guard_type g(lock);
    if(!client)
        throw std::runtime_error("Not connected");
    std::istringstream ss(content_.get<std::string>("target.ra") + "  " + content_.get<std::string>("target.dec"));
    int rh,rm,rs,dh,dm,ds;
    char c;
    ss >> rh >>c >> rm >> c >> rs >> dh>>c>>dm>>c>>ds;
    if(!ss) {
        throw std::runtime_error("Bad format:" + ss.str());
    }
    client->go_to(parseRA(rh,rm,rs),parseDEC(dh,dm,ds));
}

} // namespace

