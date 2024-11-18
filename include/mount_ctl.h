#pragma once
#include <cppcms/application.h>
#include <cppcms/http_context.h>
#include <cppcms/http_response.h>
#include <cppcms/url_dispatcher.h>
#include <cppcms/service.h>
#include <booster/log.h>
#include <sstream>
#include <iomanip>
#include "ctl_app.h"
#include "data_items.h"

namespace booster{ namespace aio { class deadline_timer; } }

namespace ols {
    class Mount;
    class MountInterface;
    struct EqCoord;
    struct AltAzCoord;
    class MountControlApp : public ControlAppBase {
    public:
        
        MountControlApp(cppcms::service &srv,queue_pointer_type notify,MountInterface *mi) : 
            ControlAppBase(srv),
            notification_queue_(notify),
            mi_(mi)
        {
            dispatcher().map("GET", "/config/?",&MountControlApp::get_config_status,this);
            dispatcher().map("GET", "/status/?",&MountControlApp::get_status,this);
            dispatcher().map("POST","/geolocation/?",&MountControlApp::set_geolocation,this);
            dispatcher().map("POST","/alt_limits/?",&MountControlApp::set_alt_limits,this);
            dispatcher().map("POST","/start/?",&MountControlApp::start_driver,this);
            dispatcher().map("POST","/connection_proto/?",&MountControlApp::set_proto,this);
            dispatcher().map("POST","/connection_addr/?",&MountControlApp::set_addr,this);
            dispatcher().map("POST","/connect?",&MountControlApp::connect,this);
            dispatcher().map("POST","/goto/?",&MountControlApp::go_to,this);
            dispatcher().map("POST","/slew/?",&MountControlApp::slew,this);
            dispatcher().map("POST","/sync/?",&MountControlApp::sync,this);
            dispatcher().map("POST","/reset_alignment/?",&MountControlApp::reset_alignment,this);
            dispatcher().map("POST","/tracking/?",&MountControlApp::set_tracking_mode,this);
            dispatcher().map("POST","/meridian/?",&MountControlApp::set_meridian_behavior,this);
        }

        ~MountControlApp();

        void get_config_status();
        void get_status();
        void start_driver();
        void connect();
        void go_to();
        void slew();
        void sync();
        void set_proto();
        void set_addr();
        void set_geolocation();
        void set_tracking_mode();
        void reset_alignment();
        void set_alt_limits();
        void set_meridian_behavior();
    private:
        void setup_client();
        static void send_error_message(queue_pointer_type q,std::string const &msg);
        static void send_pointing_update(queue_pointer_type q,EqCoord const &pos,AltAzCoord const &altaz);
        Mount *check_connected();
        queue_pointer_type notification_queue_;
        MountInterface *mi_;
    };
}
