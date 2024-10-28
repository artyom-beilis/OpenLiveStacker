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
    struct EqCoord;
    class MountControlApp : public ControlAppBase {
    public:
        typedef std::unique_lock<std::mutex> guard_type;
        static std::mutex lock;
        static std::unique_ptr<Mount> client;
        
        MountControlApp(cppcms::service &srv,queue_pointer_type notify,std::string libdir) : 
            ControlAppBase(srv),
            notification_queue_(notify),
            libdir_(libdir)
        {
            dispatcher().map("GET", "/config/?",&MountControlApp::get_config_status,this);
            dispatcher().map("GET", "/status/?",&MountControlApp::get_status,this);
            dispatcher().map("POST","/geolocation/?",&MountControlApp::set_geolocation,this);
            dispatcher().map("POST","/start/?",&MountControlApp::start_driver,this);
            dispatcher().map("POST","/connection_proto/?",&MountControlApp::set_proto,this);
            dispatcher().map("POST","/connection_addr/?",&MountControlApp::set_addr,this);
            dispatcher().map("POST","/connect?",&MountControlApp::connect,this);
            dispatcher().map("POST","/goto/?",&MountControlApp::go_to,this);
            dispatcher().map("POST","/slew/?",&MountControlApp::slew,this);
            dispatcher().map("POST","/sync/?",&MountControlApp::sync,this);
            dispatcher().map("POST","/tracking/?",&MountControlApp::set_tracking_mode,this);
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
    private:
        void setup_client();
        static void send_error_message(queue_pointer_type q,std::string const &msg);
        static void send_pointing_update(queue_pointer_type q,EqCoord const &pos);
        void check_connected();
        queue_pointer_type notification_queue_;
        std::string libdir_;
    };
}
