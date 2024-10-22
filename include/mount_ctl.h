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

namespace ols {
    class MountClient;
    class MountControlApp : public ControlAppBase {
    public:
        typedef std::unique_lock<std::mutex> guard_type;
        static std::mutex lock;
        static std::unique_ptr<MountClient> client;
        static bool server_started;
        MountControlApp(cppcms::service &srv,std::string libdir) : 
            ControlAppBase(srv),
            libdir_(libdir)
        {
            dispatcher().map("GET", "/status/?",&MountControlApp::get_status,this);
            dispatcher().map("POST","/start/?",&MountControlApp::start_driver,this);
            dispatcher().map("POST","/connect?",&MountControlApp::connect,this);
            dispatcher().map("POST","/goto/?",&MountControlApp::go_to,this);
        }

        ~MountControlApp();

        void get_status();
        void start_driver();
        void connect();
        void go_to();
    private:
        std::string libdir_;
    };
}
