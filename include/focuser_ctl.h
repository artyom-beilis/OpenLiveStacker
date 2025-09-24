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


namespace ols {
    class Focuser;
    class FocuserControlApp : public ControlAppBase {
    public:
        
        FocuserControlApp(cppcms::service &srv) : 
            ControlAppBase(srv)
        {
            dispatcher().map("GET", "/drivers/?",   &FocuserControlApp::drivers,this);
            dispatcher().map("POST", "/load/?",     &FocuserControlApp::load,this);
            dispatcher().map("GET", "/status/?",    &FocuserControlApp::get_status,this);
            dispatcher().map("POST","/move_to/?",   &FocuserControlApp::move_to,this);
            dispatcher().map("POST","/move_by/?",   &FocuserControlApp::move_by,this);
            dispatcher().map("POST","/abort/?",&    FocuserControlApp::abort,this);
        }

        ~FocuserControlApp();

        typedef std::unique_lock<std::recursive_mutex> guard_type;
        void drivers();
        void load();
        void get_status();
        void move_to();
        void move_by();
        void abort();
        guard_type guard();
    private:
        Focuser &focuser();
    };
}
