#include "ols.h"
#include <sstream>
#include <booster/log.h>
#include <android/log.h>
#define LOG(format, ...) __android_log_print(ANDROID_LOG_ERROR, "UVC", "[%s:%d/%s] " format "\n", basename(__FILE__), __LINE__, __FUNCTION__, ##__VA_ARGS__)

namespace ols {
    class AndroidSink : public booster::log::sink {
    public:
        virtual void log(booster::log::message const &m) 
        {
            int prio = ANDROID_LOG_DEBUG;
            if(m.level() <= booster::log::error)
                prio = ANDROID_LOG_ERROR;
            else if(m.level() <= booster::log::info)
                prio = ANDROID_LOG_INFO;
            __android_log_print(prio,"OLS","[%s:%d]:%s",m.file_name(),m.file_line(),m.log_message().c_str());
        }
    };
}


extern "C" {
    static char ols_andoid_error_str[1024];
    static std::unique_ptr<ols::OpenLiveStacker> g_stacker;
    char const *ols_android_error()
    {
        return ols_andoid_error_str;
    }
    int ols_android_init(
            char const *data_path,          /// Path to data location
            char const *document_root,      /// Document root for html files
            char const *http_ip,            /// IP to bind to, default should be 127.0.0.1
            int http_port,                  /// port to bind to, default should be 8080
            char const *driver_dir,         /// location of OLS drivers to load from
            char const *driver,             /// driver name
            char const *driver_config,      /// driver config option
            int driver_parameter)           /// FD for UVC camera, or CameraID for ASI
    {
        try {
            booster::shared_ptr<booster::log::sink> logger(new ols::AndroidSink());
            booster::log::logger::instance().add_sink(logger);
            ols::CameraDriver::load_driver(driver,driver_dir,driver_config);
            BOOSTER_INFO("ols") <<"Driver loaded" << driver;
            g_stacker.reset(new ols::OpenLiveStacker(data_path));
            g_stacker->http_port = http_port;
            g_stacker->http_ip = http_ip;
            g_stacker->document_root = document_root;
            g_stacker->init(driver,driver_parameter);
            BOOSTER_INFO("ols") << "Service created";
        }
        catch(std::exception const &e) {
            snprintf(ols_andoid_error_str,sizeof(ols_andoid_error_str),"Init Failed:%s",e.what());
            g_stacker.reset();
            return 1;
        }
        return 0;
    }

    int ols_android_run()
    {
        try {
            if(!g_stacker) {
                snprintf(ols_andoid_error_str,sizeof(ols_andoid_error_str),"Stacker had not started yet");
                return 1;
            }
            BOOSTER_INFO("ols") << "Starting service";
            g_stacker->run();
            g_stacker.reset();
        }
        catch(std::exception const &e) {
            std::ostringstream ss;
            ss << booster::trace(e);
            LOG("GOT exception %s: %s",e.what(),ss.str().c_str());
            snprintf(ols_andoid_error_str,sizeof(ols_andoid_error_str),"Run Failed:%s",e.what());
            g_stacker.reset();
            return 1;
        }
        catch(...) {
            LOG("GOT Unknown exception");
            g_stacker.reset();
            return 1;
        }
        return 0;
    }
    
    int ols_android_shutdown()
    {
        if(!g_stacker) {
            snprintf(ols_andoid_error_str,sizeof(ols_andoid_error_str),"Stacker had not started yet");
            return 1;
        }
        try {
            g_stacker->shutdown();
        }
        catch(std::exception const &e) {
            snprintf(ols_andoid_error_str,sizeof(ols_andoid_error_str),"Run Failed:%s",e.what());
            return 1;
        }
        return 0;
    }
}