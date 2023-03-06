#include "ols.h"

extern "C" {
    static char ols_andoid_error_str[1024];
    static std::unique_ptr<ols::OpenLiveStacker> g_stacker;
    char const *old_android_error()
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
            int driver_parameter)           /// FD for UVC camera, or CameraID for ASI
    {
        try {
            ols::CameraDriver::load_driver(driver,driver_dir);
            g_stacker.reset(new ols::OpenLiveStacker(data_path));
            g_stacker->http_port = http_port;
            g_stacker->http_ip = http_ip;
            g_stacker->document_root = document_root;
            g_stacker->init(driver,driver_parameter);
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
            g_stacker->run();
        }
        catch(std::exception const &e) {
            snprintf(ols_andoid_error_str,sizeof(ols_andoid_error_str),"Run Failed:%s",e.what());
            g_stacker.reset();
            return 1;
        }
        return 0;
    }
    
    int ols_astroid_shutdown()
    {
        if(!g_stacker) {
            snprintf(ols_andoid_error_str,sizeof(ols_andoid_error_str),"Stacker had not started yet");
            return 1;
        }
        try {
            g_stacker->shutdown();
            g_stacker.reset();
        }
        catch(std::exception const &e) {
            snprintf(ols_andoid_error_str,sizeof(ols_andoid_error_str),"Run Failed:%s",e.what());
            g_stacker.reset();
            return 1;
        }
        return 0;
    }
}
