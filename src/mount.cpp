#include "mount.h"

#ifdef WITH_INDI 
    #include "indi_mount.h"
#endif

namespace ols {
    typedef std::unique_lock<std::recursive_mutex> guard_type;
    static std::recursive_mutex driver_lock;
    static bool driver_started = false;
    static std::string libdir;
#ifdef WITH_INDI 
    static std::string indi_host = "localhost";
    static int indi_port = 7624;
    static bool indi_is_remote=true;
#endif
    bool mount_driver_is_running()
    {
        guard_type g(driver_lock);
        return driver_started;
    }


    void mount_config_libdir(std::string const &ld)
    {
        guard_type g(driver_lock);
        libdir = ld;
    }

    void mount_driver_load(std::string const &name,std::string const &opt,MountErrorCode &e)
    {
        guard_type g(driver_lock);
#ifdef WITH_INDI 
        if(name.find("indi:") == 0) {
            if(name == "indi:remote") {
                indi_is_remote = true;
                size_t pos = opt.find(':');
                if(pos == std::string::npos) {
                    if(!opt.empty())
                        indi_host = opt;
                }
                else {
                    if(opt.size() > pos+1)
                        indi_port = atoi(opt.c_str()+pos+1);
                    if(pos > 0)
                        indi_host = opt.substr(0,pos);
                }
                driver_started = true;
                return;
            }
        #ifdef INDI_AS_LIBRARY
            else {
                indi_is_remote = false;
                std::string driver_name = name.substr(5);
                driver_started = start_indi_driver(driver_name,libdir,e);
                return;
            }
        #endif            
        }
#endif        
        e = "Unsupported driver " + name;
    }
    
    std::unique_ptr<Mount> get_mount(MountErrorCode &e)
    {
        guard_type g(driver_lock);
        std::unique_ptr<Mount> m;
        try {
#ifdef WITH_INDI
            #ifdef ANDROID_SUPPORT
            bool disable_serial= !indi_is_remote;
            #else
            bool disable_serial = false;
            #endif
            m = std::move(indi_mount_create("",indi_host,indi_port,disable_serial));
#endif
        }
        catch(std::exception const &ex) {
            e = ex;
        } 
        if(!m)
            e = "Failed to connect to the driver";
        return m;
    }
    std::vector<DriverInfo> mount_drivers_list()
    {
        guard_type g(driver_lock);
        static std::vector<DriverInfo> r;
#ifdef WITH_INDI
        if(r.empty()) {
            indi_list_drivers(r,libdir);
        }
#endif        
        return r;
    }
}
