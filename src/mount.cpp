#include "mount.h"

#ifdef WITH_INDI 
    #include "indi_mount.h"
#endif

namespace ols {


    MountInterface::MountInterface()
    {
    }
    MountInterface::~MountInterface()
    {
        shutdown_mount();
    }
    
    bool MountInterface::driver_is_loaded()
    {
        auto g=guard();
        return driver_.get();
    }
    bool MountInterface::mount_is_loaded()
    {
        auto g=guard();
        return client_.get();
    }
    Mount *MountInterface::client()
    {
        if(!client_)
            throw MountError("Client is not ready");
        return client_.get();
    }
 
    void MountInterface::config_libdir(std::string const &ld)
    {
        auto g = guard();
        libdir_ = ld;
    }

    void MountInterface::shutdown_mount()
    {
        auto g = guard();
        if(client_)
            client_.reset();
        if(driver_) {
            driver_->shutdown();
            driver_.reset();
        }
    }

    std::vector<DriverInfo> MountInterface::drivers_list()
    {
        auto g = guard();
        if(drivers_.empty()) {
#ifdef WITH_INDI
            indi_list_drivers(drivers_,libdir_);
#endif            
        }
        return drivers_;
    }

    void MountInterface::load_driver(std::string const &name,std::string const &opt)
    {
        auto g = guard();
        if(driver_) {
            throw MountError("The driver is already loaded");
        }
#ifdef WITH_INDI 
        if(name.find("indi:") == 0) {
            MountErrorCode e;
            std::unique_ptr<MountDriver> m = indi_start_driver(name,opt,libdir_,e);
            e.check();
            driver_ = std::move(m);
            return;
        }
#endif
        throw MountError("Unsupported driver " + name);
    }

    void MountInterface::load_mount()
    {
        auto g = guard();
        if(!driver_)
            throw MountError("Driver is not loaded");
        if(client_)
            throw MountError("Mount client is already loaded");
        MountErrorCode e;
        auto cl = driver_->get_mount(e);
        e.check();
        client_ = std::move(cl);
    }
    
} // namespace