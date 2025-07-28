#include "mount.h"

#include "alpaca_mount.h"
#ifdef WITH_INDI 
    #include "indi_mount.h"
#endif

#include <fstream>

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
            drivers_.push_back(DriverInfo("alpaca","http://127.0.0.1:32323",true));
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
        if(name == "alpaca") {
            MountErrorCode e;
            std::unique_ptr<MountDriver> m = alpaca_start_driver(opt,e);
            e.check();
            driver_ = std::move(m);
            return;
        }
#ifdef WITH_INDI 
        else if(name.find("indi:") == 0) {
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
   
    Mount::Mount() {
        load_alt();
    }
    Mount::~Mount()
    {
    }
    std::string Mount::ols_config_file()
    {
        char const *home = getenv("HOME");
        if(home == nullptr)
            home = ".";
        std::string file = home;
        file += "/.ols_mount_config.txt";
        return file;
    }
    void Mount::check_alt_limits(int low,int high,MountErrorCode &e)
    {
        if(high - low < 45 || high > 90 || low < -90 || high < 30) {
            e = "Invalid altitude values";
        }
    }
    void Mount::load_alt()
    {
        std::ifstream f(ols_config_file());
        if(!f)
            return;
        int low = 0, high = 0;
        f >> low >> high;
        if(!f)
            return;
        MountErrorCode e;
        check_alt_limits(low,high,e);
        if(e)
            return;
        low_alt_ = low;
        high_alt_ = high;
    }
    void Mount::save_alt(MountErrorCode &e)
    {
        std::string name = ols_config_file();
        std::ofstream f(name);
        if(!f) {
            e = "Failed to open file " + name;
            return;
        }
        f<< low_alt_ << " " << high_alt_ << "\n";
        f.close();
    }
} // namespace
