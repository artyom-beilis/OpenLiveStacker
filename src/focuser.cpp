#include "focuser.h"
#include "alpaca_focuser.h"

namespace ols {
    FocuserInterface &FocuserInterface::instance()
    {
        static FocuserInterface fi;
        return fi;
    }
    Focuser *FocuserInterface::focuser()
    {
        guard_type g(instance().guard());
        return instance().focuser_.get();
    }
    FocuserInterface::FocuserInterface() 
    {
    }
    FocuserInterface::~FocuserInterface()
    {
    }
    void FocuserInterface::load_driver(FocuserInfo const &info,FocuserErrorCode &e)
    {
        guard_type g(lock_);
        if(focuser_) {
            e = "Driver is loaded";
            return;
        }
        if(info.name == "alpaca:remote") {
            focuser_ = load_alpaca_focuser(info.opt,e);
            return;
        }
        else {
            e = "Invalid driver name: " + info.name;
        }
    }
    std::vector<FocuserInfo> FocuserInterface::drivers_list()
    {
        guard_type g(lock_);
        return {{ "alpaca:remote","auto" }};
    }
    void FocuserInterface::unload_driver()
    {
        guard_type g(lock_);
        focuser_.reset();
    }
}
