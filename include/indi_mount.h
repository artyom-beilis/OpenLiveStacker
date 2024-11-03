#pragma once
#include "mount.h"
namespace ols {
    void indi_list_drivers(std::vector<DriverInfo> &drivers,std::string const &libdir);
    std::unique_ptr<MountDriver> indi_start_driver(std::string const &d,std::string const &opt,std::string const &libdir,MountErrorCode &e);
}
