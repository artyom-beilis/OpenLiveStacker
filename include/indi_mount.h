#pragma once

namespace ols {
    class Mount;
    std::unique_ptr<Mount> indi_mount_create(std::string const &device_name,std::string const &host = "localhost",int port = 7624,bool disable_serial = false);
    void indi_list_drivers(std::vector<DriverInfo> &drivers,std::string const &libdir);
#ifdef INDI_AS_LIBRARY    
    bool start_indi_driver(std::string driver,std::string const &libdir,MountErrorCode &e);
#endif    
}
