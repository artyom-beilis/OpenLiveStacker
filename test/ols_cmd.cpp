#include "ols.h"
#include <iostream>
#include <fstream>
#include <unistd.h>

int main(int argc,char **argv)
{   
    try {

        std::string path;
        std::string driver;

        if(argc < 2) {
            std::cerr  << "Usage driver_name [/path/to/drivers/dir]" << std::endl;
            return 1;
        }
        driver=argv[1];
        if(argc >= 3)
            path = argv[2];

        char const *driver_opt_ptr = nullptr;
        std::string driver_opt;
        size_t pos;
        if((pos=driver.find(':'))!=std::string::npos) {
            driver_opt = driver.substr(pos+1);
            driver_opt_ptr = driver_opt.c_str();
            driver=driver.substr(0,pos);
        }

        ols::CameraDriver::load_driver(driver,path,driver_opt_ptr);
        ols::OpenLiveStacker stacker;
        stacker.init(driver);
        stacker.run();
        stacker.shutdown();
    }
    catch(std::exception const &e) {
        std::cerr << "failed:" << e.what() << std::endl;
        return 1;
    }
    std::cout << "Done" << std::endl;
    return 0;
}
