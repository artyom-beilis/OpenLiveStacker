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

        ols::CameraDriver::load_driver(driver,path);
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
