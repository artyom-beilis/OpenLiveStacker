#include "ols.h"
#include <cppcms/json.h>
#include <iostream>
#include <fstream>
#include <unistd.h>

int main(int argc,char **argv)
{   
    try {

        std::string path;
        std::string driver;
        std::string driver_opt;

        if(argc!=2) {
            std::cerr << "Usage ols_cmd config.json" << std::endl;
            return 1;
        }

        cppcms::json::value cfg;
        std::ifstream jsf(argv[1]);
        if(!jsf) {
            std::cerr << "Failed to open config file:"<<argv[1] << std::endl;
            return 1;
        }
        if(!cfg.load(jsf,true)) {
            std::cerr << "Failed to parse config file:" << argv[1]<<std::endl;
            return 1;
        }
        driver = cfg.get<std::string>("driver");
        path = cfg.get("libdir","");
        if(driver == "sim") {
            driver_opt = cfg.get("sim.path","");
        }
        else if(driver == "wdir") {
            int w = cfg.get<int>("wdir.width");
            int h = cfg.get<int>("wdir.height");
            std::string dir = cfg.get<std::string>("wdir.path");
            std::string format = cfg.get<std::string>("wdir.format");
            driver_opt=format + "@" + std::to_string(w) + "x" + std::to_string(h) + ":" + dir;
        }

        char const *driver_opt_ptr = nullptr;
        if(!driver_opt.empty()) {
            driver_opt_ptr = driver_opt.c_str();
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
