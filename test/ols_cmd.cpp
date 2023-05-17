#include "ols.h"
#include "plate_solver.h"
#include <cppcms/json.h>
#include <booster/regex.h>
#include <iostream>
#include <fstream>
#include <unistd.h>

bool parse_key_value(std::string arg,cppcms::json::value &cfg)
{
	booster::regex r("([^=]+)=(.*)$");
    booster::regex isnum("^-?\\d+(\\.\\d+)?([eE][\\+-]?\\d+)?$");
    booster::smatch m;
    if(!booster::regex_match(arg,m,r))
        return false;
    std::string key = m[1];
    std::string val = m[2];
    if(val == "true") {
        cfg.set(key,true);
    }
    else if(val== "false") {
        cfg.set(key,false);
    }
    else if(booster::regex_match(val,isnum)) {
        cfg.set(key,atof(val.c_str()));
    }
    else {
        cfg.set(key,val);
    }
    return true;

}

int main(int argc,char **argv)
{   
    try {

        std::string path;
        std::string driver;
        std::string driver_opt;
        std::string astap_exe,astap_db;

        if(argc<2) {
            std::cerr << 
                "Usage ols_cmd config.json [key1=value key2=value ... ]\n" 
                "   for example: ols_cmd config.js wdir.path=/tmp/wd\n";
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
        for(int i=2;i<argc;i++) {
            if(!parse_key_value(argv[i],cfg)) {
                std::cerr << "Invalid parameter " << argv[i] << std::endl;
                return 1;
            }
        }
        driver = cfg.get<std::string>("driver");
        path = cfg.get("libdir","");
        astap_exe = cfg.get("astap.exe","astap_cli");
        astap_db = cfg.get("astap.db","");
        double astap_timeout = cfg.get("astap.timeout",5.0);
        if(driver == "sim") {
            driver_opt = cfg.get("sim.path","");
        }
        else if(driver == "wdir") {
            std::ostringstream ss;
            ss<<cfg["wdir"];
            driver_opt = ss.str();
        }

        char const *driver_opt_ptr = nullptr;
        if(!driver_opt.empty()) {
            driver_opt_ptr = driver_opt.c_str();
        }

        ols::CameraDriver::load_driver(driver,path,driver_opt_ptr);
        ols::PlateSolver::init(astap_db,astap_exe,astap_timeout);
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
