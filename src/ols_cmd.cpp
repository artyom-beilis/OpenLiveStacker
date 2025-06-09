#include "ols.h"
#include "plate_solver.h"
#include <cppcms/json.h>
#include <booster/regex.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include "allocator.h"

#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
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

class ServerGuard {
public:
    ServerGuard(ServerGuard const &) = delete;
    void operator=(ServerGuard const &) = delete;
    ServerGuard(std::string const &s) : cmd_(s)
    {
        if(cmd_.empty())
            return;
        std::cout << "Running " << cmd_ << std::endl;
        std::vector<char *> parts;
        for(char *part = strtok(&cmd_[0]," ");part;part = strtok(nullptr," ")) {
            parts.push_back(part);
        }
        parts.push_back(nullptr);
        if(parts.size() < 2) {
            throw std::runtime_error("Invalid command " + s);
        }
        int child_pid = fork();
        if(child_pid < 0) {
            throw std::runtime_error("Failed to fork");
        }
        else if(child_pid > 0) {
            child_pid_ = child_pid;
        }
        else {
            execvp(parts[0],&parts[0]);
            perror("execv");
            std::cerr << "Server failed to start" << std::endl;
            exit(1);
        }

    }
    ~ServerGuard()
    {
        if(child_pid_ != -1) {
            kill(child_pid_,SIGTERM);
            int count = 0;
            while(true) {
                if(count > 50) {
                    kill(child_pid_,SIGKILL);
                }
                int status;
                int res = 0;
                if((res = waitpid(child_pid_,&status,WNOHANG)) > 0)
                    break;
                if(res < 0) {
                    if(errno == ECHILD)
                        break;
                    if(errno == EINTR)
                        continue;
                    break;
                } 
                usleep(100000);
                count++;
            }
        }
    }
private:
    std::string cmd_;
    int child_pid_ = -1;
};


int main(int argc,char **argv)
{   
    try {

        std::string path;
        std::string driver;
        std::string driver_opt;
        std::string server_cmd;
        std::string astap_exe,astap_db;
        int mem_limit_mb = 0;

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
        mem_limit_mb = cfg.get<int>("mem_limit_mb",0);
        driver = cfg.get<std::string>("driver");
        std::string libdir = cfg.get<std::string>("indi.libdir","./");
        path = cfg.get("libdir","");
        astap_exe = cfg.get("astap.exe","astap_cli");
        astap_db = cfg.get("astap.db","");
        if(driver == "sim") {
            driver_opt = cfg.get("sim.path","");
        }
        else if(driver == "wdir") {
            std::ostringstream ss;
            ss<<cfg["wdir"];
            driver_opt = ss.str();
        }
        else if(driver == "toup") {
            bool oem = cfg.get("toup.oem",false);
            if(oem)
                driver_opt = "oem";
        }
        else if(driver == "indigo") {
            driver_opt = cfg.get("indigo.connection","");
        }
        else if(driver == "indi") {
            driver_opt = cfg.get("indi.connection","");
        }
        server_cmd = cfg.get("start_cmd",server_cmd);

        std::string data_dir = cfg.get("data_dir","./data");
        setenv("OLS_DATA_DIR",data_dir.c_str(),1);
        std::string www_dir = cfg.get("www_dir","./www-data");
        
        char const *driver_opt_ptr = nullptr;
        if(!driver_opt.empty()) {
            driver_opt_ptr = driver_opt.c_str();
        }

        ServerGuard sg(server_cmd);
       
        ols::AllocatorGuard alloc_guard(mem_limit_mb > 0);
        {
            ols::CameraDriver::load_driver(driver,path,driver_opt_ptr,cfg.get("camera.log","/tmp/ols_camera.log"),cfg.get("camera.debug",false));
            ols::PlateSolver::init(astap_db,astap_exe);
            ols::OpenLiveStacker stacker(data_dir);
            stacker.config_libdir(libdir);
            if(!libdir.empty()) {
                setenv("INDI_MATH_PLUGINS_DIRECTORY",libdir.c_str(),1);
            }
            stacker.mem_limit_mb = mem_limit_mb;
            stacker.http_ip = cfg.get("http.ip",stacker.http_ip);
            stacker.http_port = cfg.get("http.port",stacker.http_port);
            stacker.document_root = www_dir;
            stacker.init(driver);
            stacker.run();
            stacker.shutdown();
        }
    }
    catch(std::exception const &e) {
        std::cerr << "failed:" << e.what() << std::endl;
        return 1;
    }
    std::cout << "Done" << std::endl;
    return 0;
}
