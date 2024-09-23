#include "ols.h"
#include "plate_solver.h"
#include <cppcms/json.h>
#include <booster/regex.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include "allocator.h"

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

//#define DEBUG_ALLOCATOR
#ifdef DEBUG_ALLOCATOR

static cv::MatAllocator *stda = nullptr;



static std::atomic<int> counter_a;
static std::atomic<size_t> memory_use;
static std::atomic<size_t> memory_max;


class MyAlloc : public cv::MatAllocator
{
public:
    virtual cv::UMatData *  allocate (int dims, const int *sizes, int type, void *data, size_t *step, cv::AccessFlag flags, cv::UMatUsageFlags usageFlags) const override
    {
        cv::UMatData *p = stda->allocate(dims,sizes,type,data,step,flags,usageFlags);
        p->currAllocator = this;
        memory_use += p->size;
        memory_max = std::max(size_t(memory_max),size_t(memory_use));
        std::cerr << "A1 " << int(counter_a++) << " total " << memory_use / 1024 / 1024 << " max=" << memory_max / 1024 / 1024<< std::endl;
        return p;
    }
    virtual bool allocate (cv::UMatData *data, cv::AccessFlag accessflags, cv::UMatUsageFlags usageFlags) const override 
    {
        std::cerr << "A2 " << int(counter_a++) << std::endl;
        return stda->allocate(data,accessflags,usageFlags);
    }
    virtual void deallocate   (   cv::UMatData *  data    )   const override
    {
        memory_use -= data->size;
        std::cerr << "D " << int(--counter_a) << " total " << memory_use / 1024 / 1024 << std::endl;
        stda->deallocate(data);
    }

} alloc_inst;

#endif

int main(int argc,char **argv)
{   
    try {

#ifdef DEBUG_ALLOCATOR
        stda = cv::Mat::getStdAllocator();
        cv::Mat::setDefaultAllocator(&alloc_inst);
#endif        

        std::string path;
        std::string driver;
        std::string driver_opt;
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

        char const *driver_opt_ptr = nullptr;
        if(!driver_opt.empty()) {
            driver_opt_ptr = driver_opt.c_str();
        }
       
        ols::AllocatorGuard alloc_guard(mem_limit_mb > 0);
        { 
            ols::CameraDriver::load_driver(driver,path,driver_opt_ptr,cfg.get("camera.log","/tmp/ols_camera.log"),cfg.get("camera.debug",false));
            ols::PlateSolver::init(astap_db,astap_exe);
            ols::OpenLiveStacker stacker;
            stacker.mem_limit_mb = mem_limit_mb;
            stacker.http_ip = cfg.get("http.ip",stacker.http_ip);
            stacker.http_port = cfg.get("http.port",stacker.http_port);
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
