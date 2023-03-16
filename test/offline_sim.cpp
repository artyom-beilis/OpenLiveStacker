#include "data_items.h"
#include "tiffmat.h"
#include "processors.h"
#include "util.h"
#include <cppcms/json.h>
#include <booster/log.h>
#include <iostream>
#include <fstream>
#include <opencv2/imgcodecs.hpp>


namespace ols {
    class Simulator {
    public:
        Simulator(std::string const &dir,std::string output_dir) :
            dir_(dir),
            output_dir_(output_dir)
        {
            load_config();
        }

        void run()
        {
            std::thread t1 = start_preprocessor(input_queue_,stacker_queue_);
            std::thread t2 = start_stacker(stacker_queue_,nullptr,nullptr,output_dir_);
            std::shared_ptr<StackerControl> ctl(new StackerControl(cfg_));
            std::cerr << cfg_.width << " " << cfg_.height << std::endl;
            input_queue_->push(ctl);
            try {
                load_frames();
                ctl.reset(new StackerControl());
                ctl->op = StackerControl::ctl_save;
                input_queue_->push(ctl);
            }
            catch(std::exception &e) {
                std::cerr << "Failed " << e.what() << std::endl;
            }
            std::shared_ptr<ShutDownData> end(new ShutDownData());
            input_queue_->push(end);
            t1.join();
            t2.join();
        }

        std::string file_name(int frame_id,std::string const &ext)
        {
            char name[256];
            snprintf(name,sizeof(name),"/frame_%08d.%s",frame_id,ext.c_str());
            return dir_ + name;
        }

        void load_frames()
        {
            std::ifstream data(dir_  + "/log.txt");
            if(!data)
                throw std::runtime_error("Failed to read log file");
            std::string ext;
            std::string str;
            while(std::getline(data,str)) {
                size_t pos = str.find(',');
                std::string op = str.substr(0,pos);
                if(op == "PAUSE") {
                    std::shared_ptr<StackerControl> ctl(new StackerControl());
                    ctl->op = StackerControl::ctl_pause;
                    input_queue_->push(ctl);
                }
                else {
                    int frame_id = atoi(op.c_str());
                    double timestamp = atof(str.substr(pos+1).c_str());
                    if(ext.empty()) {
                        if(exists(file_name(frame_id,"jpeg")))
                            ext="jpeg";
                        else
                            ext="tiff";
                    }
                    std::string path = file_name(frame_id,ext);
                    cv::Mat img;
                    int dr;
                    if(ext == "tiff") {
                        img = load_tiff(path);
                        dr = (1ll << (8*img.elemSize1())) - 1;
                    }
                    else {
                        img = cv::imread(path);
                        dr = 255;
                    }
                    std::shared_ptr<CameraFrame> frame(new CameraFrame);
                    frame->frame = img;
                    frame->frame_dr = dr;
                    frame->timestamp = timestamp;
                    BOOSTER_INFO("stacker") << "Loaded image " << (path) << " " << img.rows<<"x"<<img.cols << std::endl;
                    input_queue_->push(frame);
                }
            }
            

        }

        void load_config()
        {
            std::ifstream cfg_file(dir_ + "/info.json");
            cppcms::json::value v;
            if(!v.load(cfg_file,true))
                throw std::runtime_error("Failed to load config");

            StackerControl cfg;
            cfg.name = v.get<std::string>("name");
            cfg.width = v.get<int>("width");
            cfg.height = v.get<int>("height");
            cfg.calibration = v.get<bool>("calibration");
            cfg.derotate = v.get<bool>("derotate");
            cfg.derotate_mirror = v.get<bool>("derotate_mirror");
            cfg.ra = v.get<double>("ra");
            cfg.de = v.get<double>("de");
            cfg.lat = v.get<double>("lat");
            cfg.lon = v.get<double>("lon");
            cfg.source_gamma = v.get<double>("source_gamma");
            cfg.darks_path = v.get<std::string>("darks");
            cfg.flats_path = v.get<std::string>("flats","");
            cfg.dark_flats_path = v.get<std::string>("dark_flats","");
            cfg.auto_stretch = v.get<bool>("auto_stretch");
            cfg.stretch_low = v.get<double>("stretch_low");
            cfg.stretch_high = v.get<double>("stretch_high");
            cfg.stretch_gamma = v.get<double>("stretch_gamma");

            if(!cfg.calibration)
                cfg.output_path =  output_dir_ + "/" + cfg.name;
            else
                cfg.output_path =  output_dir_;
            cfg_ = cfg;
        }

        std::string dir_,output_dir_;
       
        // limit queue size for offline processing 
        queue_pointer_type input_queue_      = std::shared_ptr<queue_type>(new queue_type(10));
        queue_pointer_type stacker_queue_    = std::shared_ptr<queue_type>(new queue_type(10));
        StackerControl cfg_;
    };
}

int main(int argc,char **argv)
{
    if(argc !=3) {
        std::cerr << "Usage debug_dir output_dir " << std::endl;
        return 1;
    }
    try {
        booster::log::logger::instance().set_default_level(booster::log::debug);
        booster::log::logger::instance().add_sink(std::make_shared<booster::log::sinks::standard_error>());
        ols::Simulator sim(argv[1],argv[2]);
        sim.run();
    }
    catch(std::exception const &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
