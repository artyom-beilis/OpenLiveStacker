#include "data_items.h"
#include "tiffmat.h"
#include "processors.h"
#include "util.h"
#include <cppcms/json.h>
#include <booster/log.h>
#include <iostream>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "hot_removal.h"

#include <unistd.h>

namespace ols {
    class Simulator {
    public:
        Simulator(std::string const &dir,std::string output_dir) :
            dir_(dir),
            output_dir_(output_dir)
        {
            load_config();
        }

        void run(bool run_pp)
        {
            static int counter = 0;
            queue_pointer_type out;
            if(run_pp) {
                out = std::shared_ptr<queue_type>(new queue_type());
                out->call_on_push([&](std::shared_ptr<QueueData> p){
                    std::shared_ptr<CameraFrame> frame = std::dynamic_pointer_cast<CameraFrame>(p);
                    if(!frame || !frame->jpeg_frame)
                        return;
                    std::ofstream tmp("/tmp/frame_" + std::to_string(counter) + ".jpeg");
                    tmp.write((char*)frame->jpeg_frame->data(),frame->jpeg_frame->size());
                    tmp.close();
                    counter++;
                });
            }
            std::thread t1 = start_preprocessor(input_queue_,stacker_queue_,nullptr);
            std::thread t2 = start_stacker(stacker_queue_,pp_queue_);
            std::thread t3 = start_post_processor(pp_queue_,out,nullptr,nullptr,output_dir_);
            std::shared_ptr<StackerControl> ctl(new StackerControl(cfg_));
            std::cerr << cfg_.width << " " << cfg_.height << std::endl;
            input_queue_->push(ctl);
            try {
                load_frames(run_pp);
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
            t3.join();
        }

        std::string file_name(int frame_id,std::string const &ext)
        {
            char name[256];
            snprintf(name,sizeof(name),"/frame_%08d.%s",frame_id,ext.c_str());
            return dir_ + name;
        }

        void load_frames(bool run_pp)
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
                        if(cfg_.mono || bayer_ != bayer_na) {
                            if(cfg_.remove_hot_pixels){
                                img = remove_hot_pixels(img);
                            }
                        }
                        if(bayer_ != bayer_na) {
                            cv::Mat rgb;
                            switch(bayer_) {
                            case bayer_rg:  cv::cvtColor(img,rgb,cv::COLOR_BayerBG2BGR); break; // COLOR_BayerRGGB2BGR = COLOR_BayerBG2BGR
                            case bayer_gr:  cv::cvtColor(img,rgb,cv::COLOR_BayerGB2BGR); break; // COLOR_BayerGRBG2BGR = COLOR_BayerGB2BGR
                            case bayer_bg:  cv::cvtColor(img,rgb,cv::COLOR_BayerRG2BGR); break; // COLOR_BayerBGGR2BGR = COLOR_BayerRG2BGR
                            case bayer_gb:  cv::cvtColor(img,rgb,cv::COLOR_BayerGR2BGR); break; // COLOR_BayerGBRG2BGR = COLOR_BayerGR2BGR
                            case bayer_na:  cv::cvtColor(img,rgb,cv::COLOR_GRAY2BGR); break;    // handle indigo misreporting mono as raw
                            default:
                                BOOSTER_ERROR("stacker") << "Invalid bayer patter";
                            }
                            img = rgb;
                        }
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
                    if(run_pp && cfg_.method == stack_dynamic) {
                        std::shared_ptr<StackerControl> ctl(new StackerControl());
                        ctl->op = StackerControl::ctl_save;
                        input_queue_->push(ctl);
                    }
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
            cfg.mono = v.get<bool>("mono",false);
            cfg.synthetic_exposure_mpl = v.get<int>("synthetic_exposure_mpl",cfg.synthetic_exposure_mpl);
            std::string method = v.get("method","undefined");
            if(method == "undefined")
                cfg.method = v.get<bool>("calibration") ? stack_calibration : stack_dso;
            else if(method == "calibration")
                cfg.method = stack_calibration;
            else if(method == "dso")
                cfg.method = stack_dso;
            else if(method == "planetary")
                cfg.method = stack_planetary;
            else if(method == "dynamic")
                cfg.method = stack_dynamic;
            else
                throw std::runtime_error("Invalid stack method " + method);
            cfg.derotate = v.get<bool>("derotate");
            cfg.derotate_mirror = v.get<bool>("derotate_mirror");
            cfg.remove_gradient = v.get<bool>("remove_gradient",false);
            cfg.remove_hot_pixels = v.get<bool>("remove_hot_pixels",false);
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
            cfg.remove_satellites = v.get("remove_satellites",cfg.remove_satellites);
            
            cfg.filters.remove_first=v.get("filters.remove_first",cfg.filters.remove_first);
            cfg.filters.min_stat_size=v.get("filters.min_stat_size",cfg.filters.min_stat_size);
            cfg.filters.dynamic_delay=v.get("filters.dynamic_delay",cfg.filters.min_stat_size);
            cfg.filters.sharpness_percentile=v.get("filters.sharpness_percentile",cfg.filters.sharpness_percentile);
            cfg.filters.reg_score_percentile=v.get("filters.reg_score_percentile",cfg.filters.reg_score_percentile);
            cfg.filters.avg_brightness_sigma=v.get("filters.avg_brightness_sigma",cfg.filters.avg_brightness_sigma);


            bayer_ = bayer_type_from_str(v.get<std::string>("bayer","NA"));

            if(cfg.method != stack_calibration)
                cfg.output_path =  output_dir_ + "/" + cfg.name;
            else
                cfg.output_path =  output_dir_;
            cfg_ = cfg;
        }

        std::string dir_,output_dir_;
       
        // limit queue size for offline processing 
        queue_pointer_type input_queue_      = std::shared_ptr<queue_type>(new queue_type(10));
        queue_pointer_type stacker_queue_    = std::shared_ptr<queue_type>(new queue_type(10));
        queue_pointer_type pp_queue_         = std::shared_ptr<queue_type>(new queue_type(10));
        StackerControl cfg_;
        CamBayerType bayer_;
    };
}

int main(int argc,char **argv)
{
    bool run_pp = false;
    int threads = -1;
    while(argc >= 2 && argv[1][0]=='-' && argv[1][1] != 0) {
        char code = argv[1][1];
        switch(code) {
        case 'p':
            run_pp = true;
            break;
        case 't':
            threads = atoi(argv[1]+2);
            break;
        default:
            fprintf(stderr,"Invalid parameter %c",code);
            return 1;
        }
        argc--;
        argv++;
    }
    if(argc !=3) {
        std::cerr << "Usage [-p] [-tN] debug_dir output_dir " << std::endl;
        return 1;
    }
    try {
        if(threads != -1)
            cv::setNumThreads(threads);
        booster::log::logger::instance().set_default_level(booster::log::debug);
        booster::log::logger::instance().add_sink(std::make_shared<booster::log::sinks::standard_error>());
        ols::Simulator sim(argv[1],argv[2]);
        sim.run(run_pp);
    }
    catch(std::exception const &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
