#include "data_items.h"
#include "rotation.h"
#include "stacker.h"
#include "tiffmat.h"
#include "common_utils.h"
#include <booster/log.h>
#include "processors.h"
#include <cppcms/json.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/hal/intrin.hpp>
#include <fstream>
#include <iomanip>
#include <chrono>
#include "util.h"

#include "simd_utils.h"

namespace ols {

    class DebugSaver {
    public:
        DebugSaver(queue_pointer_type in,queue_pointer_type err,std::string output_dir) :
            in_(in),
            err_(err),
            out_(output_dir),
            counter_(0),
            save_(false)
        {
        }
        void run()
        {
            while(true) {
                auto data_ptr = in_->pop();
                auto stop_ptr = std::dynamic_pointer_cast<ShutDownData>(data_ptr);
                if(stop_ptr) {
                    break;
                }
                auto video_ptr = std::dynamic_pointer_cast<CameraFrame>(data_ptr);
                if(save_ && video_ptr) {
                    try {
                        handle_video(video_ptr);
                    }
                    catch(std::exception const &e) {
                        send_message(err_,"debug save",e);
                    }
                    continue;
                }
                auto config_ptr = std::dynamic_pointer_cast<StackerControl>(data_ptr);
                if(config_ptr) {
                    try {
                        handle_config(config_ptr);
                    }
                    catch(std::exception const &e) {
                        send_message(err_,"debug config",e);
                    }

                }
            }
        }
        std::string log_file()
        {
            return dirname_ + "/log.txt";
        }
        void handle_video(std::shared_ptr<CameraFrame> video)
        {
            char fname[256];
            snprintf(fname,sizeof(fname),"frame_%08d",counter_);
            std::string base_name = dirname_ + "/" + fname;
            if(video->format.format == stream_mjpeg) {
                std::ofstream f(base_name + ".jpeg",std::ofstream::binary);
                f.write((char*)video->source_frame->data(),video->source_frame->size());
                if(!f) {
                    BOOSTER_ERROR("stacker") << "Failed to save jpeg to " << base_name << ".jpeg ";
                    send_message(err_,"save jpeg","Failed to save jpeg to "  + base_name + ".jpeg");
                }
                f.close();
            }
            else {
                try {
                    save_tiff(video->raw,base_name + ".tiff");
                }
                catch(std::exception const &e){
                    send_message(err_,"save tiff",e.what());
                    BOOSTER_ERROR("stacker") << "Failed to save tiff to " << base_name << ".tiff: " << e.what();
                }
            }
            std::ofstream log(log_file(),std::ofstream::app);
            log << counter_ <<"," << std::fixed << std::setprecision(3) << video->timestamp << std::endl;
            counter_++;
            if(counter_ == 1) {
                cppcms::json::value v;
                std::ifstream info_r(dirname_ + "/info.json");
                if(v.load(info_r,true)) {
                    info_r.close();
                    v["format"] = stream_type_to_str(video->format.format);
                    v["bayer"] = bayer_type_to_str(video->bayer);
                    std::ofstream info(dirname_ + "/info.json");
                    if(!info) {
                        BOOSTER_ERROR("stacker") << "Failed to update JSON info from "<< dirname_ << "/info.json";
                        send_message(err_,"update info","Failed to update JSON info from " + dirname_ + "/info.json");
                    }
                    else {
                        v.save(info,cppcms::json::readable);
                    }
                }
                else {
                    BOOSTER_ERROR("stacker") << "Failed to read JSON info from "<< dirname_ << "/info.json";
                    send_message(err_,"read info","Failed to read info from " + dirname_ + "/info.json");
                }
            }
        }
        void handle_config(std::shared_ptr<StackerControl> ctl)
        {
            switch(ctl->op) {
            case StackerControl::ctl_init:
                {
                    save_ = ctl->save_inputs;
                    if(!save_)
                        return;
                    dirname_ = out_ + "/" + ctl->name;
                    counter_ = 0;
                    make_dir(dirname_);
                    cppcms::json::value v;
                    v["name"]=ctl->name;
                    v["width"] = ctl->width;
                    v["height"] = ctl->height;
                    v["mono"] = ctl->mono;
                    v["synthetic_exposure_mpl"] = ctl->synthetic_exposure_mpl;
                    v["darks"] = ctl->darks_path;
                    v["flats"] = ctl->flats_path;
                    v["dark_flats"] = ctl->dark_flats_path;
                    v["calibration"] = ctl->calibration;
                    v["derotate"] = ctl->derotate;
                    v["derotate_mirror"] = ctl->derotate_mirror;
                    v["ra"] = ctl->ra;
                    v["de"] = ctl->de;
                    v["lat"] = ctl->lat;
                    v["lon"] = ctl->lon;
                    v["source_gamma"] = ctl->source_gamma;
                    v["auto_stretch"] = ctl->auto_stretch;
                    v["stretch_low"] = ctl->stretch_low;
                    v["stretch_high"] = ctl->stretch_high;
                    v["stretch_gamma"] = ctl->stretch_gamma;
                    v["remove_satellites" ] = ctl->remove_satellites;
                    std::ofstream info(dirname_ + "/info.json");
                    v.save(info,cppcms::json::readable);
                }
                break;
            case StackerControl::ctl_update:
                {
                    if(!save_)
                        return;
                    std::ifstream info_r(dirname_ + "/info.json");
                    cppcms::json::value v;
                    if(v.load(info_r,true)) {
                        info_r.close();
                        std::ofstream info(dirname_ + "/info.json");
                        v["auto_stretch"] = ctl->auto_stretch;
                        v["stretch_low"] = ctl->stretch_low;
                        v["stretch_high"] = ctl->stretch_high;
                        v["stretch_gamma"] = ctl->stretch_gamma;
                        v.save(info,cppcms::json::readable);
                    }
                }
                break;
            case StackerControl::ctl_pause:
                {
                    if(!save_)
                        return;
                    std::ofstream log(log_file(),std::ofstream::app);
                    log << "PAUSE,0" << std::endl;
                }
                break;

            default:
                /// not much to do
                ;
            }
        }
    private:

        queue_pointer_type in_,err_;
        std::string out_;
        std::string dirname_;
        int counter_;
        bool save_;
    };

    std::thread start_debug_saver(queue_pointer_type in,queue_pointer_type err,std::string debug_dir)
    {
        std::shared_ptr<DebugSaver> p(new DebugSaver(in,err,debug_dir));
        return std::thread([=]() { p->run(); });
    }

}
