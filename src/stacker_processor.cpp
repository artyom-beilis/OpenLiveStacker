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

    class StackerProcessor {
    public:
        StackerProcessor(queue_pointer_type in,queue_pointer_type out,queue_pointer_type stats) :
            in_(in),
            out_(out),
            stats_(stats)
        {
        }
        void run()
        {
            while(true) {
                auto data_ptr = in_->pop();
                auto stop_ptr = std::dynamic_pointer_cast<ShutDownData>(data_ptr);
                if(stop_ptr) {
                    if(out_)
                        out_->push(data_ptr);
                    break;
                }
                auto video_ptr = std::dynamic_pointer_cast<CameraFrame>(data_ptr);
                if(video_ptr) {
                    auto frames = handle_video(video_ptr);
                    auto res = frames.first;
                    auto ps  = frames.second;
                    if(res) {
                        if(out_)
                            out_->push(res);
                    }
                    continue;
                }
                auto config_ptr = std::dynamic_pointer_cast<StackerControl>(data_ptr);
                if(config_ptr) {
                    try {
                        handle_config(config_ptr);
                    }
                    catch(std::exception const &e) {
                        send_message(stats_,"Config",e);
                        BOOSTER_ERROR("stacker") << "Config change error:" <<e.what();
                    }
                }
                if(out_)
                    out_->push(data_ptr);
            }
        }

        std::shared_ptr<CameraFrame> generate_dummy_frame()
        {
            return ols::generate_dummy_frame(width_,height_,channels_);
        }
        std::shared_ptr<CameraFrame> generate_output_frame(cv::Mat img)
        {
            cv::Mat img = data.first;
            std::shared_ptr<CameraFrame> frame(new CameraFrame());
            std::shared_ptr<CameraFrame> plate_solving_frame;
            frame->format.width = img.cols;
            frame->format.height = img.rows;
            frame->processed_frame = img.clone();
        }

        std::shared_ptr<StatsData> create_stats()
        {
            std::shared_ptr<StatsData> p(new StatsData());
            StatsData &stats = *p;
            if(calibration_) {
                stats.stacked = cframe_count_;
            }
            else {
                stats.stacked = stacker_->stacked_count();
                stats.missed  = stacker_->total_count() - stats->stacked;
            }
            stats.dropped = dropped_count_;
            return p;
        }

        std::shared_ptr<CameraFrame>  handle_video(std::shared_ptr<CameraFrame> video)
        {
            std::shared_ptr<CameraFrame> res;
		    auto start = std::chrono::high_resolution_clock::now();
            try {
                dropped_count_ += video->dropped;
                if(calibration_) {
                    cframe_ +=  video->processed_frame;
                    cframe_count_ ++;
                    res = video;
                    auto end = std::chrono::high_resolution_clock::now();
                    double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(end-start).count();
                    BOOSTER_INFO("stacker") << "Stacking took " << (1e3*time) << " ms, calibration frame #" << cframe_count_;
                }
                else {
                    if(stacker_->stack_image(video->processed_frame,restart_)) {
                        restart_ = false;
                        res = generate_output_frame(stacker_->get_raw_stacked_image())
		                auto p1 = std::chrono::high_resolution_clock::now();
                        double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(p1-start).count();
                        BOOSTER_INFO("stacker") << "Stacking took " << (1e3*time) << " ms";
                    }
                    else {
                        BOOSTER_INFO("stacker") << "Failed to stack frame";
                        auto end = std::chrono::high_resolution_clock::now();
                        double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(end-start).count();
                        BOOSTER_INFO("stacker") << "Stacking took " << (1e3*time) << " ms";
                    }
                }
                if(stats_) {
                    stats_->push(create_stats());
                }
            }
            catch(std::exception const &e) {
                send_message(stats_,"Stacking",e);
                BOOSTER_ERROR("stacker") << "Stacking Failed:" << e.what();
            }
            return res;
        }
        void send_updated_image()
        {
            if(out_) {
                if(stacker_->stacked_count() > 0) {
                    auto frame = generate_output_frame(stacker_->get_stacked_image());
                    out_->push(frame);
                }
                else
                    out_->push(generate_dummy_frame());
            }
        }
        void handle_config(std::shared_ptr<StackerControl> ctl)
        {
            switch(ctl->op) {
            case StackerControl::ctl_init:
                width_ = ctl->width;
                height_ = ctl->height;
                mono_ = ctl->mono;
                saved_count_ = 0;
                version_ = 0;
                channels_ = mono_ ? 1 : 3;
                cv_type_ = mono_ ? CV_32FC1 : CV_32FC3;
                calibration_ = ctl->calibration;
                output_path_ = ctl->output_path;
                name_ = ctl->name;
                dropped_count_ = 0;
                stacker_.reset();
                stack_info_ = *ctl;
                if(calibration_) {
                    cframe_ = cv::Mat(height_,width_,cv_type_);
                    cframe_.setTo(0);
                    cframe_count_ = 0;
                }
                else {
                    stacker_.reset(new Stacker(width_,height_,channels_));
                    stacker_->set_remove_satellites(ctl->remove_satellites);
                    stacker_->set_rollback_on_pause(ctl->rollback_on_pause);
                    restart_ = true;
                }
                if(out_)
                    out_->push(generate_dummy_frame());
                if(stats_) {
                    std::shared_ptr<StatsData> stats(new StatsData());
                    stats_->push(stats);
                }
                break;
            case StackerControl::ctl_pause:
                restart_ = true;
                if(stacker_) {
                    stacker_->handle_pause();
                    send_updated_image();
                }
                if(stats_) {
                    stats_->push(create_stats());
                }
                break;
            case StackerControl::ctl_cancel:
                if(stacker_) {
                    stacker_.reset();
                }
                else if(calibration_) {
                    calibration_ = false;
                }
                break;
            case StackerControl::ctl_save:
                if(stacker_) {
                    save_stacked_image_and_send();
                    saved_count_ = stacker_->stacked_count();
                }
                else if(calibration_) {
                    save_calibration();
                    saved_count_ = cframe_count_;
                }
                if(stats_) {
                    stats_->push(create_stats());
                }
                break;
            case StackerControl::ctl_update:
                if(stacker_) {
                    stacker_->set_stretch(ctl->auto_stretch,ctl->stretch_low,ctl->stretch_high,ctl->stretch_gamma);
                    BOOSTER_INFO("stacker") << "Getting to stretch settings in stacker auto="<<ctl->auto_stretch << " low="<<ctl->stretch_low << " high=" << ctl->stretch_high << " gamma=" << ctl->stretch_gamma;
                    send_updated_image();
                }
                break;
            default:
                /// not much to do
                ;
            }
        }
    private:
        queue_pointer_type in_,out_,stats_;
        int width_,height_;
        bool mono_;
        int version_;
        int channels_,cv_type_;
        bool calibration_=false;
        std::string output_path_,name_;
        cv::Mat cframe_;
        int cframe_count_;
        int dropped_count_ = 0;
        std::unique_ptr<Stacker> stacker_;
        bool restart_;
        int saved_count_ = 0;
        StackerControl stack_info_;
    };

    std::thread start_stacker(queue_pointer_type in,queue_pointer_type out,queue_pointer_type stats)
    {
        std::shared_ptr<StackerProcessor> p(new StackerProcessor(in,out,stats,plate_solving,data_dir));
        return std::thread([=]() { p->run(); });
    }
} // ols
