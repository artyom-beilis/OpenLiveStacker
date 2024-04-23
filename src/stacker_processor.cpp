#include "data_items.h"
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
        StackerProcessor(queue_pointer_type in,queue_pointer_type out) :
            in_(in),
            out_(out)
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
                    std::shared_ptr<StackedFrame> res = handle_video(video_ptr);
                    if(out_) {
                        out_->push_or_replace(res);
                    }
                    continue;
                }
                auto config_ptr = std::dynamic_pointer_cast<StackerControl>(data_ptr);
                if(config_ptr) {
                    try {
                        handle_config(config_ptr);
                    }
                    catch(std::exception const &e) {
                        send_message(out_,"Config",e);
                        BOOSTER_ERROR("stacker") << "Config change error:" <<e.what();
                    }
                }
                if(out_)
                    out_->push(data_ptr);
            }
        }

        void send_updated_image()
        {
            if(out_) {
                if(stacker_->stacked_count() > 0) {
                    std::shared_ptr<StackedFrame> res(new StackedFrame());
                    res->frame = stacker_->get_raw_stacked_image();
                    res->roi = stacker_->fully_stacked_area();
                    create_stats(res);
                    out_->push(res);
                }
            }
        }

        void create_stats(std::shared_ptr<StatsBase> stats)
        {
            if(calibration_) {
                stats->stacked = cframe_count_;
            }
            else {
                stats->stacked = stacker_->stacked_count();
                stats->missed  = stacker_->total_count() - stats->stacked;
            }
            stats->dropped = dropped_count_;
        }

        std::shared_ptr<StackedFrame> handle_video(std::shared_ptr<CameraFrame> video)
        {
            std::shared_ptr<StackedFrame> res(new StackedFrame());
		    auto start = std::chrono::high_resolution_clock::now();
            try {
                dropped_count_ += video->dropped;
                if(calibration_) {
                    cframe_ +=  video->processed_frame;
                    cframe_count_ ++;
                    res->frame = cframe_.mul(cv::Scalar::all(1.0 / cframe_count_));
                    auto end = std::chrono::high_resolution_clock::now();
                    double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(end-start).count();
                    BOOSTER_INFO("stacker") << "Stacking took " << (1e3*time) << " ms, calibration frame #" << cframe_count_;
                }
                else {
                    if(stacker_->stack_image(video->processed_frame,restart_)) {
                        restart_ = false;
		                auto p1 = std::chrono::high_resolution_clock::now();
                        double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(p1-start).count();
                        res->frame = stacker_->get_raw_stacked_image();
                        res->roi = stacker_->fully_stacked_area();
                        BOOSTER_INFO("stacker") << "Stacking took " << (1e3*time) << " ms";
                    }
                    else {
                        BOOSTER_INFO("stacker") << "Failed to stack frame";
                        auto end = std::chrono::high_resolution_clock::now();
                        double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(end-start).count();
                        BOOSTER_INFO("stacker") << "Stacking took " << (1e3*time) << " ms";
                        return nullptr;
                    }
                }
                create_stats(res);
            }
            catch(std::exception const &e) {
                BOOSTER_ERROR("stacker") << "Stacking Failed:" << e.what();
                return nullptr;
            }
            return res; 
        }
        void handle_config(std::shared_ptr<StackerControl> ctl)
        {
            switch(ctl->op) {
            case StackerControl::ctl_init:
                width_ = ctl->width;
                height_ = ctl->height;
                mono_ = ctl->mono;
                version_ = 0;
                channels_ = mono_ ? 1 : 3;
                cv_type_ = mono_ ? CV_32FC1 : CV_32FC3;
                calibration_ = ctl->method == stack_calibration;
                dropped_count_ = 0;
                stacker_.reset();
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
                break;
            case StackerControl::ctl_pause:
                restart_ = true;
                if(stacker_) {
                    stacker_->handle_pause();
                    send_updated_image();
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
            default:
                /// not much to do
                ;
            }
        }
    private:
        queue_pointer_type in_,out_;
        int width_,height_;
        bool mono_;
        int version_;
        int channels_,cv_type_;
        bool calibration_=false;
        cv::Mat cframe_;
        int cframe_count_;
        int dropped_count_ = 0;
        std::unique_ptr<Stacker> stacker_;
        bool restart_;
    };

    std::thread start_stacker(queue_pointer_type in,queue_pointer_type out)
    {
        std::shared_ptr<StackerProcessor> p(new StackerProcessor(in,out));
        return std::thread([=]() { p->run(); });
    }

} // namespace
