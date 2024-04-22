#include "data_items.h"
#include "post_processor.h"
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
    class PostProcessorProcessor {
    public:
        PostProcessorProcessor(queue_pointer_type in,queue_pointer_type out,queue_pointer_type stats,queue_pointer_type plate_solving,std::string data_dir) :
            in_(in),
            out_(out),
            stats_(stats),
            plate_solving_(plate_solving),
            data_dir_(data_dir),
            pp_(new PostProcessor())
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
                auto video_ptr = std::dynamic_pointer_cast<StackedFrame>(data_ptr);
                if(video_ptr) {
                    auto frames = handle_video(video_ptr);
                    auto res = frames.first;
                    auto ps  = frames.second;
                    if(res) {
                        if(out_)
                            out_->push(res);
                    }
                    if(ps && plate_solving_) {
                        plate_solving_->push(ps);
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
                auto error_ptr = std::dynamic_pointer_cast<ErrorNotificationData>(data_ptr);
                if(error_ptr){
                    if(stats_) {
                        stats_->push(error_ptr);
                        continue;
                    }
                }
                if(out_)
                    out_->push(data_ptr);
            }
        }

        void save_stretch(StretchInfo const &stretch)
        {
            cppcms::json::value s;
            s["gain"]   = stretch.gain;
            s["cut"]    = stretch.cut;
            s["gamma" ] = stretch.gamma;
            s["auto_stretch"] = stretch.auto_stretch;
            std::string fname = data_dir_ + "/stretch.json";
            std::ofstream f(fname);
            if(!f) {
                BOOSTER_ERROR("stacker") << " Failed to save info to " << fname;
                return;
            }
            s.save(f,cppcms::json::readable);
            f.close();
        }
        

        std::pair<std::shared_ptr<CameraFrame>,std::shared_ptr<CameraFrame> > generate_output_frame(std::pair<cv::Mat,StretchInfo> data,bool create_ps_frame=true)
        {
            cv::Mat img = data.first;
            save_stretch(data.second);
            cv::Mat img8;
            img.convertTo(img8,CV_8UC3,255);
            std::shared_ptr<CameraFrame> frame(new CameraFrame());
            std::shared_ptr<CameraFrame> plate_solving_frame;
            frame->format.width = img8.cols;
            frame->format.height = img8.rows;
            std::vector<unsigned char> buf;
            cv::imencode(".jpeg",img8,buf);
            frame->jpeg_frame = std::shared_ptr<VideoFrame>(new VideoFrame(buf.data(),buf.size()));
            if(plate_solving_ && create_ps_frame) {
                plate_solving_frame.reset(new CameraFrame());
                plate_solving_frame->format.width = img8.cols;
                plate_solving_frame->format.height = img8.rows;
                plate_solving_frame->frame = img8;
                plate_solving_frame->frame_dr = 255;
            }
            return std::make_pair(frame,plate_solving_frame);
        }

        cv::Mat to16bit(cv::Mat m)
        {
            cv::Mat m2 = cv::max(0,m);
            double max_v;
            cv::minMaxLoc(m2,nullptr,&max_v);
            cv::Mat res;
            m2.convertTo(res,m.channels() == 3 ? CV_16UC3 : CV_16UC1,65535/max_v);
            return res;
        }


        void create_meta(std::ostream &m)
        {
            int frames = last_frame_->stacked;
            double exp = get_exp_s();
            m << "Name            " << name_ << std::endl;
            m << "Time            " << timestamp() << std::endl;
            m << "Frames          " << frames  << std::endl;
            m << "True Frames     " << frames * stack_info_.synthetic_exposure_mpl << std::endl;
            m << "Integration (s) " << frames * exp << std::endl;
            m << "Frame:"<<std::endl;
            m << " - format " << stack_info_.format << std::endl;
            m << " - width  " << stack_info_.width << std::endl;
            m << " - height " << stack_info_.height << std::endl;
            m << " - bin    " << stack_info_.bin << std::endl;
            m << " - mono   " << stack_info_.mono << std::endl;
            m << "Stackig:"<<std::endl;
            m << " - remove_satellites  " << stack_info_.remove_satellites << std::endl;
            m << " - derotate           " << stack_info_.derotate << std::endl;
            m << " - derotate_mirror    " << stack_info_.derotate_mirror << std::endl;
            m << " - remove_gradient    " << stack_info_.remove_gradient << std::endl;
            m << "Target:" << std::endl;
            m << " - RA " << stack_info_.ra << std::endl;
            m << " - DE " << stack_info_.de << std::endl;
            m << "Geolocation" << std::endl;
            m << " - lat " << stack_info_.lat << std::endl;
            m << " - lon " << stack_info_.lon << std::endl;
            m << "Camera Settings:"<<std::endl;
            for(auto op:stack_info_.camera_config) {
                m << " - " << cam_option_id_to_name(op.first) << ": " << op.second << std::endl;
            }
        }
        void save_stacked_image_and_send()
        {
            version_++;
            std::string base_name = output_path_ + "_stacked_v" + std::to_string(version_);
            std::string path = base_name + ".jpeg";
            std::string ipath = base_name + ".txt";
            std::string tpath = base_name + ".tiff";
            save_tiff(to16bit(last_frame_->frame),tpath);
            auto img = pp_->post_process_image(last_frame_->frame,last_frame_->roi);
            auto frames = generate_output_frame(img,false);
            auto frame = frames.first;
            std::ofstream f(path,std::ofstream::binary);
            f.write((char*)frame->jpeg_frame->data(),frame->jpeg_frame->size());
            f.close();
            if(out_)
                out_->push(frame);
            std::ofstream log(ipath);
            create_meta(log);
        }

        std::string timestamp()
        {
            return ftime("%Y-%m-%d %H:%M:%S",time(nullptr));
        }

        double get_exp_s()
        {
            return stack_info_.camera_config[opt_exp] * 1e-3 * stack_info_.synthetic_exposure_mpl; // ms to s
        }

        std::shared_ptr<StatsData> create_stats()
        {
            std::shared_ptr<StatsData> stats(new StatsData(*last_frame_));
            stats->since_saved_s = get_exp_s() * (stats->stacked - saved_count_);
            if(stats->stacked>0 && !calibration_) {
                stats->histogramm = std::move(pp_->get_histogramm());
            }
            return stats;
        }

        std::pair<std::shared_ptr<CameraFrame>,std::shared_ptr<CameraFrame> > handle_video(std::shared_ptr<StackedFrame> video)
        {
            last_frame_ = video;
            std::shared_ptr<CameraFrame> res;
            std::shared_ptr<CameraFrame> ps;
            try {
                if(out_ && last_frame_) {
                    auto p1 = std::chrono::high_resolution_clock::now();
                    std::pair<cv::Mat,StretchInfo> img;
                    if(calibration_)
                        img.first=last_frame_->frame;
                    else
                        img = pp_->post_process_image(last_frame_->frame,last_frame_->roi);
                    auto p2 = std::chrono::high_resolution_clock::now();
                    auto frames = generate_output_frame(img);
                    res=frames.first;
                    ps=frames.second;
                    auto p3 = std::chrono::high_resolution_clock::now();
                    double gtime = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(p2-p1).count();
                    double jtime = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(p3-p2).count();
                    double total_time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(p3-p1).count();
                    BOOSTER_INFO("stacker") << "post processing took " << (1e3 *total_time) <<"ms. proccessing " << (1e3*gtime) << " ms, jpeg took=" << (1e3*jtime);
                }
                if(stats_ && last_frame_) {
                    stats_->push(create_stats());
                }
            }
            catch(std::exception const &e) {
                send_message(stats_,"Stacking",e);
                BOOSTER_ERROR("stacker") << "Stacking Failed:" << e.what();
            }
            return std::make_pair(res,ps);
        }
        void save_calibration()
        {
            cv::Mat calib = last_frame_->frame;
            double minV,maxV;
            cv::minMaxLoc(calib,&minV,&maxV);
            std::string tiff_path = output_path_ + "/" + name_ + ".tiff";
            std::string db_path = output_path_ + "/index.json";
            BOOSTER_INFO("stacker") << "Saving calibration frame to " << tiff_path << " frame " << last_frame_->stacked << " maxv=" << maxV << " minv=" << minV;
            cppcms::json::value setup;
            setup["id"] = name_;
            setup["path"] = name_ + ".tiff";
            setup["date"] = timestamp();
            setup["frames"] = last_frame_->stacked;
            setup["width"] = calib.cols;
            setup["height"] = calib.rows;
            BOOSTER_INFO("stacker") << "Saving calibration frame to " << tiff_path;
            save_tiff(calib,tiff_path);
            cppcms::json::value db;
            std::ifstream indx(db_path);
            bool found=false;
            if(!db.load(indx,true)) {
                BOOSTER_ERROR("stacker") << "Error parsing " << db_path << " darks DB file " << std::endl;
                send_message(stats_,"save calibration","Error parsing calibrarion index file:" + db_path);
                db = cppcms::json::value();
                db[0] = setup;
                found = true;
            }
            cppcms::json::array &ar=db.array();
            for(size_t i=0;!found && i<ar.size();i++) {
                if(ar[i].get<std::string>("id") == name_) {
                    ar[i] = setup;
                    found = true;
                    break;
                }
            }
            if(!found)
                ar.push_back(setup);
            std::ofstream res(db_path);
            db.save(res,cppcms::json::readable);
            res.close();
        }
        std::shared_ptr<CameraFrame> generate_dummy_frame()
        {
            return ols::generate_dummy_frame(width_,height_,mono_ ? 1 : 3);
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
                calibration_ = ctl->calibration;
                output_path_ = ctl->output_path;
                name_ = ctl->name;
                dropped_count_ = 0;
                stack_info_ = *ctl;
                last_frame_ = nullptr;
                pp_->set_stretch(ctl->auto_stretch,ctl->stretch_low,ctl->stretch_high,ctl->stretch_gamma);
                if(out_) {
                    out_->push(generate_dummy_frame());
                }
                if(stats_) {
                    std::shared_ptr<StatsData> stats(new StatsData());
                    stats_->push(stats);
                }
                break;
            case StackerControl::ctl_cancel:
                last_frame_ = nullptr;
                break;
            case StackerControl::ctl_save:
                if(last_frame_) {
                    if(calibration_) {
                        save_calibration();
                    }
                    else {
                        save_stacked_image_and_send();
                    }
                    saved_count_ = last_frame_->stacked;
                    if(stats_) {
                        stats_->push(create_stats());
                    }
                }
                break;
            case StackerControl::ctl_update:
                if(!calibration_) {
                    pp_->set_stretch(ctl->auto_stretch,ctl->stretch_low,ctl->stretch_high,ctl->stretch_gamma);
                    BOOSTER_INFO("stacker") << "Getting to stretch settings in stacker auto="<<ctl->auto_stretch << " low="<<ctl->stretch_low << " high=" << ctl->stretch_high << " gamma=" << ctl->stretch_gamma;
                    if(last_frame_) {
                        auto frames = handle_video(last_frame_);
                        auto res = frames.first;
                        auto ps  = frames.second;
                        if(res && out_) {
                            out_->push(res);
                        }
                        if(ps && plate_solving_) {
                            plate_solving_->push(ps);
                        }
                    }
                    
                }
                break;
            default:
                /// not much to do
                ;
            }
        }
    private:
        queue_pointer_type in_,out_,stats_,plate_solving_;
        std::string data_dir_;
        int width_,height_;
        bool mono_;
        int version_;
        bool calibration_=false;
        std::string output_path_,name_;
        cv::Mat cframe_;
        int cframe_count_;
        int dropped_count_ = 0;
        std::unique_ptr<PostProcessor> pp_;
        std::shared_ptr<StackedFrame> last_frame_;
        int saved_count_ = 0;
        StackerControl stack_info_;
    };

    std::thread start_post_processor(queue_pointer_type in,queue_pointer_type out,queue_pointer_type stats,queue_pointer_type plate_solving,std::string data_dir)
    {
        std::shared_ptr<PostProcessorProcessor> p(new PostProcessorProcessor(in,out,stats,plate_solving,data_dir));
        return std::thread([=]() { p->run(); });
    }
 
} // namespace
 

