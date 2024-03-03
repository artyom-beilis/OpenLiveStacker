#include "data_items.h"
#include "rotation.h"
#include "tiffmat.h"
#include "common_utils.h"
#include <booster/log.h>
#include "processors.h"
#include <cppcms/json.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/hal/intrin.hpp>
#include <fstream>
#include <iomanip>
#include <chrono>
#include "util.h"

#include "simd_utils.h"

namespace ols {
    class PreProcessor {
    public:
        constexpr static int gamma_table_size = 128;
        PreProcessor(queue_pointer_type in,queue_pointer_type out,queue_pointer_type err) :
            in_(in),out_(out),err_(err)
        {
        }
        void run()
        {
            while(true) {
                auto data_ptr = in_->pop();
                auto stop_ptr = std::dynamic_pointer_cast<ShutDownData>(data_ptr);
                if(stop_ptr) {
                    out_->push(data_ptr);
                    break;
                }
                auto video_ptr = std::dynamic_pointer_cast<CameraFrame>(data_ptr);
                if(video_ptr) {
		            auto start = std::chrono::high_resolution_clock::now();
                    bool status = handle_video(video_ptr);
		            auto done = std::chrono::high_resolution_clock::now();
                    if(status)
                        out_->push(data_ptr);
                    double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(done-start).count();
                    BOOSTER_INFO("stacker") << "Preprocessing took " << (time*1000) << "ms";
                    continue;
                }
                auto config_ptr = std::dynamic_pointer_cast<StackerControl>(data_ptr);
                if(config_ptr) {
                    handle_config(config_ptr);
                }
                out_->push(data_ptr);
            }
        }
        void darks_and_flats(cv::Mat &frame)
        {
            float *p=(float*)frame.data;
            float *f=(float*)flats_.data;
            float *d=(float*)darks_.data;
            int N = frame.rows*frame.cols*channels_;
            int i=0;
#ifdef USE_CV_SIMD
            cv::v_float32x4 zero = cv::v_setall_f32(0.0f);
            cv::v_float32x4 one = cv::v_setall_f32(1.0f);
            int limit = N/4*4;
            for(;i<limit;i+=4,p+=4,d+=4,f+=4) {
                cv::v_float32x4 v=cv::v_max(zero,cv::v_load(p) - cv::v_load(d));
                v = cv::v_min(cv::v_load(f) * v,one);
                cv::v_store(p,v);
            }
#endif            
            for(;i<N;i++) {
                float v=std::max(0.0f,*p - *d);
                v*= *f;
                *p = std::min(v,1.0f);
                p++,d++,f++;
            }
        }

        void darks_only(cv::Mat &frame)
        {
            float *p=(float*)frame.data;
            float *d=(float*)darks_.data;
            int N = frame.rows*frame.cols*channels_;
            int i=0;
#ifdef USE_CV_SIMD 
            cv::v_float32x4 zero = cv::v_setzero_f32();
            int limit = N/4*4;
            for(;i<limit;i+=4,p+=4,d+=4) {
                auto v=cv::v_max(zero,cv::v_load(p) - cv::v_load(d));
                cv::v_store(p,v);
            }
#endif            
            for(;i<N;i++) {
                float v=std::max(0.0f,*p - *d);
                *p = v;
                p++,d++;
            }
        }

        void prepare_gamma()
        {
            if(gamma_table_current_gamma_ == -1.0f || gamma_table_current_gamma_ != gamma_) {
                prepare_power_curve(gamma_table_size,gamma_table_,gamma_);
                gamma_table_current_gamma_ = gamma_;
            }
        }

        void apply_gamma(cv::Mat &frame)
        {
            prepare_gamma();

            float *p = (float*)frame.data;
            int N = frame.rows*frame.cols*channels_;
            int i=0;
#ifdef USE_CV_SIMD

            int limit = N / 4 * 4;

            for(i=0;i<limit;i+=4,p+=4) {
                cv::v_float32x4 v = cv::v_load(p);
                curve_simd(v,gamma_table_size,gamma_table_);
                cv::v_store(p,v);
            }
#endif
            for(;i<N;i++,p++) {
                float v = *p;
                v = curve_one(v,gamma_table_size,gamma_table_);
                *p = v;
            }
        }

        bool handle_video(std::shared_ptr<CameraFrame> video)
        {
            if(video->frame.rows != height_ || video->frame.cols != width_ || video->frame.channels() != channels_ ) {
                BOOSTER_ERROR("stacker") << "Invalid mat frame size, expecting " << height_ << "x" << width_ << "x" << channels_ 
                        << " got " << video->frame.rows<< "x"<<video->frame.cols << "x" << video->frame.channels();
                return false;
            }
            video->frame.convertTo(video->processed_frame,cv_type_,(1.0/synthetic_exposure_mpl_)/video->frame_dr);
            if(calibration_)
                return true;
            if(synthetic_exposure_mpl_ > 1) {
                synthetic_exposure_count_ ++;
                if(synthetic_exposure_count_ == 1) {
                    synthetic_frame_ = video->processed_frame;
                    return false;
                }
                else if(synthetic_exposure_count_ < synthetic_exposure_mpl_) {
                    synthetic_frame_ += video->processed_frame;
                    return false;
                }
                else {
                    video->processed_frame += synthetic_frame_;
                    synthetic_frame_ = cv::Mat();
                    synthetic_exposure_count_ = 0;
                }
            }
            if(gamma_ != 1.0) {
                apply_gamma(video->processed_frame);
            }
            if(apply_darks_ && apply_flats_) {
                darks_and_flats(video->processed_frame);
            }
            else {
                if(apply_darks_) {
                    darks_only(video->processed_frame);
                }
                if(apply_flats_) {
                    video->processed_frame = video->processed_frame.mul(flats_);
                }
            }
            if(derotator_) {
                if(first_frame_ts_ == 0)
                    first_frame_ts_ = video->timestamp;
                else {
                    double angle = derotator_->getAngleDeg(first_frame_ts_,video->timestamp);
                    if(derotate_mirror_)
                        angle = -angle;
                    BOOSTER_INFO("stacker") << "Derotating by " << angle << " dir " << (derotate_mirror_ ? "inv" : "str");
                    auto M = cv::getRotationMatrix2D(cv::Point2f(width_/2,height_/2),angle,1.0f);
                    cv::Mat frame_rotated;
                    cv::warpAffine(video->processed_frame,frame_rotated,M,cv::Size(width_,height_));
                    video->processed_frame = frame_rotated;
                }
            }
            return true;
        }
        void handle_config(std::shared_ptr<StackerControl> ctl)
        {
            switch(ctl->op) {
            case StackerControl::ctl_init:
                width_ = ctl->width;
                height_ = ctl->height;
                mono_ = ctl->mono;
                channels_ = mono_ ? 1 : 3;
                cv_type_ = mono_ ? CV_32FC1 : CV_32FC3;
                calibration_ = ctl->calibration;
                synthetic_exposure_mpl_ = 1;
                synthetic_frame_ = cv::Mat();
                synthetic_exposure_count_ = 0;
                if(calibration_)
                    break;
                gamma_ = ctl->source_gamma;
                synthetic_exposure_mpl_ = ctl->synthetic_exposure_mpl;
                if(ctl->derotate) {
                    derotator_.reset(new Derotator(ctl->lon,ctl->lat,ctl->ra,ctl->de));
                    first_frame_ts_ = 0;
                    derotate_mirror_ = ctl->derotate_mirror;
                }
                else
                    derotator_.reset();
                apply_darks_ = false;
                if(!ctl->darks_path.empty()) 
                    load_darks(ctl->darks_path);
                apply_flats_ = false;
                if(!ctl->flats_path.empty())
                    load_flats(ctl->flats_path,ctl->dark_flats_path);
                break;
            case StackerControl::ctl_pause:
                // drop frames in case of pause
                if(synthetic_exposure_mpl_ > 1) {
                    BOOSTER_INFO("stacker") << "Dropping "<< synthetic_exposure_count_ << " non-synthetic frames due to pause";
                    synthetic_frame_ = cv::Mat();
                    synthetic_exposure_count_ = 0;  
                }
                break;
            default:
                /// not much to do
                ;
            }
        }
    private:
        bool check_file(cv::Mat &m,std::string const &type,std::string const &path)
        {
            if(m.rows != height_ || m.cols != width_ || m.channels() != channels_) {
                BOOSTER_ERROR("stacker") << "Invalid " << type  << "format, expecting " << height_ <<"x" << width_ << "x" <<channels_
                    << " got " << m.rows<<"x" <<m.cols<<"x"<<m.channels()<< " for file " << path;
                return false;    
            }
            return true;

        }
        void load_flats(std::string flats_path,std::string dark_flats_path)
        {
            try {
                apply_flats_=false;
                cv::Mat flats = load_tiff(flats_path);
                if(!check_file(flats,"flats",flats_path)) {
                    return;
                }
                if(!dark_flats_path.empty()) {
                    cv::Mat dark_flats = load_tiff(dark_flats_path);
                    if(!check_file(dark_flats,"dark flats",dark_flats_path)) {
                        return;
                    }
                    flats = cv::max(0.0f,flats - dark_flats);
                }
                cv::Mat gray_flats;
                if(flats.channels() > 1)
                    cv::cvtColor(flats,gray_flats,cv::COLOR_BGR2GRAY);
                else
                    gray_flats = flats;
                double minV,maxV;
                cv::minMaxLoc(gray_flats,&minV,&maxV);
                if(minV <= 0 || maxV / minV > 100) {
                    std::ostringstream ss;
                    ss << "Flats have too grate min/max range, suspecting collection issue - disableing flats minValue=" << minV << " maxValue="<<maxV;
                    BOOSTER_ERROR("stacker") << ss.str();
                    send_message(err_,"flats",ss.str());
                    return;
                }
                gray_flats = maxV/gray_flats;
                if(flats.channels() > 1)
                    cv::cvtColor(gray_flats,flats,cv::COLOR_GRAY2BGR);
                else
                    flats = gray_flats;
                flats_ = flats;
                apply_flats_=true;
            }
            catch(std::exception const &e) {
                std::ostringstream ss;
                ss <<  "Failed to load flats from " << flats_path << " and dark flats from " << dark_flats_path << ": " << e.what();
                BOOSTER_ERROR("stacker") << ss.str();
                send_message(err_,"flats",ss.str());
                apply_flats_ = false;
            }
        }
        void load_darks(std::string darks_path)
        {
            try {
                darks_ = load_tiff(darks_path);
                if(check_file(darks_,"darks",darks_path)) {
                    apply_darks_ = true;
                    cv::Mat tmp;
                    if(gamma_ != 1.0) {
                        cv::pow(darks_,gamma_,tmp);
                        darks_ = tmp;
                    }
                }
                else {
                    apply_darks_ = false;
                }
            }
            catch(std::exception const &e) {
                std::ostringstream ss;
                ss << "Failed to load darks from " << darks_path << ": " << e.what();
                send_message(err_,"darks",ss.str());
                BOOSTER_ERROR("stacker") << ss.str();
                apply_darks_ = false;
            }
        }

        queue_pointer_type in_,out_,err_;
        int width_,height_;
        bool mono_;
        int channels_;
        int cv_type_;
        bool calibration_;
        float gamma_;
        float gamma_table_[gamma_table_size+1];
        float gamma_table_current_gamma_ = -1;
        std::unique_ptr<Derotator> derotator_;
        double first_frame_ts_;
        bool derotate_mirror_;
        cv::Mat darks_;
        cv::Mat flats_;
        bool apply_darks_;
        bool apply_flats_;
        int synthetic_exposure_mpl_ = 1;
        int synthetic_exposure_count_ = 0;
        cv::Mat synthetic_frame_;
    };

    std::thread start_preprocessor(queue_pointer_type in,queue_pointer_type out,queue_pointer_type err)
    {
        std::shared_ptr<PreProcessor> p(new PreProcessor(in,out,err));
        return std::thread([=]() { p->run(); });
    }
} // ols
