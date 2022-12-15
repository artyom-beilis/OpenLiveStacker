#include "data_item.h"
#include "rotation.h"
#include "tiffmat.h"
#include <booster/log.h>
#include "processors.h"
namespace ols {
    class PreProcessor {
    public:
        PreProcessor(queue_pointer_type in,queue_pointer_type out) :
            in_(in),out_(out)
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
                    if(handle_video(video_ptr))
                        out_->push(data_ptr);
                    continue;
                }
                auto config_ptr = std::dynamic_pointer_cast<StackerControl>(data_ptr)
                if(config_ptr) {
                    handle_config(config_ptr);
                }
                out_->push(data_ptr);
            }
        }
        bool handle_video(std::shared_ptr<CameraFrame> video)
        {
            if(video->frame.rows != height_ || video->frame.cols != width_) {
                BOOSTER_LOG("stacker") << "Invalid mat frame size";
                return false;
            }
            video->frame.convertTo(video->processed_frame,CV_32FC3,1/(255.0));
            if(calibration_)
                return true;
            if(gamma_ != 1.0)
                cv::pow(video->processed_frame,gamma_,video->processed_frame)
            if(apply_darks_)
                video->processed_frame -= darks_;
            if(derotator_) {
                if(first_frame_ts_ == 0)
                    first_frame_ts_ = video->timestamp;
                else {
                    double angle = getAngleDeg(first_frame_ts_,video->timestamp);
                    if(derotate_mirror_)
                        angle = -angle;
                    auto M = cv::getRotationMatrix2D(cv::Point2f(width_/2,height_/2),rotate,1.0f);
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
            case ctl_init:
                width_ = ctl->width_;
                height_ = ctl->height_;
                calibration_ = ctl->calibration;
                if(calibration_)
                    break;
                gamma_ = ctl->soruce_gamma;
                if(ctl->derotate) {
                    derotator_.reset(new Derotator(ctl->lon,ctl->lat,ctl->ra,ctl->de));
                    first_frame_ts_ = 0;
                    derotate_mirror_ = derotate;
                }
                else
                    derotator_.reset();
                apply_darks_ = false;
                if(!ctl->darks_id.empty()) 
                    load_darks(ctl->darks_path);
                break;
            default:
                /// not much to do
                ;
            }
        }
    private:
        void load_darks(std::string darks_path)
        {
            try {
                darks_ = load_tiff(darks_path);
                if(darks_.rows == height_ && darks_.cols == width_) {
                    apply_darks_ = true;
                    cv::Mat tmp;
                    if(gamma_ != 1.0) {
                        cv::pow(darks_,gamma_,tmp);
                        darks_ = tmp;
                    }
                }
                else {
                    apply_darks_ = false;
                    BOOSTER_LOG("stacker") << "Failed to load darks from " << id << ": size mistmatch";
                }
            }
            catch(std::exception const &e) {
                BOOSTER_LOG("stacker") << "Failed to load darks from " << id << ": " << e.what();
                apply_darks_ = false;
            }
        }
        int width_,height_;
        bool calibration_;
        float gamma_;
        std::unique_ptr<Derotator> detotator_;
        double first_frame_ts_;
        bool derotate_mirror_;
        cv::Mat darks_;
        bool apply_darks_;
    };

    std::thread start_preprocessor(queue_pointer_type in,queue_pointer_type out)
    {
        std::shared_ptr<PreProcessor> p(new PreProcessor(in,out));
        return std::thread([=]() { p->run(); });
    }
}
