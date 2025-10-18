#include "data_items.h"
#include <booster/log.h>
#include "processors.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iomanip>
#include <chrono>
#include "util.h"


namespace ols {
    class VRProcessor {
    public:
        VRProcessor(queue_pointer_type in,queue_pointer_type out) : 
            input_(in),
            output_(out)
        {
        }
        void run()
        {
            while(true) {
                data_pointer_type data_ptr = input_->pop();
                auto stop_ptr = std::dynamic_pointer_cast<ShutDownData>(data_ptr);
                if(stop_ptr) {
                    if(output_)
                        output_->push(data_ptr);
                    break;
                }
                if(active_) {
                    std::shared_ptr<CameraFrame> frame = std::dynamic_pointer_cast<CameraFrame>(data_ptr);
                    if(frame) {
                        handle_frame(frame);
                        return;
                    }
                }
                std::shared_ptr<VRInfo> info = std::dynamic_pointer_cast<VRInfo>(data_ptr);
                if(info) {
                    width_ = info->width;
                    height_ = info->height;
                    active_ = info->enable;
                    circle_ = info->circle;
                    factor_ = info->factor;
                    for_height_ = for_width_ = -1;
                    return;
                }
            }
        }
        void create_maps(int W,int H)
        {
            if(for_width_ == W && for_height_ == H)
                return;
            double scale;
            if(double(W) / H  >= double(width_) / height_) {
                crop_y_ = 0;
                crop_h_ = H;
                scale = double(height_ - 1) / (H - 1);
                crop_w_ = std::min(W,int(width_ / scale + 0.5f));
                crop_x_ = (W - crop_w_) / 2;
            }
            else {
                crop_x_ = 0;
                crop_w_ = W;
                scale = double(width_ - 1) / (W - 1);
                crop_h_ = std::min(H,int(height_ / scale + 0.5f));
                crop_y_ = (H - crop_h_) / 2;
            }
            int R_limit = std::max(width_,height_);
            float distor_scaling = factor_ / R_limit;
            map_x_.create(width_,height_,CV_32FC1);
            map_y_.create(width_,height_,CV_32FC1);
            int c_x = width_ / 2;
            int c_y = height_ / 2;
            for(int y = 0;y < height_;y++) {
                for(int x = 0; x < width_;x++) {
                    int dx = c_x - x;
                    int dy = c_y - y;
                    int R = dx*dx + dy*dy;
                    float src_x = c_x + distor_scaling * R * dx;
                    float src_y = c_y + distor_scaling * R * dy;
                    if(circle_ && R > R_limit * R_limit) {
                        src_x = -100;
                        src_y = -100;
                    }
                    map_x_.at<float>(y,x) = src_x * scale;
                    map_y_.at<float>(y,x) = src_y * scale;
                }
            }
            for_width_  = W;
            for_height_ = H;
        }
        void handle_frame(std::shared_ptr<CameraFrame> frame)
        {
            int target_type;
            if(frame->frame.elemSize() == 1)
                target_type = CV_8UC1;
            else
                target_type = CV_8UC3;
            
            create_maps(frame->frame.cols,frame->frame.rows);

            cv::Mat cropped = frame->frame(cv::Range(crop_y_,crop_y_ + crop_h_),
                                           cv::Range(crop_x_,crop_x_ + crop_w_));
            res_.create(width_,height_,target_type);
            cv::remap(cropped,res_,map_x_,map_y_,cv::INTER_LINEAR_EXACT);
            cv::Mat full_img(width_*2 + pad_,height_,target_type);
            res_.copyTo(full_img(cv::Range(0,height_),cv::Range(0,width_)));
            res_.copyTo(full_img(cv::Range(0,height_),cv::Range(width_ + pad_,2*width_ + pad_)));
            full_img(cv::Range(0,height_),cv::Range(width_,width_+pad_)).setTo(cv::Scalar::all(0));
            full_img(cv::Range(height_ / 4, 3* height_/4),cv::Range(width_ + pad_/2 - 1,width_+pad_/2 + 1)).setTo(cv::Scalar::all(255));
            
            std::vector<unsigned char> buf;
            cv::imencode(".jpeg",full_img,buf);
            std::shared_ptr<CameraFrame> video(new CameraFrame());
            video->jpeg_frame = std::shared_ptr<VideoFrame>(new VideoFrame(buf.data(),buf.size()));
            output_->push(video);
        }
    private:
        queue_pointer_type input_,output_;
        int crop_x_,crop_y_,crop_w_,crop_h_;
        cv::Mat res_;
        cv::Mat map_x_,map_y_;
        int for_width_=-1,for_height_ = -1;
        int width_ = 0,height_ = 0;
        int pad_ = 0;
        double factor_ = 0;
        bool circle_ = false;
        bool active_ = false;
    };

    std::thread start_vr_processor(queue_pointer_type in,queue_pointer_type out)
    {
        std::shared_ptr<VRProcessor> vr(new VRProcessor(in,out));
        std::thread thr([=]() {
            vr->run();
        });
        return thr;
    }
} // namespace ols
