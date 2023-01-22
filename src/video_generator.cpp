#include "video_generator.h"
#include <booster/log.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <set>
namespace ols {
    class VideoGenerator {
    public:
        VideoGenerator(queue_pointer_type queue,
                       queue_pointer_type stacking_output,
                       queue_pointer_type live_output,
                       queue_pointer_type debug): 
            data_queue_(queue),
            stack_out_(stacking_output),
            live_out_(live_output),
            debug_out_(debug)
        {
        }
        void handle_jpeg_stack(std::shared_ptr<CameraFrame> frame,cv::Mat rgb,bool copy)
        {
            std::vector<unsigned char> buf;
            
            cv::Mat normalized;
            if(rgb.elemSize1() == 2)
                rgb.convertTo(normalized,CV_8UC3,(1.0/255.0));
            else if(rgb.elemSize1() == 4)
                rgb.convertTo(normalized,CV_8UC3,(1.0/16777215.0));
            else
                normalized = rgb;

            cv::imencode(".jpeg",normalized,buf);

            frame->jpeg_frame = std::shared_ptr<VideoFrame>(new VideoFrame(buf.data(),buf.size()));
            if(stacking_active_) {
                if(copy)
                    frame->frame = rgb.clone();
                else
                    frame->frame = rgb;
            }
        }
        void process_frame(std::shared_ptr<CameraFrame> frame)
        {
            int bpp=-1;
            switch(frame->format.format) {
            case stream_mjpeg: 
                bpp=-1;
                break;
            case stream_mono8:
            case stream_raw8:
                bpp=1;
                break;
            case stream_yuv2: 
            case stream_mono16:
            case stream_raw16:
                bpp=2; 
                break;
            case stream_rgb24:
                bpp=3;
                break;
            default:
                BOOSTER_ERROR("stacker") << "Got invalid format";
                return;
            }
            if(bpp>0 && int(frame->source_frame->size()) != frame->format.height*frame->format.width*bpp) {
                BOOSTER_ERROR("stacker") << "Invalid frame size got " << frame->source_frame->size() 
                            << " bytes, expected " << (frame->format.height*frame->format.width*bpp);
                return;
            }
            switch(frame->format.format) {
            case stream_mjpeg:
                {
                    frame->jpeg_frame = frame->source_frame;
                    // decode jpeg if needed
                    if(stacking_active_) {
                        size_t len = frame->jpeg_frame->size();
                        cv::Mat buffer(1,len,CV_8UC1,frame->jpeg_frame->data());
                        try {
                            frame->frame = cv::imdecode(buffer,cv::IMREAD_UNCHANGED);
                        }
                        catch(std::exception const &e) {
                            BOOSTER_ERROR("stacker") << "Failed to extract jpeg";
                            return;
                        }
                    }
                }
                break;
            case stream_yuv2:
                {
                    cv::Mat yuv2(frame->format.height,frame->format.width,CV_8UC2,frame->source_frame->data());
                    cv::Mat rgb;
                    cv::cvtColor(yuv2,rgb,cv::COLOR_YUV2BGR_YUYV);
                    handle_jpeg_stack(frame,rgb,false);
                }
                break;
            case stream_rgb24:
                {
                    cv::Mat rgb(frame->format.height,frame->format.width,CV_8UC3,frame->source_frame->data());
                    handle_jpeg_stack(frame,rgb,true);
                }
                break;
            case stream_raw8:
            case stream_raw16:
                {
                    cv::Mat bayer(frame->format.height,frame->format.width,(bpp==1 ? CV_8UC1 : CV_16UC1),frame->source_frame->data());
                    cv::Mat rgb;
                    switch(frame->bayer) {
                    case bayer_rg:  cv::cvtColor(bayer,rgb,cv::COLOR_BayerRGGB2BGR); break;
                    case bayer_gr:  cv::cvtColor(bayer,rgb,cv::COLOR_BayerGRBG2BGR); break;
                    case bayer_bg:  cv::cvtColor(bayer,rgb,cv::COLOR_BayerBGGR2BGR); break;
                    case bayer_gb:  cv::cvtColor(bayer,rgb,cv::COLOR_BayerGBRG2BGR); break;
                    default:
                        BOOSTER_ERROR("stacker") << "Invalid bayer patter";
                    }
                    handle_jpeg_stack(frame,rgb,false);
                }
                break;
            case stream_mono8:
            case stream_mono16:
                {
                    #warning "Better mono handling is needed rather than converting Gray 2 RGB)"
                    cv::Mat mono(frame->format.height,frame->format.width,(bpp==1 ? CV_8UC1 : CV_16UC1),frame->source_frame->data());
                    cv::Mat rgb;
                    cv::cvtColor(mono,rgb,cv::COLOR_GRAY2BGR); 
                    handle_jpeg_stack(frame,rgb,false);
                }
                break;
            case stream_error:
                {
                    char const *begin = static_cast<char const *>(frame->source_frame->data());
                    char const *end = begin + frame->source_frame->size();
                    std::string msg(begin,end);
                    BOOSTER_ERROR("stacker") << "Got frame with error " << msg;
                    return;
                }
            default:
                BOOSTER_ERROR("stacker") << "Only mjpeg video genetator is supported for now got " << stream_type_to_str(frame->format.format);
                return;
            }
            live_out_->push(frame);
            if(stacking_active_)
                stack_out_->push(frame);
            if(debug_active_ && stacking_active_)
                debug_out_->push(frame);
        }
        void run()
        {
            while(true) {
                auto data_ptr = data_queue_->pop();
                auto stop_ptr = std::dynamic_pointer_cast<ShutDownData>(data_ptr);
                if(stop_ptr) {
                    live_out_->push(data_ptr);
                    stack_out_->push(data_ptr);
                    debug_out_->push(data_ptr);
                    break;
                }
                auto frame_ptr = std::dynamic_pointer_cast<CameraFrame>(data_ptr);
                if(frame_ptr) {
                    process_frame(frame_ptr);
                    continue;
                }
                auto ctl_ptr = std::dynamic_pointer_cast<StackerControl>(data_ptr);
                if(ctl_ptr){
                    switch(ctl_ptr->op) {
                    case StackerControl::ctl_init:
                        stacking_active_ = true;
                        debug_active_ = ctl_ptr->save_inputs;
                        break;
                    case StackerControl::ctl_resume:
                    case StackerControl::ctl_save_and_continue:
                    case StackerControl::ctl_update:
                        stacking_active_ = true;
                        break;
                    case StackerControl::ctl_pause:
                    case StackerControl::ctl_save:
                        stacking_active_ = false;
                        break;
                    }
                    live_out_->push(data_ptr);
                    stack_out_->push(data_ptr);
                    debug_out_->push(data_ptr);
                    continue;
                }
                
                BOOSTER_ERROR("stacker") << "Invalid data for video generator";
            }
        }
    private:
        queue_pointer_type data_queue_, stack_out_, live_out_, debug_out_;
        bool stacking_active_ = false;
        bool debug_active_ = false;
    };

    std::thread start_generator(queue_pointer_type input,
                                queue_pointer_type stacking_output,
                                queue_pointer_type live_output,
                                queue_pointer_type debug_save)
    {
        std::shared_ptr<VideoGenerator> vg(new VideoGenerator(input,stacking_output,live_output,debug_save));
        std::thread t([=](){vg->run();});
        return std::move(t);
    }

}
