#include "video_generator.h"
#include <booster/log.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <set>
namespace ols {
    class VideoGenerator {
    public:
        VideoGenerator(queue_pointer_type queue,queue_pointer_type output): 
            data_queue_(queue),
            out_(output)
        {
        }
        void process_frame(std::shared_ptr<CameraFrame> frame)
        {
            switch(frame->format.format) {
            case stream_mjpeg:
                {
                    frame->jpeg_frame = frame->source_frame;
                    // decode jpeg if needed
                    if(stacking_active_) {
                        size_t len = frame->jpeg_frame->size();
                        cv::Mat buffer(1,len,CV_8UC1,frame->jpeg_frame->data());
                        frame->frame = cv::imdecode(buffer,cv::IMREAD_UNCHANGED);
                    }
                }
                break;
            case stream_yuv2:
                {
                    if(int(frame->source_frame->size()) != frame->format.height*frame->format.width*2) {
                        BOOSTER_ERROR("stacker") << "Invalid frame size got " << frame->source_frame->size() 
                                    << " bytes, expected " << (frame->format.height*frame->format.width*2);
                        return;
                    }
                    cv::Mat yuv2(frame->format.height,frame->format.width,CV_8UC2,frame->source_frame->data());
                    cv::Mat rgb;
                    cv::cvtColor(yuv2,rgb,cv::COLOR_YUV2BGR_YUYV);
                    std::vector<unsigned char> buf;
                    cv::imencode(".jpeg",rgb,buf);
                    frame->jpeg_frame = std::shared_ptr<VideoFrame>(new VideoFrame(buf.data(),buf.size()));
                    // keep rgb if needed
                    if(stacking_active_)
                        frame->frame = rgb;
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
            out_->push(frame);
        }
        void run()
        {
            while(true) {
                auto data_ptr = data_queue_->pop();
                auto stop_ptr = std::dynamic_pointer_cast<ShutDownData>(data_ptr);
                if(stop_ptr) {
                    out_->push(data_ptr);
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
                }
                out_->push(data_ptr);

                BOOSTER_ERROR("stacker") << "Invalid data for video generator";
            }
        }
    private:
        queue_pointer_type data_queue_;
        queue_pointer_type out_;
        bool stacking_active_ = false;
    };

    std::thread start_generator(queue_pointer_type input,queue_pointer_type output)
    {
        std::shared_ptr<VideoGenerator> vg(new VideoGenerator(input,output));
        std::thread t([=](){vg->run();});
        return std::move(t);
    }

}
