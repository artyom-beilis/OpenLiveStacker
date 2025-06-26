#include "video_generator.h"
#include "live_stretch.h"
#include "hot_removal.h"
#include <booster/log.h>
#include <booster/posix_time.h>
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
                       queue_pointer_type debug,
                       queue_pointer_type plate_solving_output): 
            data_queue_(queue),
            stack_out_(stacking_output),
            live_out_(live_output),
            debug_out_(debug),
            plate_solving_out_(plate_solving_output)
        {
        }
        void handle_jpeg_stack(std::shared_ptr<CameraFrame> frame,cv::Mat image,bool copy)
        {
            std::vector<unsigned char> buf;

            cv::Mat normalized;
            frame->live_is_stretched = false;
            if(live_auto_stretch_) {
                auto now = booster::ptime::now();
                if(cached_factor_ < 1.0 || booster::ptime::to_number(now - cached_factor_updated_) > 0) {
                    cached_factor_ = -1.0;
                    cached_factor_updated_ = now;
                }
                live_stretch(image,cached_factor_,normalized);
                frame->live_is_stretched = true;
            }
            else if(frame->frame_dr == 255 && image.elemSize1() == 1) {
                normalized = image;
            }
            else {
                double factor = 255.0 / frame->frame_dr;
                image.convertTo(normalized,image.channels() == 3 ? CV_8UC3: CV_8UC1,factor);
            }

            cv::imencode(".jpeg",normalized,buf);

            frame->jpeg_frame = std::shared_ptr<VideoFrame>(new VideoFrame(buf.data(),buf.size()));
            if(stacking_active_ || plate_solving_out_) {
                if(copy)
                    frame->frame = image.clone();
                else
                    frame->frame = image;
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
            case stream_rgb48:
                bpp=6;
                break;
            case stream_error:
                bpp=-1;
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
                    if(stacking_active_ || plate_solving_out_ || live_auto_stretch_) {
                        size_t len = frame->jpeg_frame->size();
                        cv::Mat buffer(1,len,CV_8UC1,frame->jpeg_frame->data());
                        try {
                            frame->frame = cv::imdecode(buffer,cv::IMREAD_UNCHANGED);
                            frame->frame_dr = 255;
                            frame->raw = frame->frame;
                            if(live_auto_stretch_) {
                                // can forward jpeg as is since it need to be stretched
                                handle_jpeg_stack(frame,frame->frame,true);
                            }
                        }
                        catch(std::exception const &e) {
                            BOOSTER_ERROR("stacker") << "Failed to extract jpeg:" << e.what();
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
                    frame->frame_dr = 255;
                    handle_jpeg_stack(frame,rgb,false);
                    // use unpacked since there is no native YUYV tiff format
                    frame->raw = frame->frame;
                }
                break;
            case stream_rgb24:
                {
                    cv::Mat tmp(frame->format.height,frame->format.width,CV_8UC3,frame->source_frame->data());
                    cv::Mat rgb;
                    if (frame->byteorder == byteorder_rgb) {
                        cv::cvtColor(tmp,rgb,cv::COLOR_RGB2BGR);
                    } else {
                        rgb = tmp;
                    }
                    frame->frame_dr = 255;
                    frame->raw = rgb;
                    handle_jpeg_stack(frame,rgb,true);
                }
                break;
            case stream_rgb48:
                {
                    cv::Mat rgb(frame->format.height,frame->format.width,CV_16UC3,frame->source_frame->data());
                    frame->frame_dr = 65535;
                    frame->raw = rgb;
                    handle_jpeg_stack(frame,rgb,false);
                }
                break;
            case stream_raw8:
            case stream_raw16:
                {
                    cv::Mat bayer(frame->format.height,frame->format.width,(bpp==1 ? CV_8UC1 : CV_16UC1),frame->source_frame->data());
                   cv::Mat desp,rgb;
                    if(remove_hot_pixels_) 
                        desp = remove_hot_pixels(bayer,false);
                    else
                        desp = bayer;
                    switch(frame->bayer) {
                    case bayer_rg:  cv::cvtColor(desp,rgb,cv::COLOR_BayerBG2BGR); break; // COLOR_BayerRGGB2BGR = COLOR_BayerBG2BGR
                    case bayer_gr:  cv::cvtColor(desp,rgb,cv::COLOR_BayerGB2BGR); break; // COLOR_BayerGRBG2BGR = COLOR_BayerGB2BGR
                    case bayer_bg:  cv::cvtColor(desp,rgb,cv::COLOR_BayerRG2BGR); break; // COLOR_BayerBGGR2BGR = COLOR_BayerRG2BGR
                    case bayer_gb:  cv::cvtColor(desp,rgb,cv::COLOR_BayerGR2BGR); break; // COLOR_BayerGBRG2BGR = COLOR_BayerGR2BGR
                    case bayer_na:  cv::cvtColor(desp,rgb,cv::COLOR_GRAY2BGR);    break; // handle case when indigo reports raw but it actually mono
                    default:
                        BOOSTER_ERROR("stacker") << "Invalid bayer patter";
                    }
                    if (frame->byteorder == byteorder_rgb) {
                        switch(frame->bayer) {
                        case bayer_rg:  cv::cvtColor(desp,rgb,cv::COLOR_BayerBG2RGB); break; // COLOR_BayerRGGB2RGB = COLOR_BayerBG2RGB
                        case bayer_gr:  cv::cvtColor(desp,rgb,cv::COLOR_BayerGB2RGB); break; // COLOR_BayerGRBG2RGB = COLOR_BayerGB2RGB
                        case bayer_bg:  cv::cvtColor(desp,rgb,cv::COLOR_BayerRG2RGB); break; // COLOR_BayerBGGR2RGB = COLOR_BayerRG2RGB
                        case bayer_gb:  cv::cvtColor(desp,rgb,cv::COLOR_BayerGR2RGB); break; // COLOR_BayerGBRG2RGB = COLOR_BayerGR2RGB
                        case bayer_na:  cv::cvtColor(desp,rgb,cv::COLOR_GRAY2RGB);    break; // handle case when indigo reports raw but it actually mono
                        default:
                            BOOSTER_ERROR("stacker") << "Invalid bayer patter";
                        }
                    } else {
                        switch(frame->bayer) {
                        case bayer_rg:  cv::cvtColor(desp,rgb,cv::COLOR_BayerBG2BGR); break; // COLOR_BayerRGGB2BGR = COLOR_BayerBG2BGR
                        case bayer_gr:  cv::cvtColor(desp,rgb,cv::COLOR_BayerGB2BGR); break; // COLOR_BayerGRBG2BGR = COLOR_BayerGB2BGR
                        case bayer_bg:  cv::cvtColor(desp,rgb,cv::COLOR_BayerRG2BGR); break; // COLOR_BayerBGGR2BGR = COLOR_BayerRG2BGR
                        case bayer_gb:  cv::cvtColor(desp,rgb,cv::COLOR_BayerGR2BGR); break; // COLOR_BayerGBRG2BGR = COLOR_BayerGR2BGR
                        case bayer_na:  cv::cvtColor(desp,rgb,cv::COLOR_GRAY2BGR);    break; // handle case when indigo reports raw but it actually mono
                        default:
                            BOOSTER_ERROR("stacker") << "Invalid bayer patter";
                        }
                    }
                    frame->frame_dr = (bpp==1 ? 255 : 65535);
                    handle_jpeg_stack(frame,rgb,false);
                    frame->raw = bayer;
                }
                break;
            case stream_mono8:
            case stream_mono16:
                {
                    cv::Mat mono(frame->format.height,frame->format.width,(bpp==1 ? CV_8UC1 : CV_16UC1),frame->source_frame->data());
                    cv::Mat desp = mono;
                    if(remove_hot_pixels_)
                        desp = remove_hot_pixels(mono,true);
                    frame->frame_dr = (bpp==1 ? 255 : 65535);
                    handle_jpeg_stack(frame,desp,true);
                    frame->raw = mono;
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
            if(plate_solving_out_ && !stacking_in_process_)
                plate_solving_out_->push(frame);
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
                        remove_hot_pixels_ = ctl_ptr->remove_hot_pixels; 
                        stacking_in_process_ = true;
                        debug_active_ = ctl_ptr->save_inputs;
                        break;
                    case StackerControl::ctl_resume:
                        stacking_active_ = true;
                        stacking_in_process_ = true;
                        break;
                    case StackerControl::ctl_pause:
                        stacking_active_ = false;
                        stacking_in_process_ = true;
                        break;
                    case StackerControl::ctl_cancel:
                        stacking_active_ = false;
                        stacking_in_process_ = false;
                        remove_hot_pixels_ = false;
                        break;
                    case StackerControl::ctl_save:
                    case StackerControl::ctl_update:
                        break;
                    }
                    live_out_->push(data_ptr);
                    stack_out_->push(data_ptr);
                    debug_out_->push(data_ptr);
                    continue;
                }
                auto live_ptr = std::dynamic_pointer_cast<LiveControl>(data_ptr);
                if(live_ptr){
                    handle_live_ctl(live_ptr);
                    continue;
                }
               
                char const *name = "nullptr";
                if(data_ptr) {
                    auto &v=*data_ptr;
                    name = typeid(v).name();
                }
                BOOSTER_ERROR("stacker") << "Invalid data for video generator got " << name;
            }
        }
        void handle_live_ctl(std::shared_ptr<LiveControl> ctl)
        {
            switch(ctl->opt) {
            case opt_live_stretch:
                live_auto_stretch_ = int(ctl->value);
                cached_factor_ = -1.0;
                break;
            default:
                BOOSTER_ERROR("OLS") << "Internal error invaid live option";
            }
        }
    private:
        queue_pointer_type data_queue_, stack_out_, live_out_, debug_out_, plate_solving_out_;
        bool stacking_active_ = false;
        bool stacking_in_process_ = false;
        bool debug_active_ = false;
        bool remove_hot_pixels_ = false;
        bool live_auto_stretch_ = true;
        float cached_factor_ = -1;
        booster::ptime cached_factor_updated_;
    };

    std::thread start_generator(queue_pointer_type input,
                                queue_pointer_type stacking_output,
                                queue_pointer_type live_output,
                                queue_pointer_type debug_save,
                                queue_pointer_type plate_solving_out)
    {
        std::shared_ptr<VideoGenerator> vg(new VideoGenerator(input,stacking_output,live_output,debug_save,plate_solving_out));
        std::thread t([=](){vg->run();});
        return t;
    }

}
