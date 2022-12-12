#pragma once
#include <cppcms/application.h>
#include <cppcms/http_context.h>
#include <cppcms/http_response.h>
#include <cppcms/service.h>
#include <booster/log.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <set>

namespace ols {
    class VideoGeneratorApp : public cppcms::application {
    public:
        std::string boundary = "092345910257012374590237592375935734--";
        VideoGeneratorApp(cppcms::service &srv) : cppcms::application(srv)
        {
        }
        virtual void main(std::string )
        {
            BOOSTER_INFO("stacker") << "New video client connected";
            booster::shared_ptr<cppcms::http::context> context=release_context();
            streams_.insert(context);
            context->response().full_asynchronous_buffering(false);
            context->response().content_type("multipart/x-mixed-replace;boundary=" + boundary);
            context->response().cache_control("no-cache");
            serve(context);
        }
        // thread safe method
        void frame_handler(std::shared_ptr<VideoFrame> frame)
        {
            service().post([=](){
                update_frame(frame);
            });
        }
    private:
        void serve(std::shared_ptr<cppcms::http::context> ctx)
        {
            if(frame_) {
                ctx->response().out()<<"--" << boundary << "\r\nContent-Type: image/jpeg\r\nContent-Length: " << frame_->size() << "\r\n\r\n";
                ctx->response().out().write(static_cast<char*>(frame_->data()),frame_->size());
            }
            else {
                // flush headers
                ctx->response().out();
            }
            ctx->response().out()<<std::flush;
            if(!ctx->response().out()) {
                BOOSTER_INFO("stacker") << "Failed to send a frame to client";
                streams_.erase(ctx);
                return;
            }
            if(ctx->response().pending_blocked_output()) {
                int current_frame = frame_counter_;
                ctx->async_flush_output([=](cppcms::http::context::completion_type type) {
                    if(type == cppcms::http::context::operation_completed) {
                        BOOSTER_DEBUG("stacker") << "frame sent async " << frame_counter_;
                        streams_.insert(ctx);
                        if(current_frame != frame_counter_) {
                            serve(ctx);
                        }
                    }
                    else {
                        BOOSTER_INFO("stacker") << "client gone";
                    }
                });
                streams_.erase(ctx);
            }
            else {
                BOOSTER_DEBUG("stacker") << "frame sent direct " << frame_counter_;
                streams_.insert(ctx);
            }
        }
        void update_frame(std::shared_ptr<VideoFrame> frame)
        {
            frame_ = frame;
            frame_counter_++;
            auto send_to = streams_;
            for(auto ctx : send_to) {
                serve(ctx);
            }
        }
        std::set<std::shared_ptr<cppcms::http::context>> streams_;
        std::shared_ptr<VideoFrame> frame_;
        int frame_counter_ = 0;
    };

    class VideoGenerator {
    public:
        typedef std::function<void(std::shared_ptr<VideoFrame>)> callback_type;
        
        VideoGenerator(queue_pointer_type queue,booster::intrusive_ptr<VideoGeneratorApp> app): 
            app_(app),
            data_queue_(queue)
        {
        }
        void process_frame(std::shared_ptr<CameraFrame> frame)
        {
            if(frame->format.format == stream_mjpeg) {
                frame->jpeg_frame = frame->source_frame;
            }
            else if(frame->format.format == stream_yuv2) {
                if(int(frame->source_frame->size()) != frame->format.height*frame->format.width*2)
                    throw std::runtime_error("Invalid frame size");
                cv::Mat yuv2(frame->format.height,frame->format.width,CV_8UC2,frame->source_frame->data());
                cv::Mat rgb;
                cv::cvtColor(yuv2,rgb,cv::COLOR_YUV2BGR_YUYV);
                std::vector<unsigned char> buf;
                cv::imencode(".jpeg",rgb,buf);
                frame->frame = rgb;
                frame->jpeg_frame = std::shared_ptr<VideoFrame>(new VideoFrame(buf.data(),buf.size()));
            }
            else {
                BOOSTER_ERROR("stacker") << "Only mjpeg video genetator is supported for now got " << stream_type_to_str(frame->format.format);
                return;
            }
            app_->frame_handler(frame->jpeg_frame);
        }
        void run()
        {
            while(true) {
                auto data_ptr = data_queue_->pop();
                auto stop_ptr = std::dynamic_pointer_cast<ShutDownData>(data_ptr);
                if(stop_ptr)
                    break;
                auto frame_ptr = std::dynamic_pointer_cast<CameraFrame>(data_ptr);
                if(frame_ptr) {
                    process_frame(frame_ptr);
                    continue;
                }
                BOOSTER_ERROR("stacker") << "Invalid data for video generator";
            }
        }
    private:
        booster::intrusive_ptr<VideoGeneratorApp> app_;
        queue_pointer_type data_queue_;
    };
}
