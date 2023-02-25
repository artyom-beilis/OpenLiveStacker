#pragma once
#include <cppcms/application.h>
#include <cppcms/http_context.h>
#include <cppcms/http_response.h>
#include <cppcms/service.h>
#include <booster/log.h>
#include <booster/aio/deadline_timer.h>
#include <set>

#include "data_items.h"
namespace ols {
    class VideoGeneratorApp : public cppcms::application {
    public:
        int frame_rate_ = 10;

        std::string boundary = "092345910257012374590237592375935734--";
        VideoGeneratorApp(cppcms::service &srv,std::string const &type) : cppcms::application(srv),type_(type),
            timer_(srv.get_io_service())
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
        std::function<void(data_pointer_type)> get_callback()
        {
            booster::intrusive_ptr<VideoGeneratorApp> self = this;
            return [=](data_pointer_type p) {
                std::shared_ptr<CameraFrame> frame = std::dynamic_pointer_cast<CameraFrame>(p);
                if(frame && frame->jpeg_frame) {
                    self->frame_handler(frame->jpeg_frame);
                }
            };
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
                ctx->response().out()<<"--" << boundary <<"\r\nContent-Type: image/jpeg\r\nContent-Length: " << frame_->size() << "\r\n\r\n";
                ctx->response().out().write(static_cast<char*>(frame_->data()),frame_->size());
                ctx->response().out()<<"\r\n";
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
                        BOOSTER_DEBUG("stacker") << type_ <<" frame sent async " << frame_counter_;
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
                BOOSTER_DEBUG("stacker") << type_ <<" frame sent direct " << frame_counter_;
                streams_.insert(ctx);
            }
        }
        void send_updated_frame()
        {
            last_update_ = booster::ptime::now();
            auto send_to = streams_;
            for(auto ctx : send_to) {
                serve(ctx);
            }
        }
        void handle_timer()
        {
            booster::ptime delay = booster::ptime::milliseconds(1000/frame_rate_);
            timer_.expires_from_now(delay);
            if(frame_ && (booster::ptime::now() - last_update_) >= delay)
                send_updated_frame();
            timer_.async_wait([=](booster::system::error_code const &) {
                handle_timer();
            });
        }
        void update_frame(std::shared_ptr<VideoFrame> frame)
        {
            if(!frame_) {
                handle_timer();
            }
            frame_ = frame;
            frame_counter_++;
            BOOSTER_INFO("stacker") << type_ << " frame " << frame_counter_ << " arrived";
            send_updated_frame();
        }
        std::set<std::shared_ptr<cppcms::http::context>> streams_;
        std::shared_ptr<VideoFrame> frame_;
        int frame_counter_ = 0;
        std::string type_;
        booster::aio::deadline_timer timer_;
        booster::ptime last_update_;
    };
} // namespace


