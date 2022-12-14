#pragma once
#include "camera.h"
#include "data_items.h"
#include "video_stream.h"
#include "video_generator.h"
#include "camera_iface.h"
#include <cppcms/service.h>
#include <thread>

namespace ols {


    class OpenLiveStacker : public CameraInterface {
    public:

        typedef std::unique_lock<std::recursive_mutex> guard;
        OpenLiveStacker(std::string data_dir = "./data");
        ~OpenLiveStacker();

        int http_port = 8080;
        std::string http_ip = "0.0.0.0";
        std::string document_root = "www-data";

        void init(std::string driver);
        void run();
        void shutdown();

        virtual std::recursive_mutex &lock() 
        {
            return camera_lock_;
        }
        virtual Camera &cam();
        virtual CamStatus status();
        virtual void open_camera(int id);
        virtual void close_camera();
        virtual void start_stream(CamStreamFormat format);
        virtual CamStreamFormat stream_format()
        {
            return current_format_;
        }
        virtual void stop_stream();
        virtual CameraDriver &driver()
        {
            return *driver_;
        }

    private:
        void stop();
        void handle_video_frame(CamFrame const &cf);

        /// Data Queues

        queue_pointer_type video_generator_queue_    = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type video_display_queue_      = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type preprocessor_queue_       = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type stacker_queue_            = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type stack_display_queue_      = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type debug_save_queue_         = std::shared_ptr<queue_type>(new queue_type());
        
        std::recursive_mutex camera_lock_;
        std::unique_ptr<Camera> camera_;
        std::unique_ptr<CameraDriver> driver_;
        bool stream_active_ = false;
        CamStreamFormat current_format_;

        std::string data_dir_;
        std::string debug_dir_;

        booster::intrusive_ptr<VideoGeneratorApp> video_generator_app_;
        booster::intrusive_ptr<VideoGeneratorApp> stacked_video_generator_app_;
        
        std::thread video_generator_thread_;
        std::thread debug_save_thread_;
        std::thread preprocessor_thread_;
        std::thread stacker_thread_;
        
        std::shared_ptr<cppcms::service> web_service_;

    };
};
