#pragma once
#include "camera.h"
#include "data_items.h"
#include "video_stream.h"
#include "video_generator.h"
#include "camera_iface.h"
#include <cppcms/service.h>
#include <booster/posix_time.h>
#include <thread>
#include <atomic>
#pragma once
#include "camera.h"
#include "camera_iface.h"

namespace ols {

    class StackerStatsNotification;

    class OpenLiveStacker : public CameraInterface {
    public:

        typedef std::unique_lock<std::recursive_mutex> guard;
        OpenLiveStacker(std::string data_dir = "./data");
        ~OpenLiveStacker();

        static void disableCVThreads();

        int mem_limit_mb = 0;
        int http_port = 8080;
        std::string http_ip = "0.0.0.0";
        std::string document_root = "www-data";
        std::string indi_libdir;

        void init(std::string driver,int external_option = -1);
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
        virtual void start_stream(CamStreamFormat format,double max_framerate);
        virtual CamStreamFormat stream_format()
        {
            return current_format_;
        }
        virtual void stop_stream();
        virtual CameraDriver &driver()
        {
            return *driver_;
        }

        static int get_frames_count()
        {
            return received_;
        }

        std::string log_queues();

    private:
        void stop();
        void handle_video_frame(CamFrame const &cf);
        static void set_plate_solving_image(data_pointer_type p);

        /// Data Queues

        queue_pointer_type video_generator_queue_    = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type video_display_queue_      = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type preprocessor_queue_       = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type stacker_queue_            = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type pp_queue_                 = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type stack_display_queue_      = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type debug_save_queue_         = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type stacker_stats_queue_      = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type plate_solving_queue_      = std::shared_ptr<queue_type>(new queue_type());
        
        std::recursive_mutex camera_lock_;
        std::unique_ptr<Camera> camera_;
        std::unique_ptr<CameraDriver> driver_;
        bool stream_active_ = false;
        CamStreamFormat current_format_;

        int dropped_ = 0;
        int dropped_since_last_update_ = 0;
        booster::ptime last_dropped_frame_ts_ = booster::ptime::now();
        booster::ptime last_frame_ts_;
        double max_framerate_ = 0;
        static std::atomic<int> received_;

        std::string data_dir_;
        std::string debug_dir_;

        booster::intrusive_ptr<VideoGeneratorApp> video_generator_app_;
        booster::intrusive_ptr<VideoGeneratorApp> stacked_video_generator_app_;
        booster::intrusive_ptr<StackerStatsNotification> stats_stream_app_;
        
        std::thread video_generator_thread_;
        std::thread debug_save_thread_;
        std::thread preprocessor_thread_;
        std::thread stacker_thread_;
        std::thread pp_thread_;
        
        std::shared_ptr<cppcms::service> web_service_;

    };
};
