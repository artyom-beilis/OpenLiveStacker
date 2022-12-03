#pragma once

namespace ols {


    class OpenLiveStacker {
    public:
        OpenLiveStacker();
        ~OpenLiveStacker();

    private:

        /// Data Queues

        queue_pointer_type converter_queue_         = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type stacker_queue_           = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type post_processing_queue_   = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type stacked_display_queue_   = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type video_display_queue_     = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type data_save_queue_         = std::shared_ptr<queue_type>(new queue_type());
        queue_pointer_type stacking_progress_queue_ = std::shared_ptr<queue_type>(new queue_type());

        /// Flow

        /// Camera |-> data_save_queue_ -> DebugFrameSaver
        ///        |-> video_display_queue_ -> RawVideoGenerator 
        ///        |-> converter_queue_ -> Frame2Mat -> stacker_queue_ -> Stacker |-> post_processing_queue_ -> PostProcess -> stacked_display_queue_ -> StackedVideoGenerator
        ///                                                                       |-> stacking_progress_queue_ -> PublishStats  
        
        std::mutex camera_lock_;
        std::unique_ptr<Camera> camera_;

        std::unique_ptr<VideoGenerator> video_generator_;

        //std::unique_ptr<Stacker> stacker_;
        //std::thread stacker_thread_;
        
        std::shared_ptr<cppcms::service> web_service_;

    };
};
