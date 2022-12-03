#pragma once
namespace ols {
    class VideoGenerator {
    public:
        typedef std::function<void(std::shared_ptr<VideoFrame>)> callback_type;
        VideoGenerator(queue_pointer_type queue,callback_type callback): 
            broadcast_(callback),
            queue_(queue)
        {
        }
        void process_frame(std::shared_ptr<CameraFrame> frame)
        {
            if(frame->format.format == stream_mjpeg) {
                frame->jpeg_frame = frame->soruce_frame;
            }
            else {
                fprintf(stderr,"Only mjpeg video genetator is supported for now\n");
                return;
            }
            broadcast_(frame->jpeg_frame);
        }
        void run()
        {
            while(true) {
                auto data_ptr = data_queue_.pop();
                auto stop_ptr = std::dynamic_poimter_cast<std::shared_ptr<ShutDownData>>(data_ptr);
                if(stop_ptr)
                    break;
                auto frame_ptr = std::dynamic_poimter_cast<std::shared_ptr<CameraFrame>>(data_ptr);
                if(frame_ptr) {
                    process_frame(frame_ptr);
                    continue
                }
                fprintf(stderr,"Invalid data for video generator\n");
            }
        }
    private:
        callback_type broadcast_;
        queue_pointer_type data_queue_;
    };
}
