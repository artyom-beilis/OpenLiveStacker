#pragma once
#include <memory>
#include <opencv2/core.hpp>
#include "sync_queue.h"
#include "video_frame.h"

namespace ols {
    struct QueueData {
        virtual ~QueueData() {}
    };

    typedef std::shared_ptr<QueueData> data_pointer_type;
    typedef sync_queue<data_pointer_type> queue_type;
    typedef std::shared_ptr<queue_type> queue_pointer_type;

    struct ShutDownData : public QueueData {
        virtual ~ShutDownData() {}
    };

    struct CameraFrame : public QueueData {
        CamStreamFormat format;
        double timestamp;
        std::shared_ptr<VideoFrame> source_frame;
        std::shared_ptr<VideoFrame> jpeg_frame;
        cv::Mat frame;
        cv::Mat post_processed_frame;
    };

    struct StackerControl : public QueueData {
        enum ControlType {
            ctl_init,
            ctl_pause,
            ctl_resume,
            ctl_save,
            ctl_save_and_continue
        };
        ControlType op = ctl_init;

        double ra=-1,de=-1,lat=-1,lon=-1;
        double soruce_gamma = 1.0;
        std::string frame_id;
        bool calibration = false;
        bool auto_stretch = true;
        double strech_low=0,strech_high=1,strech_gamma=2.2;
    };
}
