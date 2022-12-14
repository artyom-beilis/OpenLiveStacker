#pragma once
#include <memory>
#include <opencv2/core.hpp>
#include "sync_queue.h"
#include "video_frame.h"
#include "camera.h"

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
        cv::Mat processed_frame;
    };

    struct StackerControl : public QueueData {
        enum ControlType {
            ctl_init,     /// start stacking
            ctl_pause,    /// pause stacking (for non tracking mount)
            ctl_resume,   /// resume paused stacking
            ctl_save,     /// save result and complete
            ctl_save_and_continue,  /// save intermediate result and continue
            ctl_update /// update strectch parameters
        };
        ControlType op = ctl_init;

        std::string output_path; /// identification of stacking
        std::string name;
        bool save_inputs = false;
        int width = 0;
        int height = 0;
        bool calibration = false;  /// Collect calibration data

        bool derotate = false; /// enable auto derote for AZ mount
        bool derotate_mirror = false; /// inverse direction for mirror image
        double ra=-1,de=-1,lat=-1,lon=-1; /// object data an

        double source_gamma = 1.0; /// to linear space gamma

        std::string darks_path; /// if darks applied darks frame
        
        bool auto_stretch = true; /// stretch parameters
        double strech_low=0,strech_high=1,strech_gamma=2.2;
    };
}
