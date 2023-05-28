#pragma once
#include <memory>
#include <opencv2/core.hpp>
#include "sync_queue.h"
#include "video_frame.h"
#include "camera.h"
#include "common_data.h"

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

    struct StatsData : public QueueData {
        int stacked = 0;
        int missed = 0;
        int dropped = 0;
        virtual ~StatsData() {}
    };

    struct ErrorNotificationData : public QueueData {
        std::string source;
        std::string message;
        virtual ~ErrorNotificationData() {}
    };

    struct CameraFrame : public QueueData {
        CamStreamFormat format;
        CamBayerType bayer = bayer_na;
        double timestamp;
        std::shared_ptr<VideoFrame> source_frame;
        std::shared_ptr<VideoFrame> jpeg_frame;
        cv::Mat raw;
        cv::Mat frame;
        int frame_dr = 255;
        cv::Mat processed_frame;
        StretchInfo stretch;
        bool live_is_stretched = false;
        int dropped = 0;
    };

    struct LiveControl : public QueueData {
        LiveControl(CamOptionId o=opt_count,double v=0) :
            opt(o),
            value(v)
        {
        }
        CamOptionId opt;
        double value;
        virtual ~LiveControl() {}
    };

    struct StackerControl : public QueueData {
        enum ControlType {
            ctl_init,     /// start stacking
            ctl_pause,    /// pause stacking (for non tracking mount)
            ctl_resume,   /// resume paused stacking
            ctl_save,     /// save result and complete
            ctl_cancel,   /// cancel processing and discard the result
            ctl_update /// update strectch parameters
        };
        ControlType op = ctl_init;

        std::string output_path; /// identification of stacking
        std::string name;
        bool save_inputs = false;
        int width = 0;
        int height = 0;
        bool mono = false; // is mono data
        bool calibration = false;  /// Collect calibration data
        bool remove_satellites = false; // apply sat removal algorithm

        bool derotate = false; /// enable auto derote for AZ mount
        bool derotate_mirror = false; /// inverse direction for mirror image
        double ra=-1,de=-1,lat=-1,lon=-1; /// object data an

        double source_gamma = 1.0; /// to linear space gamma

        std::string darks_path; /// if darks applied darks frame
        std::string flats_path;
        std::string dark_flats_path;
        
        bool auto_stretch = true; /// stretch parameters
        double stretch_low=0.5,stretch_high=0.5,stretch_gamma=0.5;
    };
}
