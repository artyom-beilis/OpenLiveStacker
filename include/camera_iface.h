#pragma once
#include "camera.h"
namespace ols {
    class CameraInterface {
    public:
        enum CamStatus {
            cam_closed,
            cam_open,
            cam_streaming  
        };
        virtual ~CameraInterface() {}
        virtual std::recursive_mutex &lock() = 0;
        virtual CamStatus status() = 0;
        virtual Camera &cam() = 0;
        virtual void open_camera(int id) = 0;
        virtual void close_camera() = 0;
        virtual void start_stream(CamStreamFormat format) = 0;
        virtual CamStreamFormat stream_format() = 0;
        virtual void stop_stream() = 0;
        virtual CameraDriver &driver() = 0;
    };
}
