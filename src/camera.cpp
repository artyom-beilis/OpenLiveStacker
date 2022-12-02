#include "camera.h"
#include "uvc_camera.h"

namespace ols {
std::vector<std::string> CameraDriver::drivers()
{
    return std::vector<std::string>({"UVC"});
}

std::unique_ptr<CameraDriver> CameraDriver::get(int id)
{
    if(id!=0)
        throw CamError("Invalid driver id"); 
    return get_uvc_driver();
}
std::ostream &operator<<(std::ostream &out,CamStreamFormat const &fmt)
{
    switch(fmt.format) {
    case stream_yuv2: out << "YUV2"; break;
    case stream_mjpeg: out << "MJPEG"; break;
    default: out << "Unknown";
    }
    out << ":" << fmt.width << "x" << fmt.height <<"@" << fmt.framerate;
    return out;
}
} // ols
