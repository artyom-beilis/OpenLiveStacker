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

static char const *option_string_ids[] = {
    "auto_exp", "auto_wb", "exp", "wb", "gain", "gamma"
};
static char const *option_names[] = {
    "Auto Exposure", "Auto White Balance", "Exposure", "White Balance", "Gain", "Gamma"
};

std::string cam_option_id_to_string_id(CamOptionId id)
{
    int nid = id;
    if(nid < 0 || nid >= sizeof(option_string_ids)/sizeof(option_string_ids[0]))
        throw CamError("Invalid option id");
    return option_string_ids[nid];
}


std::string cam_option_id_to_name(CamOptionId id)
{
    int nid = id;
    if(nid < 0 || nid >= sizeof(option_names)/sizeof(option_names[0]))
        throw CamError("Invalid option id");
    return option_names[nid];
}

CamOptionId cam_option_id_from_string_id(std::string const &name)
{
    for(size_t i=0;i<sizeof(option_string_ids)/sizeof(option_string_ids[0]);i++)
        if(name == option_string_ids[i])
            return static_cast<CamOptionId>(i);
    throw CamError("Invalid option id");
}


} // ols
