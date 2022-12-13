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
    "auto_exp", "auto_wb", "exp", "wb", "gain", "gamma", "brightness","contrast",
};
static char const *option_names[] = {
    "Auto Exposure", "Auto White Balance", "Exposure", "White Balance", "Gain", "Gamma", "Brightness", "Contrast"
};

std::string cam_option_id_to_string_id(CamOptionId id)
{
    size_t nid = id;
    if(nid >= sizeof(option_string_ids)/sizeof(option_string_ids[0]))
        throw CamError("Invalid option id");
    return option_string_ids[nid];
}

std::string stream_type_to_str(CamStreamType s)
{
    switch(s) {
    case stream_yuv2: return "yuv2";
    case stream_mjpeg: return "mjpeg";
    default:
        throw CamError("Invalid Stream type");
    }
}

CamStreamType stream_type_from_str(std::string s)
{
    if(s=="mjpeg")
        return stream_mjpeg;
    if(s=="yuv2")
        return stream_yuv2;
    throw CamError("Invalid stream type " + s);
}


static char const *cam_option_type_names[]={"bool","number","msec","percent","kelvin"};

std::string cam_option_type_to_str(CamOptionType type)
{
    unsigned id = type;
    if(id >= sizeof(cam_option_type_names)/sizeof(cam_option_type_names[0]))
        throw CamError("Invalid type");
    return cam_option_type_names[id];
}
CamOptionType cam_option_type_from_str(std::string const &name)
{
    for(unsigned i=0;i<sizeof(cam_option_type_names)/sizeof(cam_option_type_names[0]);i++) {
        if(cam_option_type_names[i] == name) {
            return static_cast<CamOptionType>(i);
        }
    }
    throw CamError("Invalid type:" + name);
}


std::string cam_option_id_to_name(CamOptionId id)
{
    size_t nid = id;
    if(nid >= sizeof(option_names)/sizeof(option_names[0]))
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
