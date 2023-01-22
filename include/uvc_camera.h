#pragma once
namespace ols {
    class CameraDriver;
    // get camera driver for descriptor
}

extern "C" {
    ols::CameraDriver *ols_get_uvc_driver(int fd);
}
