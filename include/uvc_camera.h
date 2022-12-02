#pragma once
namespace ols {
    // get camera driver for descriptor
    std::unique_ptr<CameraDriver> get_uvc_driver(int fd = -1);
};
