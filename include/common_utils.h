#pragma once
#include <memory>

namespace ols {
    struct CameraFrame;
    std::shared_ptr<CameraFrame> generate_dummy_frame(int w,int h,int r=192,int g=192,int b=192);
}
