#pragma once
#include <opencv2/core.hpp>
#include "camera.h"
namespace ols {
    std::pair<cv::Mat,CamBayerType> load_fits(std::string file);
}
