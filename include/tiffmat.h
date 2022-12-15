#pragma once
#include <opencv2/core.hpp>
namespace ols {
    void save_tiff(cv::Mat data,std::string file,bool bgr2rgb=true);
    cv::Mat load_tiff(std::string file,bool bgr2rgb=true);
}
