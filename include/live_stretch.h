#pragma once
#include <opencv2/core.hpp>
#include <opencv2/core/hal/intrin.hpp>
#include "simd_utils.h"
namespace ols {

    inline float calc_stretch_factor_from_hist(int total,int *bins,int size)
    {
        int limit = total * 0.0001;
        int S = 0;
        for(int i=size-1;i>=0;i--) {
            S+= bins[i];
            if(S >= limit) {
                return size / (i + 0.5);
            }
        }
        return 1.0f;
    }

    inline float calc_stretch_factor16(cv::Mat &in)
    {
        int bins[1024]={};
        int N = in.channels() * in.rows * in.cols;
        uint16_t *p = reinterpret_cast<uint16_t*>(in.data);
        for(int i=0;i<N;i++) {
            bins[(p[i]>>6)]++;
        }
        return calc_stretch_factor_from_hist(N,bins,1024);
    }
    inline float calc_stretch_factor8(cv::Mat &in)
    {
        int bins[256]={};
        int N = in.channels() * in.rows * in.cols;
        uint8_t *p = reinterpret_cast<uint8_t*>(in.data);
        for(int i=0;i<N;i++) {
            bins[p[i]]++;
        }
        return calc_stretch_factor_from_hist(N,bins,256);
    }

    inline void live_stretch(cv::Mat &in,float &factor,cv::Mat &out)
    {
        if(factor < 1) {
            if(in.elemSize1() == 1)
                factor = calc_stretch_factor8(in);
            else if(in.elemSize1() == 2)
                factor = calc_stretch_factor16(in);
            else
                throw std::runtime_error("Internal error Unsupported input format for live stretch");
        }
        double conv_factor = in.elemSize1() == 1 ? 1.0 : (255.0 / 65535.0);
        in.convertTo(out,in.channels() == 3 ? CV_8UC3: CV_8UC1,conv_factor * factor);
    }
    inline void live_stretch(cv::Mat &in,cv::Mat &out)
    {
        float factor = -1.0;
        live_stretch(in,factor,out);
    }
}

