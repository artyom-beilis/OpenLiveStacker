#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/hal/intrin.hpp>

namespace ols {
#ifdef USE_CV_SIMD
    template<typename VType>
    struct VTraits;

    template<>
    struct VTraits<uint16_t> {
        typedef cv::v_uint16x8 vtype;
    };

    template<>
    struct VTraits<uint8_t> {
        typedef cv::v_uint8x16 vtype;
    };
    
    template<typename VType,typename UInt,int Delta>
    static std::pair<VType,VType> load3x3v(UInt *r_prev,UInt *r_curr,UInt *r_next)
    {
        VType max_v,min_v,v;
        max_v = min_v = cv::v_load(r_prev + 0);
        v = cv::v_load(r_prev + Delta);      max_v=cv::v_max(max_v,v); min_v=cv::v_min(min_v,v);
        v = cv::v_load(r_prev + 2*Delta);    max_v=cv::v_max(max_v,v); min_v=cv::v_min(min_v,v);
        v = cv::v_load(r_next + 0);          max_v=cv::v_max(max_v,v); min_v=cv::v_min(min_v,v);
        v = cv::v_load(r_next + Delta);      max_v=cv::v_max(max_v,v); min_v=cv::v_min(min_v,v);
        v = cv::v_load(r_next + 2*Delta);    max_v=cv::v_max(max_v,v); min_v=cv::v_min(min_v,v);
        v = cv::v_load(r_curr + 0);          max_v=cv::v_max(max_v,v); min_v=cv::v_min(min_v,v);
        v = cv::v_load(r_curr + 2*Delta);    max_v=cv::v_max(max_v,v); min_v=cv::v_min(min_v,v);
        return std::make_pair(min_v,max_v); 
    }
#endif
    template<typename UInt,int Delta>
    std::pair<UInt,UInt> load3x3(UInt *r_prev,UInt *r_curr,UInt *r_next)
    {
        UInt max_v,min_v,v;
        max_v = min_v = r_prev[0];
        v = r_prev[Delta];      max_v=std::max(max_v,v); min_v=std::min(min_v,v);
        v = r_prev[2*Delta];    max_v=std::max(max_v,v); min_v=std::min(min_v,v);
        v = r_next[0];          max_v=std::max(max_v,v); min_v=std::min(min_v,v);
        v = r_next[Delta];      max_v=std::max(max_v,v); min_v=std::min(min_v,v);
        v = r_next[2*Delta];    max_v=std::max(max_v,v); min_v=std::min(min_v,v);
        v = r_curr[0];          max_v=std::max(max_v,v); min_v=std::min(min_v,v);
        v = r_curr[2*Delta];    max_v=std::max(max_v,v); min_v=std::min(min_v,v);
        return std::make_pair(min_v,max_v); 
    }
    template<typename UInt,int Delta>
    void remove_hot_pixels_impl(cv::Mat &in,cv::Mat &out)
    {
        int step = in.cols;
        for(int r=Delta;r<in.rows-Delta;r++) {
            int c=Delta;
            UInt *p = (UInt*)in.data  + r*step + c;
            UInt *o = (UInt*)out.data + r*step + c;                
#ifdef USE_CV_SIMD
            typedef typename VTraits<UInt>::vtype vtype;
            constexpr int elems = 16/sizeof(UInt);
            int N = (in.cols - 2*Delta) / elems;
            for(int i=0;i<N;i++) {
                auto range = load3x3v<vtype,UInt,Delta>(p-Delta*step - Delta,p-Delta,p+Delta*step-Delta);
                vtype px = cv::v_load(p);
                vtype diff = px - range.second;
                vtype grad = range.second - range.first;
                auto mask = (px > range.second) & (diff > grad);
                vtype res = cv::v_select(mask,range.second,px);
                cv::v_store(o,res);
                
                p+= elems;
                o+= elems;
                c+= elems;
            }
#endif            
            for(;c<in.cols-Delta;c++,p++,o++) {
                auto range = load3x3<UInt,Delta>(p-Delta*step - Delta,p-Delta,p+Delta*step-Delta);
                UInt px = p[0];
                if(px > range.second) {
                    UInt diff = px - range.second;
                    if(diff > range.second - range.first)
                        o[0] = range.second;
                }
            }
        }
    }
    cv::Mat remove_hot_pixels(cv::Mat in,bool mono=false)
    {
        if(in.elemSize1() > 2 || in.channels()!=1)
            return in;
        cv::Mat res = in.clone();
        if(in.elemSize1() == 1) {
            if(mono)
                remove_hot_pixels_impl<uint8_t,1>(in,res);
            else
                remove_hot_pixels_impl<uint8_t,2>(in,res);
        }
        else if(in.elemSize1() == 2) {
            if(mono)
                remove_hot_pixels_impl<uint16_t,1>(in,res);
            else
                remove_hot_pixels_impl<uint16_t,2>(in,res);
        }
        return res;
    }
}
