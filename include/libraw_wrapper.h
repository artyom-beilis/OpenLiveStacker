#pragma once
#include "camera.h"
#include "sw_bin.h"
#include <libraw/libraw.h>
namespace ols {
    
    inline std::string raw_index2color(LibRaw &raw)
    {
        std::string res;
        int pat_index[4] = {raw.COLOR(0,0),raw.COLOR(0,1),raw.COLOR(1,0),raw.COLOR(1,1)};
        for(int i=0;i<4;i++) {
            char const *v="RGBG";
            if(pat_index[i] < 0 || pat_index[i]>3)
                res += '?';
            else
                res += v[pat_index[i]];
        }
        return res;
    }


    inline std::pair<cv::Mat,CamBayerType> load_libraw(LibRaw &raw,int h=-1,int w=-1,int bin=1)
    {
        cv::Mat img;
        int code=0;
        if((code=raw.unpack())!=LIBRAW_SUCCESS)
        {
            throw std::runtime_error(std::string("Failed to unpack raw ") + libraw_strerror(code));
        }
        #if 0
        printf("%p raw=%dx%d act=%dx%d stride_bytes %d pat=%s\n",raw.imgdata.rawdata.raw_image,
                raw.imgdata.sizes.raw_height,raw.imgdata.sizes.raw_width,
                raw.imgdata.sizes.height,raw.imgdata.sizes.width,
                raw.imgdata.sizes.raw_pitch,
                raw.imgdata.idata.cdesc);
        printf("max=%d black=%d data_max=%d\n",raw.imgdata.color.maximum,raw.imgdata.color.black,raw.imgdata.color.data_maximum);
        #endif

        if(!raw.imgdata.rawdata.raw_image) {
            throw std::runtime_error("Is not 16 bit bayer");
        }
        cv::Mat full(raw.imgdata.sizes.raw_height,raw.imgdata.sizes.raw_width,CV_16UC1,raw.imgdata.rawdata.raw_image,raw.imgdata.sizes.raw_pitch);
        int dh=0,dw=0;
        int binfact = 2 * bin;
        if(w!=-1 && h!=-1) {
            w*=bin;
            h*=bin;
            dh = (raw.imgdata.sizes.height - h) / 4 * 2 ;
            dw = (raw.imgdata.sizes.width  - w) / 4 * 2 ;
            dh = dh / 2 * 2;
            dw = dw / 2 * 2;
        }
        else {
            w=raw.imgdata.sizes.width  / binfact * binfact;
            h=raw.imgdata.sizes.height / binfact * binfact;
        }
        if(w % binfact != 0 || h % binfact != 0 || h > raw.imgdata.sizes.raw_height || w > raw.imgdata.sizes.raw_width) {
            char buf[256];
            snprintf(buf,sizeof(buf),"w=%d mod %d != 0 || h=%d mod %d != 0 || h > %d || w > %d",w,binfact,h,binfact,raw.imgdata.sizes.raw_height,raw.imgdata.sizes.raw_width);
            throw std::runtime_error(buf);
        }
        cv::Mat raw_image = full(cv::Rect(raw.imgdata.sizes.left_margin+dw,raw.imgdata.sizes.top_margin+dh,w,h));
        int black = raw.imgdata.color.black;
        int maxv  = raw.imgdata.color.maximum;
        int scale = 65535 / (maxv - black);
        switch(bin) {
        case 1: img = (raw_image - cv::Scalar::all(black));
                img = img.mul(cv::Scalar::all(scale)); break;
        case 2: img = apply_bin2(raw_image,scale,black); break;
        case 3: img = apply_bin3(raw_image,scale,black); break;
        default:
            throw std::runtime_error("Invalid bin value");
        }
        std::string bayer_name = raw_index2color(raw);
        CamBayerType bayer = bayer_type_from_str(bayer_name);

        return std::make_pair(img,bayer);
    }
    
    inline std::pair<cv::Mat,CamBayerType> load_libraw(std::string const &path,int h=-1,int w=-1,int bin=1)
    {
        LibRaw raw;
        int code=0;
        if((code = raw.open_file(path.c_str()))!=LIBRAW_SUCCESS)
            throw std::runtime_error("Failed to open raw file " + path + ": " + libraw_strerror(code));
        return load_libraw(raw,h,w,bin);
    }
    inline std::pair<cv::Mat,CamBayerType> load_libraw(char *data,size_t size,int h=-1,int w=-1,int bin=1)
    {
        LibRaw raw;
        int code=0;
        if((code = raw.open_buffer(data,size))!=LIBRAW_SUCCESS) {
            throw std::runtime_error("Failed to open raw data : " + std::string(libraw_strerror(code)));
        }
        return load_libraw(raw,h,w,bin);
    }
}
