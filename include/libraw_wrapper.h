#pragma once
#include "camera.h"
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
    
    inline std::pair<cv::Mat,CamBayerType> load_libraw(LibRaw &raw)
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
        cv::Mat raw_image = full(cv::Rect(raw.imgdata.sizes.left_margin,raw.imgdata.sizes.top_margin,raw.imgdata.sizes.width,raw.imgdata.sizes.height));
        int black = raw.imgdata.color.black;
        int maxv  = raw.imgdata.color.maximum;
        int scale = 65535 / (maxv - black);
        std::string bayer_name = raw_index2color(raw);
        CamBayerType bayer = bayer_type_from_str(bayer_name);
        img = (raw_image - cv::Scalar::all(black));
        img = img.mul(cv::Scalar::all(scale));

        return std::make_pair(img,bayer);
    }
    
    inline std::pair<cv::Mat,CamBayerType> load_libraw(std::string const &path)
    {
        LibRaw raw;
        int code=0;
        if((code = raw.open_file(path.c_str()))!=LIBRAW_SUCCESS)
            throw std::runtime_error("Failed to open raw file " + path + ": " + libraw_strerror(code));
        return load_libraw(raw);
    }
    inline std::pair<cv::Mat,CamBayerType> load_libraw(char *data,size_t size)
    {
        LibRaw raw;
        int code=0;
        if((code = raw.open_buffer(data,size))!=LIBRAW_SUCCESS) {
            throw std::runtime_error("Failed to open raw data : " + std::string(libraw_strerror(code)));
        }
        return load_libraw(raw);
    }
}
