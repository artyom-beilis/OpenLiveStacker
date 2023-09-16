#pragma once
#include "camera.h"
#include <libraw/libraw.h>
#include <opencv2/core.hpp>
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

    inline cv::Mat apply_bin2(cv::Mat in)
    {
        cv::Mat out(in.rows/2,in.cols/2,CV_16UC1);
        int stride = in.step[0]/2;
        for(int r=0;r<out.rows;r+=2) {
            uint16_t *r0 = reinterpret_cast<uint16_t*>(in.data) + stride * r * 2;
            uint16_t *r1 = r0 + stride;
            uint16_t *r2 = r0 + stride * 2;
            uint16_t *r3 = r0 + stride * 3;

            uint16_t *or0 = reinterpret_cast<uint16_t*>(out.data) + out.cols * r;
            uint16_t *or1 = or0 + out.cols;
            for(int c=0;c<out.cols;c+=2) {
                uint16_t v00 = (2u + r0[0] + r0[2] + r2[0] + r2[2]) / 4;
                uint16_t v01 = (2u + r0[1] + r0[3] + r2[1] + r2[3]) / 4;
                uint16_t v10 = (2u + r1[0] + r1[2] + r3[0] + r3[2]) / 4;
                uint16_t v11 = (2u + r1[1] + r1[3] + r3[1] + r3[3]) / 4;
                or0[0] = v00;
                or0[1] = v01;
                or1[0] = v10;
                or1[1] = v11;
                r0+=4;
                r1+=4;
                r2+=4;
                r3+=4;
                or0 += 2;
                or1 += 2;
            }
        }
        return out;
    }
    inline cv::Mat apply_bin3(cv::Mat in)
    {
        cv::Mat out(in.rows/3,in.cols/3,CV_16UC1);
        int stride = in.step[0]/2;
        for(int r=0;r<out.rows;r+=2) {
            uint16_t *r0 = reinterpret_cast<uint16_t*>(in.data) + stride * r * 3;
            uint16_t *r1 = r0 + stride;
            uint16_t *r2 = r0 + stride * 2;
            uint16_t *r3 = r0 + stride * 3;
            uint16_t *r4 = r0 + stride * 4;
            uint16_t *r5 = r0 + stride * 5;

            uint16_t *or0 = reinterpret_cast<uint16_t*>(out.data) + out.cols * r;
            uint16_t *or1 = or0 + out.cols;
            for(int c=0;c<out.cols;c+=2) {
                uint16_t v00 = (4u + r0[0] + r0[2] + r0[4] + r2[0] + r2[2] + r2[4] + r4[0] + r4[2] + r4[4]) / 9;
                uint16_t v01 = (4u + r0[1] + r0[3] + r0[5] + r2[1] + r2[3] + r2[5] + r4[1] + r4[3] + r4[5]) / 9;
                uint16_t v10 = (4u + r1[0] + r1[2] + r1[4] + r3[0] + r3[2] + r3[4] + r5[0] + r5[2] + r5[4]) / 9;
                uint16_t v11 = (4u + r1[1] + r1[3] + r1[5] + r3[1] + r3[3] + r3[5] + r5[1] + r5[3] + r5[5]) / 9;

                or0[0] = v00;
                or0[1] = v01;
                or1[0] = v10;
                or1[1] = v11;
                r0+=6;
                r1+=6;
                r2+=6;
                r3+=6;
                r4+=6;
                r5+=6;
                or0 += 2;
                or1 += 2;
            }
        }
        return out;
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
        switch(bin) {
        case 1: break;
        case 2: raw_image = apply_bin2(raw_image); break;
        case 3: raw_image = apply_bin3(raw_image); break;
        default:
            throw std::runtime_error("Invalid bin value");
        }
        int black = raw.imgdata.color.black;
        int maxv  = raw.imgdata.color.maximum;
        int scale = 65535 / (maxv - black);
        std::string bayer_name = raw_index2color(raw);
        CamBayerType bayer = bayer_type_from_str(bayer_name);
        img = (raw_image - cv::Scalar::all(black));
        img = img.mul(cv::Scalar::all(scale));

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
