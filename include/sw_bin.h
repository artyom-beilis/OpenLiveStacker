#pragma once
#include <opencv2/core.hpp>
namespace ols {


    template<int D>
    inline uint16_t apply_scale_offset_div(int sum,int scale,int offset)
    {
        return std::min(65535,(std::max(sum - offset,0)*scale + D/2) / D);
    }

    inline void apply_bin2(cv::Mat in,cv::Mat &out,int scale,int offset)
    {
        offset *= 4;
        printf("%d %d\n",scale,offset);
        out.create(in.rows/2,in.cols/2,CV_16UC1);
        int stride = in.step[0]/2;
        for(int r=0;r<out.rows;r+=2) {
            uint16_t *r0 = reinterpret_cast<uint16_t*>(in.data) + stride * r * 2;
            uint16_t *r1 = r0 + stride;
            uint16_t *r2 = r0 + stride * 2;
            uint16_t *r3 = r0 + stride * 3;

            uint16_t *or0 = reinterpret_cast<uint16_t*>(out.data) + out.cols * r;
            uint16_t *or1 = or0 + out.cols;
            for(int c=0;c<out.cols;c+=2) {
                uint16_t v00 = apply_scale_offset_div<4>((int)r0[0] + r0[2] + r2[0] + r2[2],scale,offset);
                uint16_t v01 = apply_scale_offset_div<4>((int)r0[1] + r0[3] + r2[1] + r2[3],scale,offset);
                uint16_t v10 = apply_scale_offset_div<4>((int)r1[0] + r1[2] + r3[0] + r3[2],scale,offset);
                uint16_t v11 = apply_scale_offset_div<4>((int)r1[1] + r1[3] + r3[1] + r3[3],scale,offset);
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
    }
    inline void apply_bin3(cv::Mat in,cv::Mat &out,unsigned scale,int offset)
    {
        offset *= 9;
        out.create(in.rows/3,in.cols/3,CV_16UC1);
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
                uint16_t v00 = apply_scale_offset_div<9>((int)r0[0] + r0[2] + r0[4] + r2[0] + r2[2] + r2[4] + r4[0] + r4[2] + r4[4],scale,offset);
                uint16_t v01 = apply_scale_offset_div<9>((int)r0[1] + r0[3] + r0[5] + r2[1] + r2[3] + r2[5] + r4[1] + r4[3] + r4[5],scale,offset);
                uint16_t v10 = apply_scale_offset_div<9>((int)r1[0] + r1[2] + r1[4] + r3[0] + r3[2] + r3[4] + r5[0] + r5[2] + r5[4],scale,offset);
                uint16_t v11 = apply_scale_offset_div<9>((int)r1[1] + r1[3] + r1[5] + r3[1] + r3[3] + r3[5] + r5[1] + r5[3] + r5[5],scale,offset);

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
    }
    inline cv::Mat apply_bin2(cv::Mat in,int scale,int offset)
    {
        cv::Mat out;
        apply_bin2(in,out,scale,offset);
        return out;
    }
    inline cv::Mat apply_bin3(cv::Mat in,int scale,int offset)
    {
        cv::Mat out;
        apply_bin3(in,out,scale,offset);
        return out;
    }

} // namespace
