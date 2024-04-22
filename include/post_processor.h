#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/hal/intrin.hpp>
#include <booster/log.h>
#include "common_data.h"

#include "simd_utils.h"

//#define DEBUG
#ifdef DEBUG
#include <opencv2/imgcodecs.hpp>
#endif


namespace ols {

    struct PostProcessor {
    public:
        float high_per_=99.99f;
        double gamma_limit_ = 4.0;
        double mean_target_ = 0.25;
        static constexpr int hist_bins=1024;
        int counters_[hist_bins];
        

        PostProcessor()
        {
            set_stretch(true,0,0,0.5f);
        }

        void set_stretch(bool auto_stretch,float low_index,float high_index,float stretch_index)
        {
            enable_stretch_ = auto_stretch;
            
            high_cut_ = std::max(1.0f,std::min(32.0f,high_index));
            low_cut_  = std::min(high_cut_ - 0.001f,std::max(0.0f,low_index));
            target_gamma_ = std::max(1.0f,std::min(8.0f,stretch_index));
        }

        std::vector<int> get_histogramm()
        {
            std::vector<int> res(counters_,counters_+hist_bins);
            return res;
        }
       

        static double tdiff(std::chrono::time_point<std::chrono::high_resolution_clock> const &a,
                            std::chrono::time_point<std::chrono::high_resolution_clock> const &b)
        {
            double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(b-a).count();
            return time * 1000;
        }

        float sum_rgb_line(float *p,int N,float line_sums[3])
        {
            float v[3]={};
            float maxv=0;
            int i=0;
#ifdef USE_CV_SIMD
            cv::v_float32x4 s[3];
            s[0] = cv::v_setzero_f32();
            s[1] = cv::v_setzero_f32();
            s[2] = cv::v_setzero_f32();

            int limit = N/12*12;
            cv::v_float32x4 val;
            for(;i<limit;i+=12,p+=12) {
                cv::v_float32x4 c[3];
                cv::v_load_deinterleave(p,c[0],c[1],c[2]);
                s[0]+=c[0];
                s[1]+=c[1];
                s[2]+=c[2];

                auto max_rgb = cv::v_reduce_max(cv::v_max(cv::v_max(c[0],c[1]),c[2]));
                maxv = std::max(maxv,max_rgb);
            }
            v[0] = cv::v_reduce_sum(s[0]);
            v[1] = cv::v_reduce_sum(s[1]);
            v[2] = cv::v_reduce_sum(s[2]);
#endif      
            for(;i<N;i+=3) {
                float c1=*p++;
                float c2=*p++;
                float c3=*p++;
                maxv = std::max(std::max(maxv,c1),std::max(c2,c3));

                v[0]+=c1;
                v[1]+=c2;
                v[2]+=c3;
            }
            for(int c=0;c<3;c++) {
                line_sums[c]+=v[c];
            }
            return maxv;
        }

        void scale_mono_and_clip(cv::Mat &m,float f1)
        {
            float *p = (float *)m.data;
            int N = m.rows*m.cols;
            int i=0;
#ifdef USE_CV_SIMD
            cv::v_float32x4 w = cv::v_setall_f32(f1);
            cv::v_float32x4 zero = cv::v_setzero_f32();
            cv::v_float32x4 one  = cv::v_setall_f32(1.0f);
            int limit = N/4*4;
            for(;i<limit;i+=4,p+=4) {
                cv::v_store(p,cv::v_max(zero,cv::v_min(one,cv::v_load(p+0)*w)));
            }
#endif            
            for(;i<N;i++,p++) {
                p[0] = std::max(0.0f,std::min(1.0f,p[0] * f1));
            }
        }

        void scale_rgb_and_clip(cv::Mat &m,float f1,float f2,float f3)
        {
            float *p = (float *)m.data;
            int N = m.rows*m.cols*3;
            int i=0;
#ifdef USE_CV_SIMD
            float w[12]={f1,f2,f3,f1, f2,f3,f1,f2, f3,f1,f2,f3};
            cv::v_float32x4 w0 = cv::v_load(w+0);
            cv::v_float32x4 w4 = cv::v_load(w+4);
            cv::v_float32x4 w8 = cv::v_load(w+8);
            cv::v_float32x4 zero = cv::v_setzero_f32();
            cv::v_float32x4 one  = cv::v_setall_f32(1.0f);
            int limit = N/12*12;
            for(;i<limit;i+=12,p+=12) {
                cv::v_store(p+0,cv::v_max(zero,cv::v_min(one,cv::v_load(p+0)*w0)));
                cv::v_store(p+4,cv::v_max(zero,cv::v_min(one,cv::v_load(p+4)*w4)));
                cv::v_store(p+8,cv::v_max(zero,cv::v_min(one,cv::v_load(p+8)*w8)));
            }
#endif            
            for(;i<N;i+=3,p+=3) {
                p[0] = std::max(0.0f,std::min(1.0f,p[0] * f1));
                p[1] = std::max(0.0f,std::min(1.0f,p[1] * f2));
                p[2] = std::max(0.0f,std::min(1.0f,p[2] * f3));
            }
        }

        void offset_scale_and_clip(cv::Mat &m,float offset,float scale)
        {
            float *p = (float *)m.data;
            int N = m.rows*m.cols*m.channels();
            int i=0;
#ifdef USE_CV_SIMD
            cv::v_float32x4 zero = cv::v_setzero_f32();
            cv::v_float32x4 one  = cv::v_setall_f32(1.0f);
            cv::v_float32x4 vscale = cv::v_setall_f32(scale);
            cv::v_float32x4 voffset = cv::v_setall_f32(offset);
            for(;i<(N / 4) * 4;i+=4,p+=4) {
                cv::v_float32x4 v = cv::v_load(p);
                v = cv::v_max(zero,cv::v_min(one,(v+voffset)*vscale));
                cv::v_store(p,v);
            }
#endif            
            for(;i<N;i++,p++) {
                float v = *p;
                v = std::max(0.0f,std::min(1.0f,(v+offset)*scale));
                *p = v;
            }
        }
        void offset_scale_and_clip_gamma(cv::Mat &m,float offset,float scale,float gamma)
        {
            float *p = (float *)m.data;
            int N = m.rows*m.cols*m.channels();
            float invg= 1.0f/gamma;
            constexpr int M=128;
            float table[M+1];
            //prepare_power_curve(M,table,invg);
            prepare_asinh_curve(M,table,invg);
            int i=0;
#ifdef USE_CV_SIMD
            int limit = N / 4 * 4;
            
            cv::v_float32x4 one = cv::v_setall_f32(1.0f);
            cv::v_float32x4 zero = cv::v_setzero_f32();
            cv::v_float32x4 voffset = cv::v_setall_f32(offset);
            cv::v_float32x4 vscale = cv::v_setall_f32(scale);

            for(i=0;i<limit;i+=4,p+=4) {
                cv::v_float32x4 v = cv::v_load(p);
                v = cv::v_min(one,cv::v_max(zero,(v+voffset) * vscale));
                curve_simd(v,M,table);
                cv::v_store(p,v);
            }
#endif
            for(;i<N;i++,p++) {
                float v = *p;
                v = std::min(1.0f,std::max(0.0f,(v+offset)*scale));
                v = curve_one(v,M,table); 
                *p = v;
            }
        }

        std::pair<cv::Mat,StretchInfo> post_process_image(cv::Mat raw_image,cv::Rect fully_stacked_area)
        {
            typedef  std::chrono::time_point<std::chrono::high_resolution_clock> tp;
            tp raw  = std::chrono::high_resolution_clock::now();
            cv::Mat tmp = raw_image;
            
            tp start = std::chrono::high_resolution_clock::now();
            tp wb_coeff,wb_apply;
            if(raw_image.channels() == 3) {
                float scale[3];
                calc_wb(tmp(fully_stacked_area),scale);
                wb_coeff = std::chrono::high_resolution_clock::now();
                scale_rgb_and_clip(tmp,scale[0],scale[1],scale[2]);
                wb_apply = std::chrono::high_resolution_clock::now();
            }
            double gscale=1.0;
            double goffset = 0.0;
            double mean = 0.5;
            float gamma_correction = 1.0f;
            
            int N = calc_hist(tmp(fully_stacked_area));
            if(enable_stretch_) {
                stretch(N,gscale,goffset,mean);
                gamma_correction = cv::max(1.0,cv::min(gamma_limit_,log(mean)/log(mean_target_)));
                BOOSTER_INFO("stacker") << "Stretch mean " << mean << "-> gamma=" << gamma_correction;
            }
            else {
                gamma_correction = target_gamma_;
                gscale = high_cut_;
                goffset = -low_cut_ / gscale;
            }
            tp calc_stretch = std::chrono::high_resolution_clock::now();
            if(gamma_correction == 1.0) {
                offset_scale_and_clip(tmp,goffset,gscale);
            }
            else {
                offset_scale_and_clip_gamma(tmp,goffset,gscale,gamma_correction);
            }
            tp apply_stretch =  std::chrono::high_resolution_clock::now();

            BOOSTER_INFO("stacker") << "get img=" << tdiff(raw,start) << "ms calc wb=" << tdiff(start,wb_coeff) <<"ms apply wb=" << tdiff(wb_coeff,wb_apply) << "ms calc stretch" << tdiff(wb_apply,calc_stretch) 
                << "ms stretch="<<tdiff(calc_stretch,apply_stretch) << "ms";

            StretchInfo stretch;
            stretch.gamma = gamma_correction;
            stretch.gain = gscale;
            stretch.cut = - goffset * gscale;
            stretch.auto_stretch = enable_stretch_;
            return std::make_pair(tmp,stretch);
        }

    private:
        int calc_hist(cv::Mat img)
        {
            memset(counters_,0,sizeof(counters_));
            int N=img.rows*img.cols;
            if(img.channels() == 3) {
                for(int r=0;r<img.rows;r++) {
                    float *p = (float*)(img.data + img.step[0] * r);
                    for(int c=0;c<img.cols;c++) {
                        float R = *p++;
                        float G = *p++;
                        float B = *p++;
                        unsigned Y = unsigned((0.3f * R + 0.6f * G + 0.1f * B) * (hist_bins-1));
                        Y &= (hist_bins - 1); // protect gainst overflow
                        counters_[Y]++;
                    }
                }
            }
            else {
                for(int r=0;r<img.rows;r++) {
                    float *p = (float*)(img.data + img.step[0] * r);
                    for(int c=0;c<img.cols;c++) {
                        unsigned Y = (hist_bins-1) * *p++;
                        Y &= (hist_bins - 1); // protect gainst overflow
                        counters_[Y]++;
                    }
                }
            }
            return N;
        }
        void stretch(int N,double &scale,double &offset,double &mean)
        {

            int sum=N;
            int hp=-1;
            for(int i=hist_bins-1;i>=0;i--) {
                sum-=counters_[i];
                if(sum*100.0f/N <= high_per_) {
                    hp = i;
                    break;
                }
            }
            int end = hp / 2;
            int max_diff = 0;
            for(int i=0;i<end-1;i++) {
                int diff = counters_[i+1] - counters_[i];
                max_diff = std::max(diff,max_diff);
            }
            int lp = 0;
            for(int i=0;i<end-1;i++) {
                lp = i;
                int diff = counters_[i+1] - counters_[i];
                if(diff * 10 >= max_diff)
                    break;
            }
            scale = (hist_bins-1.0)/hp;
            offset = -lp/(hist_bins - 1.0);
            
            mean = 0;
            int total = 0;
            for(int i=0;i<lp;i++) {
                total += counters_[i];
            }
            for(int i=lp;i<=hp;i++) {
                mean += (i - lp) / (hist_bins - 1.0) * counters_[i] * scale;
                total += counters_[i];
            }
            for(int i=hp+1;i<hist_bins;i++) {
                mean += counters_[i] * scale;
                total += counters_[i];
            }
            mean = mean / total;
            BOOSTER_INFO("stacker") << "Scale " << scale << " offset=" << (-offset) << " relative cut =" << (-offset*scale) ;
        }

        void calc_wb(cv::Mat img,float scale[3])
        {
            float *base = (float*)img.data;
            int stride = img.step1(0);
            int rows = img.rows;
            int cols = img.cols;
            float maxV = 0;
            scale[0]=scale[1]=scale[2] = 0;
            for(int r=0;r<rows;r++) {
                float *p = base + stride*r;
                float line_max = sum_rgb_line(p,cols*3,scale);
                maxV = std::max(maxV,line_max);
            }
            // normalize once again
            float smax = std::max(scale[0],std::max(scale[1],scale[2]));
            float smin = std::min(scale[0],std::min(scale[1],scale[2]));
            if(smin > 0) {
                for(int i=0;i<3;i++)
                    scale[i] = smax / scale[i];
            }
            else {
                for(int i=0;i<3;i++)
                    scale[i] = 1.0;
            }
            BOOSTER_INFO("stacker") << "WB " << scale[0]<<"," << scale[1] << "," << scale[2];
        }


        bool enable_stretch_ = true;
        float low_cut_ = 0.0f;
        float high_cut_ = 1.0f;
        float target_gamma_ = 2.2f;

    };



}
