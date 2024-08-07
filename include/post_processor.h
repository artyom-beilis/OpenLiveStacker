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
    class PostProcessorBase {
    public:
        
        virtual void set_stretch(bool /*auto_stretch*/,float /*low_index*/,float /*high_index*/,float /*stretch_index*/) 
        {
        }
        virtual void set_deconv(float /*sigma*/,int /*iterations*/) 
        {
        }
        virtual void set_unsharp_mask(float /*sigma*/,float /*strength*/)
        {
        }
        virtual std::vector<int> get_histogramm() {
            return std::vector<int>();
        }

        virtual std::pair<cv::Mat,StretchInfo> post_process_image(cv::Mat raw_image,cv::Rect fully_stacked_area) = 0;
        virtual ~PostProcessorBase() {}

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

        void apply_wb(cv::Mat img,cv::Rect area)
        {
            if(img.channels() == 3) {
                float scale[3];
                calc_wb(img(area),scale);
                scale_rgb_and_clip(img,scale[0],scale[1],scale[2]);
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

    };

    struct PlanetaryPostProcessor : public PostProcessorBase {
    public:
        PlanetaryPostProcessor() {
            set_deconv(3,10);
            set_unsharp_mask(3,1.5);
        }
        virtual void set_deconv(float sigma,int iterations) override
        {
            int size = std::ceil(sigma)*2+1;
            deconv_kern_ = limit_kern(size);
            deconv_sigma_ = sigma;
            deconv_iters_ = std::max(0,std::min(30,iterations));
        }
        virtual void set_unsharp_mask(float sigma,float strength) override
        {
            int size = std::ceil(sigma)*2+1;
            unsharp_size_ = limit_kern(size);
            unsharp_sigma_ = sigma;
            unsharp_strength_ = std::max(0.0f,std::min(10.0f,strength));
        }
        virtual std::pair<cv::Mat,StretchInfo> post_process_image(cv::Mat raw_image,cv::Rect fully_stacked_area) override
        {
            auto p1 = std::chrono::high_resolution_clock::now();
            cv::Mat img = raw_image.clone();
            apply_wb(img,fully_stacked_area);
            stretch(img);
            auto p2 = std::chrono::high_resolution_clock::now();
            gauss_time_ = 0;
            if(deconv_kern_ > 1 && deconv_iters_ > 1) {
                apply_deconv(img);
            }
            double deconv_g = gauss_time_;
            auto p3 = std::chrono::high_resolution_clock::now();
            if(unsharp_size_ > 1 && unsharp_strength_ > 0.0f) {
                apply_unsharp(img);
            }
            auto p4 = std::chrono::high_resolution_clock::now();
            BOOSTER_INFO("stacker") << "Planetary post processing wb/stretch:" << tdiff(p1,p2) <<" ms, deconvolution " << tdiff(p2,p3) << "ms (blur " << deconv_g << " ms) , unsharp " << tdiff(p3,p4) <<" ms";
            return std::make_pair(img,StretchInfo());
        }
        void stretch(cv::Mat &img)
        {
            double maxV=1.0;
            cv::minMaxLoc(img.reshape(1),nullptr,&maxV);
            double scale = 1.0 / maxV * 0.8;
            if(scale > 1.0) {
                img = img.mul(cv::Scalar::all(scale));
                fprintf(stderr,"Scale=%f\n",scale);
            }
        }
    private:
        void mpl_eps_clip(cv::Mat &am,cv::Mat &bm,float eps)
        {
            int N = am.channels()*am.rows*am.cols;
            int i=0;
            float *a=reinterpret_cast<float*>(am.data);
            float *b=reinterpret_cast<float*>(bm.data);
            #ifdef USE_CV_SIMD
            int limit = N/4*4;
            cv::v_float32x4 veps = cv::v_setall_f32(eps);
            cv::v_float32x4 zero = cv::v_setall_f32(0.0f);
            cv::v_float32x4 one  = cv::v_setall_f32(1.0f);
            for(;i<limit;i+=4) {
                cv::v_float32x4 val =  cv::v_load(a+i) * (cv::v_load(b+i) + veps);
                val = cv::v_max(zero,cv::v_min(one,val));
                cv::v_store(a+i,val);
            }
            #endif
            for(;i<N;i++) {
                a[i]=std::max(0.0f,std::min(1.0f,a[i]*(b[i]+eps)));
            }
        }
        void devide_eps(cv::Mat am,cv::Mat bm,cv::Mat &cm,float eps)
        {
            cm.create(am.rows,am.cols,am.channels()==1 ? CV_32FC1 : CV_32FC3);
            int N = am.channels()*am.rows*am.cols;
            int i=0;
            float *a=reinterpret_cast<float*>(am.data);
            float *b=reinterpret_cast<float*>(bm.data);
            float *c=reinterpret_cast<float*>(cm.data);
            #ifdef USE_CV_SIMD
            int limit = N/4*4;
            cv::v_float32x4 veps = cv::v_setall_f32(eps);
            for(;i<limit;i+=4) {
                cv::v_float32x4 val =  cv::v_load(a+i) / (cv::v_load(b+i) + veps);
                cv::v_store(c+i,val);
            }
            #endif
            for(;i<N;i++) {
                c[i]=a[i]/(b[i]+eps);
            }
        }

        void gaussian_blur(cv::Mat src,cv::Mat &tgt,int size,float sigma)
        {
            auto start =  std::chrono::high_resolution_clock::now();
            cv::GaussianBlur(src,tgt,cv::Size(size,size),sigma);
            auto end =  std::chrono::high_resolution_clock::now();
            gauss_time_ += tdiff(start,end);
        }

        void apply_unsharp(cv::Mat &image)
        {
            cv::Mat blur;
            gaussian_blur(image,blur,unsharp_size_,unsharp_sigma_);
            int N = image.channels()*image.rows*image.cols;
            int i=0;
            float *a=reinterpret_cast<float*>(image.data);
            float *b=reinterpret_cast<float*>(blur.data);
            float iw = unsharp_strength_ + 1.0f;
            #ifdef USE_CV_SIMD
            int limit = N/4*4;
            cv::v_float32x4 zero = cv::v_setzero_f32();
            cv::v_float32x4 one  = cv::v_setall_f32(1.0f);
            cv::v_float32x4 iw_v = cv::v_setall_f32(iw);
            cv::v_float32x4 s_v  = cv::v_setall_f32(unsharp_strength_);
            for(;i<limit;i+=4) {
                cv::v_float32x4 val =  cv::v_load(a+i) * iw_v - cv::v_load(b+i)*s_v;
                val = cv::v_max(zero,cv::v_min(one,val));
                cv::v_store(a+i,val);
            }
            #endif
            for(;i<N;i++) {
                float val = iw * a[i] - unsharp_strength_ * b[i];
                val = std::max(0.0f,std::min(1.0f,val));
                a[i] = val;
            }
            
        }

        // Richardson-Lucy deconvolution
        void apply_deconv(cv::Mat &image)
        {
            cv::Mat im_deconv(image.rows,image.cols,image.channels() == 3 ? CV_32FC3 : CV_32FC1,cv::Scalar::all(0.5));
            cv::Mat conv,relative_blur,tmp;
            float eps = 1e-12;
            for(int i=0;i<deconv_iters_;i++) {
                // psf blur
                gaussian_blur(im_deconv,conv,deconv_kern_,deconv_sigma_);
                // relative_blur = image/(conv+eps)
                devide_eps(image,conv,relative_blur,eps);
                // psf' blur - gaussian kernel is symmetric
                gaussian_blur(relative_blur,tmp,deconv_kern_,deconv_sigma_);
                // im_deconv = clip(im_deconv * (tmp+eps),0,1)
                mpl_eps_clip(im_deconv,tmp,eps);  
            }
            image = im_deconv;
        }
        int limit_kern(int size)
        {
            int new_size = std::max(1,std::min(11,(size / 2)*2 + 1));
            return new_size;
        }
        int deconv_kern_ = 1;
        float deconv_sigma_ = 1.0f;
        int deconv_iters_ = 1;

        int unsharp_size_ = 1;
        float unsharp_sigma_ = 1.0f;
        float unsharp_strength_ = 0.0f;
        double gauss_time_ = 0.0;
    };

    struct PostProcessor : public PostProcessorBase{
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

        void set_stretch(bool auto_stretch,float low_index,float high_index,float stretch_index) override
        {
            enable_stretch_ = auto_stretch;
            
            high_cut_ = std::max(1.0f,std::min(32.0f,high_index));
            low_cut_  = std::min(high_cut_ - 0.001f,std::max(0.0f,low_index));
            target_gamma_ = std::max(1.0f,std::min(8.0f,stretch_index));
        }

        std::vector<int> get_histogramm() override
        {
            std::vector<int> res(counters_,counters_+hist_bins);
            return res;
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

        std::pair<cv::Mat,StretchInfo> post_process_image(cv::Mat raw_image,cv::Rect fully_stacked_area) override
        {
            typedef  std::chrono::time_point<std::chrono::high_resolution_clock> tp;
            cv::Mat tmp = raw_image.clone();
            tp start = std::chrono::high_resolution_clock::now();
            apply_wb(tmp,fully_stacked_area);
            tp wb_apply = std::chrono::high_resolution_clock::now();
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

            BOOSTER_INFO("stacker") << "apply wb=" << tdiff(start,wb_apply) << "ms calc stretch" << tdiff(wb_apply,calc_stretch) 
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
            int end = hp - 1;
            int max_val = 0;
            int max_pos = 0;
            for(int i=0;i<end-1;i++) {
                int val = counters_[i];
                if(val > max_val) {
                    max_pos = i;
                    max_val = val;
                }
            }
            int lp = max_pos;
            while(lp > 1 && counters_[lp-1] * 2 >= max_val) {
                lp--;
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



        bool enable_stretch_ = true;
        float low_cut_ = 0.0f;
        float high_cut_ = 1.0f;
        float target_gamma_ = 2.2f;

    };



}
