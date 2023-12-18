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

    struct Stacker {
    public:

        bool enable_subpixel_registration = false;
        int  subpixel_factor_ = 1;

        float high_per_=99.99f;
        double gamma_limit_ = 4.0;
        double mean_target_ = 0.25;
        static constexpr int hist_bins=1024;
        int counters_[hist_bins];
        

        Stacker(int width,int height,int channels,int roi_x=-1,int roi_y=-1,int roi_size = -1,int exp_multiplier=1) : 
            frames_(0),
            exp_multiplier_(exp_multiplier)
        {
            channels_ = channels;
            cv_type_ = channels == 1 ? CV_32FC1 : CV_32FC3;
            if(!enable_subpixel_registration || subpixel_factor_ == 1) {
                subpixel_factor_ = 1;
                enable_subpixel_registration = false;
            }

            set_stretch(true,0,0,0.5f);
            fully_stacked_area_ = cv::Rect(0,0,width,height);
            fully_stacked_count_ = 0;
            if(roi_size == -1) {
                window_size_ = std::min(height,width);
                #if 1
                int minSize = 8;
                for(int i=0;i<16;i++) {
                    if((1<<i)<=window_size_ && (1<<(i+1)) > window_size_) {
                        minSize = 1<<i;
                        break;
                    }
                }
                while(minSize < window_size_) {
                    int new_size;
                    if((new_size = cv::getOptimalDFTSize(minSize+1)) <= window_size_) {
                        minSize = new_size;
                    }
                    else {
                        break;
                    }
                }
                BOOSTER_INFO("stacker") << "Optimal size " << minSize << " for window " << window_size_;
                window_size_ = minSize;
                #endif
            }
            else {
                window_size_ = roi_size;
            }
            if(roi_x == -1 && roi_y == -1) {
                dx_ = (width  - window_size_)/2;
                dy_ = (height - window_size_)/2;
            }
            else {
                dx_ = std::max(0,roi_x - window_size_/2);
                dy_ = std::max(0,roi_y - window_size_/2);
                dx_ = std::min(width-window_size_,dx_);
                dy_ = std::min(height-window_size_,dy_);
            }
            
            sum_ = cv::Mat(height*subpixel_factor_,width*subpixel_factor_,cv_type_);
            sum_.setTo(0);
            make_fft_blur();
        }

        void set_rollback_on_pause(bool v)
        {
            rollback_on_pause_ = v;
        }

        void set_remove_satellites(bool v)
        {
            remove_satellites_ = v;
            if(remove_satellites_) {
                frame_max_ = sum_.clone();
            }
        }

        void make_fft_blur()
        {
            if(window_size_ == 0)
                return;
            fft_kern_ = cv::Mat(window_size_,window_size_,CV_32FC2);
            int rad = (window_size_/8);
            for(int r=0;r<window_size_;r++) {
                for(int c=0;c<window_size_;c++) {
                    int dy = fft_pos(r);
                    int dx = fft_pos(c);
                    std::complex<float> val = 1;
                    if(dx*dx+dy*dy > rad*rad) {
                        val = 0;
                    }
                    fft_kern_.at<std::complex<float> >(r,c) = val;
                }
            }
        }


        int total_count()
        {
            return total_count_;
        }
        int stacked_count()
        {
            return fully_stacked_count_;
        }

       
        cv::Mat get_raw_stacked_image()
        {
            BOOSTER_INFO("stacked") << "So far stacked " << fully_stacked_count_ << std::endl;
            if(remove_satellites_ && fully_stacked_count_ > 1) {
                stacked_res_ = (sum_ - frame_max_) * (1.0/ (fully_stacked_count_ - 1));
            }
            else {
                stacked_res_ = sum_ * (1.0/ fully_stacked_count_);
            }
            if(subpixel_factor_ != 1) {
                cv::Mat tmp;
                cv::resize(stacked_res_,tmp,cv::Size(0,0),1.0/subpixel_factor_,1.0/subpixel_factor_);
                stacked_res_ = tmp;
            }
            return stacked_res_;
        }
        cv::Rect get_valid_roi()
        {
            return fully_stacked_area_;
        }


        void handle_pause()
        {
            if(rollback_on_pause_ && fully_stacked_count_ > 1) {
                prev_sum_.copyTo(sum_);
                fully_stacked_count_ = prev_fully_stacked_count_;
                if(remove_satellites_) 
                    prev_frame_max_.copyTo(frame_max_);
            }
        }
        
        bool stack_image(cv::Mat frame,bool restart_position = false)
        {
            total_count_ ++;
            if(exp_multiplier_ != 1) {
                if(manual_exposure_counter_ == 0)
                    manual_frame_ = frame;
                else
                    manual_frame_ += frame;
                manual_exposure_counter_++;
                if(manual_exposure_counter_ < exp_multiplier_)
                    return true;
                manual_exposure_counter_ = 0;
                frame = manual_frame_ * (1.0f / exp_multiplier_);
            }
            if(window_size_ == 0) {
                add_image(frame,cv::Point2f(0,0));
                frames_ ++;
                return true;
            }
            bool added = true;
            if(frames_ == 0) {
                add_image(frame,cv::Point2f(0,0));
                fft_roi_ = calc_fft(frame,true);
                frames_ = 1;
                reset_step(cv::Point2f(0,0));
            }
            else {
                cv::Mat fft_frame = calc_fft(frame,false);
                cv::Point2f shift = get_dx_dy(fft_frame);
                BOOSTER_INFO("stacker") <<"Registration at "<< frames_ <<":" << shift << std::endl;
                if(restart_position) {
                    add_image(frame,shift);
                    reset_step(shift);
                    frames_ ++;
                }
                else {
                    if(check_step(shift)) {
                        add_image(frame,shift);
                        frames_ ++;
                    }
                    else {
                        added = false;
                    }
                }
            }
            return added;
        }
    private:

        void reset_step(cv::Point2f p)
        {
            current_position_ = p;
            step_sum_sq_ = 0;
            count_frames_ = 0;
            missed_frames_ = 0;
        }
        bool check_step(cv::Point2f p)
        {
            float dx = current_position_.x - p.x;
            float dy = current_position_.y - p.y;
            float step_sq_ = dx*dx + dy*dy;
            if(count_frames_ == 0) {
                current_position_ = p;
                step_sum_sq_ = dx*dx + dy*dy;
                count_frames_ = 1;
                missed_frames_ = 0;
                return true;
            }
            else {
                float step_avg_ = sqrt(step_sum_sq_ / count_frames_);
                constexpr int missed_in_a_row_limit = INT_MAX;
                constexpr float pixel_0_threshold = 3;
                float step = sqrt(step_sq_);
                float step_limit = std::max((2 + (float)sqrt(missed_frames_)) * step_avg_,pixel_0_threshold);
                char log_txt[256];
                snprintf(log_txt,sizeof(log_txt),
                        "Step size %5.2f from (%0.1f,%0.1f) to (%0.1f,%0.1f) limit =%5.1f avg_step=%5.1f\n",step,
                        current_position_.x,current_position_.y,
                        p.x,p.y,
                        step_limit,step_avg_);
                BOOSTER_INFO("stacker") << log_txt;

                if(missed_frames_ > missed_in_a_row_limit || step > step_limit) {
                    missed_frames_ ++;
                    return false;
                }
                else {
                    current_position_ = p;
                    count_frames_ ++;
                    step_sum_sq_+=step_sq_;
                    missed_frames_ = 0;
                    return true;
                }
            }
        }

        int fft_pos(int x)
        {
            if(x > window_size_ / 2)
                return x - window_size_;
            else
                return x;
        }

        float max3p(float v0,float v1,float v2)
        {
            // find max a*x^2 + b*x + c = 0 where x is 0,1,2 and ys = v0, v1, v2
            // than find extermum of this func
            float a = (v2 - v0) / 2 - (v1 - v0);
            float b = (v1 - v0)*2 - (v2 - v0) / 2;
            if(fabs(a) < 1e-10) // if v0=v1=v2 return center
                return 0.0f;
            float x = - b /(2*a);
            return x - 1.0f;
        }

        cv::Point2f fft_pos2d(int r,int c,cv::Mat &shift)
        {
            float vals[3][3];
            for(int pr = 0;pr < 3; pr++) {
                for(int pc = 0; pc < 3; pc ++) {
                    int n_r = (r + pr - 1 + shift.rows) % shift.rows;
                    int n_c = (c + pc - 1 + shift.cols) % shift.cols;
                    vals[pr][pc] = shift.at<float>(n_r,n_c);
                }
            }
            float dc = max3p(vals[1][0],vals[1][1],vals[1][2]);
            float dr = max3p(vals[0][1],vals[1][1],vals[2][1]);
            return cv::Point2f(c+dc,r+dr);
        }
        
        void calcPC(cv::Mat &A,cv::Mat &B,cv::Mat &spec)
        {
            float *a = (float *)(A.data);
            float *b = (float *)(B.data);
            spec.create(A.rows,A.cols,CV_32FC2); // complext
            float *s = (float *)(spec.data);
            int N = A.rows*A.cols;
            int i=0;
#ifdef USE_CV_SIMD            
            int limit=N/4*4;
            for(;i<limit;i+=4,a+=8,b+=8,s+=8) {
                cv::v_float32x4 a_re,a_im,b_re,b_im;
                v_load_deinterleave(a,a_re,a_im);
                v_load_deinterleave(b,b_re,b_im);

                // mul conj
                cv::v_float32x4 res_re = a_re*b_re + a_im*b_im;
                cv::v_float32x4 res_im = a_im*b_re - a_re*b_im;

                // abs
                cv::v_float32x4 res_abs = cv::v_sqrt(res_re*res_re + res_im*res_im);

                // div by abs
                res_abs = cv::v_max(cv::v_setall_f32(1e-38f),res_abs);
                res_re /= res_abs;
                res_im /= res_abs;

                cv::v_store_interleave(s,res_re,res_im);
            }
#endif                
            for(;i<N;i++) {
                float a_re = *a++;
                float a_im = *a++;
                float b_re = *b++;
                float b_im = *b++;
                b_im = -b_im; // conj
                auto ac=std::complex<float>(a_re,a_im);
                auto bc=std::complex<float>(b_re,b_im);
                auto res = ac*bc;
                float abs_val = std::max(1e-38f,std::abs(res));
                *s++ = res.real() / abs_val;
                *s++ = res.imag() / abs_val;
            }
        }

        cv::Point2f get_dx_dy(cv::Mat dft)
        {
            cv::Mat shift;
            cv::Mat dspec;
            
            calcPC(fft_roi_,dft,dspec);
            cv::idft(dspec,shift,cv::DFT_REAL_OUTPUT);
#ifdef DEBUG
            {
                double minv,maxv;
                cv::minMaxLoc(shift,&minv,&maxv);
                cv::Mat tmp;
                cv::Mat scaled = ((shift - minv)/((maxv-minv)*255));
                scaled.convertTo(tmp,CV_8UC1);
                static int n=1;
                cv::imwrite("/tmp/map_" + std::to_string(n++) + "_b.png",tmp);
            }
#endif        

            cv::Point pos;
            cv::minMaxLoc(shift,nullptr,nullptr,nullptr,&pos);
            if(enable_subpixel_registration)
                return fft_pos2d(fft_pos(pos.y),fft_pos(pos.x),shift);
            else
                return cv::Point2f(fft_pos(pos.x),fft_pos(pos.y));
        }

        cv::Mat calc_fft(cv::Mat frame,bool first_frame)
        {
            cv::Mat rgb[3],gray,dft;
            cv::Mat roi = cv::Mat(frame,cv::Rect(dx_,dy_,window_size_,window_size_));
            if(channels_ == 3) {
                cv::split(roi,rgb);
                rgb[1].convertTo(gray,CV_32FC1);
            }
            else {
                roi.copyTo(gray);
            }
            cv::dft(gray,dft,cv::DFT_COMPLEX_OUTPUT);
            if(first_frame) {
                cv::mulSpectrums(dft,fft_kern_,dft,0);
            }
#ifdef DEBUG
            {
                cv::mulSpectrums(dft,fft_kern_,dft,0);
                cv::idft(dft,gray,cv::DFT_REAL_OUTPUT);
                double minv,maxv;
                cv::minMaxLoc(gray,&minv,&maxv);
                gray = (gray - minv)/(maxv-minv)*255;
                cv::Mat tmp;
                gray.convertTo(tmp,CV_8UC1);
                static int n=1;
                cv::imwrite("/tmp/blurred_" + std::to_string(n++) + "_b.png",tmp);
            }
#endif        
            return dft;
        }

        void calc_stacked_area(cv::Point shift)
        {
            int dx = round(shift.x);
            int dy = round(shift.y);
            int width  = (sum_.cols - std::abs(dx));
            int height = (sum_.rows - std::abs(dy));
            cv::Rect src_rect = cv::Rect(std::max(dx,0),std::max(dy,0),width,height);
            fully_stacked_area_ = fully_stacked_area_ & src_rect;
        }


        void add_image_upscaled(cv::Mat img,cv::Point shift)
        {
            if(rollback_on_pause_ && fully_stacked_count_ >= 1) {
                sum_.copyTo(prev_sum_);
                prev_fully_stacked_count_ = fully_stacked_count_;
                if(remove_satellites_) 
                    frame_max_.copyTo(prev_frame_max_);
            }
            int dx = shift.x;
            int dy = shift.y;
            int width  = (sum_.cols - std::abs(dx));
            int height = (sum_.rows - std::abs(dy));
            cv::Rect src_rect = cv::Rect(std::max(dx,0),std::max(dy,0),width,height);
            cv::Rect img_rect = cv::Rect(std::max(-dx,0),std::max(-dy,0),width,height);
            cv::Mat(sum_,src_rect) += cv::Mat(img,img_rect);
            if(remove_satellites_) {
                cv::Mat max_roi = cv::Mat(frame_max_,src_rect);
                max_roi = cv::max(max_roi,cv::Mat(img,img_rect));
            }
        }

        void add_image(cv::Mat img,cv::Point2f shift)
        {
            calc_stacked_area(cv::Point(shift.x,shift.y));

            float dx = round(shift.x * subpixel_factor_);
            float dy = round(shift.y * subpixel_factor_);

            cv::Mat resized;
            if(subpixel_factor_ != 1) {
                cv::resize(img,resized,cv::Size(0,0),subpixel_factor_,subpixel_factor_);
            }
            else {
                resized = img;
            }

            add_image_upscaled(resized,cv::Point(dx,dy));

            fully_stacked_count_++;
        }

        int frames_;
        int total_count_ = 0;
        bool has_darks_;
        cv::Rect fully_stacked_area_;
        int fully_stacked_count_ = 0;
        bool remove_satellites_ = false;
        cv::Mat frame_max_;
        cv::Mat sum_;
        cv::Mat prev_sum_,prev_frame_max_;
        int prev_fully_stacked_count_ = 0;
        cv::Mat darks_;
        cv::Mat darks_gamma_corrected_;
        bool darks_corrected_ = false;
        cv::Mat fft_kern_;
        cv::Mat fft_roi_;
        cv::Point2f current_position_;

        cv::Mat stacked_res_;
        int count_frames_,missed_frames_;
        float step_sum_sq_;
        int dx_,dy_,window_size_;
        int manual_exposure_counter_ = 0;
        int exp_multiplier_;
        cv::Mat manual_frame_;

        int channels_;
        int cv_type_;

        bool rollback_on_pause_ = false;

    };



}
