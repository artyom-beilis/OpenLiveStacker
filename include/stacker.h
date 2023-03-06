#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <booster/log.h>
#include "common_data.h"
namespace ols {

    struct Stacker {
    public:

        bool enable_subpixel_registration = false;
        int  subpixel_factor_ = 1;

        float high_per_=99.99f;
        double gamma_limit_ = 4.0;
        double mean_target_ = 0.25;

        Stacker(int width,int height,int roi_x=-1,int roi_y=-1,int roi_size = -1,int exp_multiplier=1) : 
            frames_(0),
            exp_multiplier_(exp_multiplier)
        {
            if(!enable_subpixel_registration || subpixel_factor_ == 1) {
                subpixel_factor_ = 1;
                enable_subpixel_registration = false;
            }

            set_stretch(true,0,0,0.5f);
            fully_stacked_area_ = cv::Rect(0,0,width,height);
            fully_stacked_count_ = 0;
            if(roi_size == -1) {
                window_size_ = std::min(height,width);
                #if 0
                for(int i=0;i<16;i++) {
                    if((1<<i)<=window_size_ && (1<<(i+1)) > window_size_) {
                        window_size_ = 1<<i;
                        break;
                    }
                }
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
            
            sum_ = cv::Mat(height*subpixel_factor_,width*subpixel_factor_,CV_32FC3);
            sum_.setTo(0);
            make_fft_blur();
        }

        void set_stretch(bool auto_stretch,float low_index,float high_index,float stretch_index)
        {
            enable_stretch_ = auto_stretch;
            
            low_cut_  = std::min(1.0f,std::max(0.0f,low_index));
            high_cut_ = std::max(1.0f,std::min(16.0f,high_index));
            target_gamma_ = std::max(1.0f,std::min(8.0f,stretch_index));
        }

        void make_fft_blur()
        {
            if(window_size_ == 0)
                return;
            fft_kern_ = cv::Mat(window_size_,window_size_,CV_32FC2);
            int rad = (window_size_/4);
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
            cv::Mat res = sum_ * (1.0/ fully_stacked_count_);
            if(subpixel_factor_ != 1) {
                cv::Mat tmp;
                cv::resize(res,tmp,cv::Size(0,0),1.0/subpixel_factor_,1.0/subpixel_factor_);
                res = tmp;
            }
            return res;
        }

        std::pair<cv::Mat,StretchInfo> get_stacked_image()
        {
            cv::Mat tmp = get_raw_stacked_image();
            
            double scale[3];
            calc_wb(tmp(fully_stacked_area_),scale);
            tmp = tmp.mul(cv::Scalar(scale[0],scale[1],scale[2]));
            tmp = cv::max(0,cv::min(1,tmp));
            double gscale=1.0;
            double goffset = 0.0;
            double mean = 0.5;
            float gamma_correction = 1.0f;

            if(enable_stretch_) {
                stretch(tmp(fully_stacked_area_),gscale,goffset,mean);
                gamma_correction = cv::max(1.0,cv::min(gamma_limit_,log(mean)/log(mean_target_)));
                BOOSTER_INFO("stacker") << "Stretch mean " << mean << "-> gamma=" << gamma_correction;
            }
            else {
                gamma_correction = target_gamma_;
                gscale = high_cut_;
                goffset = -low_cut_ / gscale;
            }
            tmp += cv::Scalar::all(goffset);
            tmp = tmp.mul(cv::Scalar::all(gscale));
            tmp = cv::max(0,cv::min(1,tmp));

            double linear_point = 0.008;
            double linear_gain = std::pow(linear_point,1/gamma_correction) / linear_point;
            if(gamma_correction > 1.0) {
                cv::Mat linear_part = tmp.mul(cv::Scalar::all(linear_gain));
                cv::pow(tmp,1/gamma_correction,tmp);
                tmp = cv::min(linear_part,tmp);
            }
            StretchInfo stretch;
            stretch.gamma = gamma_correction;
            stretch.gain = gscale;
            stretch.cut = - goffset * gscale;
            return std::make_pair(tmp,stretch);
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
                fft_roi_ = calc_fft(frame);
                frames_ = 1;
                reset_step(cv::Point2f(0,0));
            }
            else {
                cv::Mat fft_frame = calc_fft(frame);
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
        void stretch(cv::Mat img,double &scale,double &offset,double &mean)
        {
            cv::Mat tmp;
            static constexpr int hist_bins = 1<<10;
            img.convertTo(tmp,CV_16UC3,(hist_bins-1),0);
            int counters[hist_bins]={};
            int N=tmp.rows*tmp.cols;
            uint16_t *p=reinterpret_cast<uint16_t*>(tmp.data);
            for(int i=0;i<N;i++) {
                unsigned R = *p++;
                unsigned G = *p++;
                unsigned B = *p++;
                unsigned Y = unsigned(0.3f * R + 0.6f * G + 0.1f * B);
                counters[Y]++;
            }
            int sum=N;
            int hp=-1;
            for(int i=hist_bins-1;i>=0;i--) {
                sum-=counters[i];
                if(sum*100.0f/N <= high_per_) {
                    hp = i;
                    break;
                }
            }
            int end = hp / 2;
            int max_diff = 0;
            for(int i=0;i<end-1;i++) {
                int diff = counters[i+1] - counters[i];
                max_diff = std::max(diff,max_diff);
            }
            int lp = 0;
            for(int i=0;i<end-1;i++) {
                lp = i;
                int diff = counters[i+1] - counters[i];
                if(diff * 10 >= max_diff)
                    break;
            }
            scale = (hist_bins-1.0)/hp;
            offset = -lp/(hist_bins - 1.0);
            
            mean = 0;
            int total = 0;
            for(int i=0;i<lp;i++) {
                total += counters[i];
            }
            for(int i=lp;i<=hp;i++) {
                mean += (i - lp) / (hist_bins - 1.0) * counters[i] * scale;
                total += counters[i];
            }
            for(int i=hp+1;i<hist_bins;i++) {
                mean += counters[i] * scale;
                total += counters[i];
            }
            mean = mean / total;
            BOOSTER_INFO("stacker") << "Scale " << scale << " offset=" << (-offset) << " relative cut =" << (-offset*scale) ;
        }

        void calc_wb(cv::Mat img,double scale[3])
        {
            cv::Mat tmp=img.clone();
            float *p = (float*)tmp.data;
            int N = tmp.rows*tmp.cols;
            double maxV,minV;
            cv::minMaxLoc(tmp,&minV,&maxV);
#if 0
            static constexpr int bins_count = 1<<12;
            float a = bins_count / (maxV - minV);
            float b = -minV;
            int bins[bins_count][3]={};
            for(int i=0;i<N;i++) {
                for(int c=0;c<3;c++) {
                    float V = *p++;
                    int binNo = std::max(0.0f,std::min(float(bins_count-1),(V+b)*a));
                    bins[binNo][c] ++;
                }
            }
            for(int c=0;c<3;c++) {
                int s=0;
                int i=0;
                for(i=0;i<bins_count;i++) {
                    if(s*2 >= N)
                        break;
                    s+= bins[i][c];
                }
                printf("Median of %d: %d\n",c,i);
                scale[c] = i;
            }
#elif 0
            scale[0]=scale[1]=scale[2] = 1;
#else
            scale[0]=scale[1]=scale[2] = 0;
            for(int i=0;i<N;i++) {
                for(int c=0;c<3;c++) {
                    scale[c] += *p++;
                }
            }
#endif                        
            // normalize once again
            double smax = std::max(scale[0],std::max(scale[1],scale[2]));
            double smin = std::min(scale[0],std::min(scale[1],scale[2]));
            if(smin > 0) {
                for(int i=0;i<3;i++)
                    scale[i] = smax / scale[i];
            }
            else {
                for(int i=0;i<3;i++)
                    scale[i] = 1.0;
            }
            printf("WB [%f,%f,%f]\n",scale[0],scale[1],scale[2]);
            for(int i=0;i<3;i++)
                scale[i] /= maxV;
        }


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
        cv::Point2f get_dx_dy(cv::Mat dft)
        {
            cv::Mat res,shift;
            cv::mulSpectrums(fft_roi_,dft,res,0,true);
            cv::Mat dspec;
#if 0 //CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 5
            cv::divSpectrums(res,cv::abs(res),dspec,0);
#else
            cv::Mat absval = cv::max(cv::Scalar::all(1e-38),cv::abs(res));
            cv::divide(res,absval,dspec);
#endif            
            cv::idft(dspec,shift,cv::DFT_REAL_OUTPUT);
            cv::Point pos;
            cv::minMaxLoc(shift,nullptr,nullptr,nullptr,&pos);
            if(enable_subpixel_registration)
                return fft_pos2d(fft_pos(pos.y),fft_pos(pos.x),shift);
            else
                return cv::Point2f(fft_pos(pos.x),fft_pos(pos.y));
        }

        cv::Mat calc_fft(cv::Mat frame)
        {
            cv::Mat rgb[3],gray,dft;
            cv::Mat roi = cv::Mat(frame,cv::Rect(dx_,dy_,window_size_,window_size_));
            cv::split(roi,rgb);
            rgb[1].convertTo(gray,CV_32FC1);
            cv::dft(gray,dft,cv::DFT_COMPLEX_OUTPUT);
            cv::mulSpectrums(dft,fft_kern_,dft,0);
#ifdef DEBUG
            {
                cv::idft(dft,gray,cv::DFT_REAL_OUTPUT);
                double minv,maxv;
                cv::minMaxLoc(gray,&minv,&maxv);
                gray = (gray - minv)/(maxv-minv)*255;
                cv::Mat tmp;
                gray.convertTo(tmp,CV_8UC1);
                static int n=1;
                imwritergb(std::to_string(n++) + "_b.png",tmp);
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


#if 0
        void add_image_weighted(cv::Mat img,cv::Point shift,float weight)
        {
            if(weight == 0)
                return;
            int dx = shift.x;
            int dy = shift.y;
            int width  = (sum_.cols - std::abs(dx));
            int height = (sum_.rows - std::abs(dy));
            cv::Rect src_rect = cv::Rect(std::max(dx,0),std::max(dy,0),width,height);
            cv::Rect img_rect = cv::Rect(std::max(-dx,0),std::max(-dy,0),width,height);

            if(weight == 1)
                cv::Mat(sum_,src_rect) += cv::Mat(img,img_rect);
            else
                cv::Mat(sum_,src_rect) += cv::Mat(img,img_rect).mul(cv::Scalar::all(weight));
        }

        void add_image(cv::Mat img,cv::Point2f shift)
        {
            calc_stacked_area(cv::Point(shift.x,shift.y));

            float dx = shift.x;
            float dy = shift.y;
            int x0 = floor(dx);
            int x1 = x0 + 1;
            float xw1 = (dx - x0);
            float xw0 = 1.0f - xw1;

            int y0 = floor(dy);
            int y1 = y0 + 1;
            float yw1 = (dy - y0);
            float yw0 = 1.0f - yw1;
            
            std::cout << "Adding frames x=" << x0 << "-" << x1 << " w" << xw0 << "-" <<xw1 << std::endl; 
            std::cout << "Adding frames y=" << y0 << "-" << y1 << " w" << yw0 << "-" <<yw1 << std::endl; 

            add_image_weighted(img,cv::Point(x0,y0),xw0*yw0);
            add_image_weighted(img,cv::Point(x1,y0),xw1*yw0);
            add_image_weighted(img,cv::Point(x0,y1),xw0*yw1);
            add_image_weighted(img,cv::Point(x1,y1),xw1*yw1);

            fully_stacked_count_++;
        }
#else    
        void add_image_upscaled(cv::Mat img,cv::Point shift)
        {
            int dx = shift.x;
            int dy = shift.y;
            int width  = (sum_.cols - std::abs(dx));
            int height = (sum_.rows - std::abs(dy));
            cv::Rect src_rect = cv::Rect(std::max(dx,0),std::max(dy,0),width,height);
            cv::Rect img_rect = cv::Rect(std::max(-dx,0),std::max(-dy,0),width,height);
            cv::Mat(sum_,src_rect) += cv::Mat(img,img_rect);
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
#endif        

        int frames_;
        int total_count_ = 0;
        bool has_darks_;
        cv::Rect fully_stacked_area_;
        int fully_stacked_count_ = 0;
        cv::Mat sum_;
        cv::Mat darks_;
        cv::Mat darks_gamma_corrected_;
        bool darks_corrected_ = false;
        cv::Mat fft_kern_;
        cv::Mat fft_roi_;
        cv::Point2f current_position_;
        int count_frames_,missed_frames_;
        float step_sum_sq_;
        int dx_,dy_,window_size_;
        int manual_exposure_counter_ = 0;
        int exp_multiplier_;
        cv::Mat manual_frame_;
        bool enable_stretch_ = true;
        float low_cut_ = 0.0f;
        float high_cut_ = 1.0f;
        float target_gamma_ = 2.2f;

        //float low_per_= 0.05;
        //float high_per_=99.999f;
    };



}
