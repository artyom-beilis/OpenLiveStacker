#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/hal/intrin.hpp>
#include <booster/log.h>
#include "common_data.h"

#include "simd_utils.h"

//#define DEBUG_SCORES            
//#define DEBUG
#if defined(DEBUG) || defined(DEBUG_SCORES)
#include <opencv2/imgcodecs.hpp>
#endif


namespace ols {

    class StackerBase {
    public:
        StackerBase(int width,int height,int channels) 
        {
            width_ = width;
            height_ = height;
            channels_ = channels;
            cv_type_ = channels == 1 ? CV_32FC1 : CV_32FC3;
            window_size_ = std::min(height,width);
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
            dx_ = (width  - window_size_)/2;
            dy_ = (height - window_size_)/2;
            sum_ = cv::Mat(height,width,cv_type_);
            sum_.setTo(0);
            make_fft_blur();
        }

        virtual ~StackerBase() {}

        virtual void set_filters(bool throw_frames,int frames_to_judge,int dynamic_delay,
                         float sharp_percetile,float reg_percentile,float brightness_sigma) = 0;
        virtual void set_rollback_on_pause(bool v) = 0;
        virtual void set_remove_satellites(bool v) = 0;
        virtual int total_count() = 0;
        virtual void handle_pause() = 0;
        virtual int stacked_count() = 0;
        virtual cv::Rect fully_stacked_area() = 0;
        virtual cv::Mat get_raw_stacked_image() = 0;
        virtual bool stack_image(cv::Mat frame,double timestamp,bool restart_position = false) = 0;

    protected:
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

        cv::Point2f get_dx_dy(cv::Mat dft,double &max_score)
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
            cv::minMaxLoc(shift,nullptr,&max_score,nullptr,&pos);
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
                mask_fft(dft);
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

        void mask_fft(cv::Mat dft)
        {
            cv::mulSpectrums(dft,fft_kern_,dft,0);
        }

        int fft_pos(int x)
        {
            if(x > window_size_ / 2)
                return x - window_size_;
            else
                return x;
        }
        static double tdiff(std::chrono::time_point<std::chrono::high_resolution_clock> const &a,
                            std::chrono::time_point<std::chrono::high_resolution_clock> const &b)
        {
            double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(b-a).count();
            return time * 1000;
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

        cv::Mat sum_;
        cv::Mat fft_kern_;
        cv::Mat fft_roi_;
        int dx_,dy_,window_size_;
        int channels_;
        int width_,height_;
        int cv_type_;

        

    };

    struct Stacker : public StackerBase {
    public:

        Stacker(int width,int height,int channels) : 
            StackerBase(width,height,channels),
            frames_(0),
            exp_multiplier_(1)
        {
            quality_roi_ = cv::Rect(width/4,height/4,width/2,height/2); 

            fully_stacked_area_ = cv::Rect(0,0,width,height);
            fully_stacked_count_ = 0;
        }

        void set_filters(bool throw_frames,int frames_to_judge,int /*dynamic_delay*/,
                         float sharp_percetile,float reg_percentile,float brightness_sigma)
        {
            throw_first_frames_ = throw_frames;
            min_size_to_judge_ = std::max(3,frames_to_judge);
            min_size_to_judge_reg_score_ = min_size_to_judge_ - 1;
            replace_1st_fame_at_ = min_size_to_judge_;
            sharp_percentile_ = std::max(0.1f,std::min(1.0f,sharp_percetile / 100.0f));
            reg_percentile_ = std::max(0.1f,std::min(1.0f,reg_percentile / 100.0f));
            brightness_sigma_ = brightness_sigma <= 0 ? -1.0f : std::max(0.5f,std::min(5.0f,brightness_sigma));
            if(sharp_percentile_ < 1.0f && throw_first_frames_ && reg_percentile_ < 1.0f) {
                min_size_to_judge_reg_score_ = std::max(1,min_size_to_judge_ / 2 - 1);
                replace_1st_fame_at_ /= 2;
            }

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



        int total_count()
        {
            return total_count_;
        }
        int stacked_count()
        {
            return fully_stacked_count_;
        }

        cv::Rect fully_stacked_area()
        {
            return fully_stacked_area_;
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
            return stacked_res_;
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

        bool check_reg_score(double score)
        {
            if(reg_percentile_ >= 1.0f) {
                return true;
            }
            bool res = add_score_and_check(reg_scores_,min_size_to_judge_reg_score_,score,reg_percentile_,"reg");
            if(!res) {
                BOOSTER_INFO("stacker") << "Registration score isn't high enough, skipping";
            }
            return res;
        }
        bool check_avg_brightness(cv::Mat frame,cv::Point2f shift,double avg)
        {
            if(brightness_sigma_ < 0)
                return true;

            if(avg == -1) // reuse
                avg = calc_avg_brightness(frame,shift);
            if(avg < 0)
                return false;
            avg_sum_ += avg;
            avg_sum2_ += avg*avg;
            avg_count_ ++;

            if(avg_count_ < min_size_to_judge_)
                return true;

            double expected_avg = avg_sum_ / avg_count_;
            double std_dev = std::sqrt(avg_sum2_ / avg_count_ -  expected_avg * expected_avg);
            double range_low  = expected_avg - std_dev * brightness_sigma_;
            double range_high = expected_avg + std_dev * brightness_sigma_;
            bool decision =  range_low <= avg && avg <= range_high;
            if(!decision) {
                BOOSTER_INFO("stacker") << "Skipping avg brightness ins't withing limits: " << avg << ", expected " << expected_avg << " +- " << std_dev << "*"<<brightness_sigma_ << "=" << (std_dev * brightness_sigma_) << " [ " << range_low <<  "," << range_high << "]";
            }
            return decision;
        }
        bool check_sharp_score_and_avg_brightness(cv::Mat frame,cv::Point2f shift,double &score,double &brightness)
        {
            score = 0;
            brightness = -1;
            if(sharp_percentile_ >= 1.0f)
                return true;
            auto r = calc_sharpness_score_avg_brightness(frame,shift);
            score = r.first;
            brightness = r.second;
            if(score < 0) {
                BOOSTER_INFO("stacker") << "Shift too big to esimate sharpness quality";
                return false;
            }
            bool res = add_score_and_check(sharp_scores_,min_size_to_judge_,score,sharp_percentile_,"sharp");
            if(!res) {
                BOOSTER_INFO("stacker") << "Image ins't sharp enought, skipping";
            }
            return res;
        }

        bool add_score_and_check(std::vector<double> &v,int min_size,double score,double per,char const *
#ifdef DEBUG_SCORES            
            type
#endif            
        )
        {
            insert_sorted(v,score);
            bool res = true;
            double th=0;
            if(int(v.size()) >= min_size) {
                th = v[(1.0f - per) * v.size()];
                res = score >= th;
            }
#ifdef DEBUG_SCORES            
            std::ostringstream ss;
            ss << type << ": ";
            for(auto val:v)
                ss << val << ",";
            ss<<"  New score " << score << " th="<< th << " decision=" << (res ?  "ok" : "fail") << " for per " << (per * 100);
            BOOSTER_INFO("stacker") << ss.str();
#endif
            return res;
        }
        void insert_sorted(std::vector<double> &v,double value)
        {
            v.insert(std::upper_bound(v.begin(),v.end(),value),value);
        }

        double calc_avg_brightness(cv::Mat frame,cv::Point2f shift)
        {
            int r0 = quality_roi_.y - shift.y;
            int c0 = quality_roi_.x - shift.x;
            int r1 = r0 + quality_roi_.height;
            int c1 = c0 + quality_roi_.width;
            if(r0 < 0 || c0 < 0 || r1 > frame.rows || c1 > frame.cols) {
                return -1;
            }
            int step = frame.cols * frame.channels();
            float *p = reinterpret_cast<float*>(frame.data);
            double avg = 0;
#ifdef DEBUG_SCORES            
            cv::Mat tmp(quality_roi_.height,quality_roi_.width,frame.type());
            tmp.setTo(0);
#endif            
            for(int r=r0;r<r1-2;r++) {
                float *line0   = p + r*step + c0 * frame.channels();
                int N= (quality_roi_.width - 2) * frame.channels();
                int limit = N / 4 * 4;
                int i=0;
                float avg_sum = 0;
                #ifdef USE_CV_SIMD
                cv::v_float32x4 vavg_sum = cv::v_setzero_f32();
                for(;i<limit;i+=4,line0+=4) {
                    cv::v_float32x4 v0 = cv::v_load(line0);
                    vavg_sum += v0;
#ifdef DEBUG_SCORES
                    cv::v_store((float*)(tmp.data) + quality_roi_.width*frame.channels() * (r-r0) + i,v0);
#endif                        
                }
                avg_sum  += cv::v_reduce_sum(vavg_sum);
                #endif
                for(;i<limit;i++) {
                    float v0 = *line0++;
                    avg_sum += v0;
#ifdef DEBUG_SCORES
                    *((float*)(tmp.data) + quality_roi_.width*frame.channels() * (r-r0) + i) = v0;
#endif                        
                }
                avg  += avg_sum;
            }
#ifdef DEBUG_SCORES            
            cv::Mat res;
            tmp.convertTo(res,frame.channels() == 3 ? CV_8UC1 : CV_8UC3,255.0);
            cv::imwrite("/tmp/sample_" + std::to_string(avg_count_) + ".png",res);
#endif            
            avg /= (quality_roi_.width-2)*(quality_roi_.height-2);
            return avg;
        }

        std::pair<double,double> calc_sharpness_score_avg_brightness(cv::Mat frame,cv::Point2f shift)
        {
            int r0 = quality_roi_.y - shift.y;
            int c0 = quality_roi_.x - shift.x;
            int r1 = r0 + quality_roi_.height;
            int c1 = c0 + quality_roi_.width;
            if(r0 < 0 || c0 < 0 || r1 > frame.rows || c1 > frame.cols) {
                return std::make_pair(-1.0,-1.0);
            }
            int step = frame.cols * frame.channels();
            int diff_step = 2 * frame.channels();
            float *p = reinterpret_cast<float*>(frame.data);
            double diff = 0;
            double avg = 0;
#ifdef DEBUG_SCORES            
            cv::Mat tmp(quality_roi_.height,quality_roi_.width,frame.type());
            tmp.setTo(0);
#endif            
            for(int r=r0;r<r1-2;r++) {
                float *line0   = p + r*step + c0 * frame.channels();
                float *line_dy = line0 + 2*step;
                float *line_dx = line0 + diff_step;
                int N= (quality_roi_.width - 2) * frame.channels();
                int limit = N / 4 * 4;
                int i=0;
                float diff_sum = 0;
                float avg_sum = 0;
                #ifdef USE_CV_SIMD
                cv::v_float32x4 vdiff_sum = cv::v_setzero_f32();
                cv::v_float32x4 vavg_sum = cv::v_setzero_f32();
                for(;i<limit;i+=4,line0+=4,line_dx+=4,line_dy+=4) {
                    cv::v_float32x4 v0 = cv::v_load(line0);
                    cv::v_float32x4 vx = cv::v_load(line_dx);
                    cv::v_float32x4 vy = cv::v_load(line_dy);
                    cv::v_float32x4 dx = v0 - vx;
                    cv::v_float32x4 dy = v0 - vy;
                    vavg_sum += v0;
                    cv::v_float32x4 ldiff = dx*dx + dy*dy;
                    vdiff_sum += ldiff;
#ifdef DEBUG_SCORES
                    cv::v_store((float*)(tmp.data) + quality_roi_.width*frame.channels() * (r-r0) + i,ldiff);
#endif                        
                }
                diff_sum += cv::v_reduce_sum(vdiff_sum);
                avg_sum  += cv::v_reduce_sum(vavg_sum);
                #endif
                for(;i<limit;i++) {
                    float v0 = *line0++;
                    float vx = *line_dx++;
                    float vy = *line_dy++;
                    float dx = v0 - vx;
                    float dy = v0 - vy;
                    avg_sum += v0;
                    float ldiff = dx*dx + dy*dy;
                    diff_sum += ldiff;
#ifdef DEBUG_SCORES
                    *((float*)(tmp.data) + quality_roi_.width*frame.channels() * (r-r0) + i) = ldiff;
#endif                        
                }
                diff += diff_sum;
                avg  += avg_sum;
            }
#ifdef DEBUG_SCORES            
            cv::Mat res;
            tmp.convertTo(res,frame.channels() == 3 ? CV_8UC1 : CV_8UC3,2500.0);
            cv::imwrite("/tmp/edges_" + std::to_string(sharp_scores_.size()) + ".png",res);
#endif            
            avg /= (quality_roi_.width-2)*(quality_roi_.height-2);
            return std::make_pair(diff,avg);
        }

        void reset_1st_frame() 
        {
            BOOSTER_INFO("stacker") << "Replacing 1st frame with best frame";
            cv::mulSpectrums(best_fft_,fft_kern_,fft_roi_,0);
            sum_.setTo(0);
            fully_stacked_count_ = prev_fully_stacked_count_ = 0;
            fully_stacked_area_ = cv::Rect(0,0,width_,height_);
            reg_scores_.clear(); 
            add_image(best_frame_,cv::Point2f(0,0));
            reset_step(cv::Point2f(0,0));
            quality_roi_.x -= int(best_shift_.x);
            quality_roi_.y -= int(best_shift_.y);
        }
        
        bool stack_image(cv::Mat frame,double /*timestamp*/,bool restart_position = false) override
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
            double sharp_score = 0;
            double brightness = 0;
            if(frames_ == 0) {
                add_image(frame,cv::Point2f(0,0));
                check_sharp_score_and_avg_brightness(frame,cv::Point2f(0,0),sharp_score,brightness);
                check_avg_brightness(frame,cv::Point2f(0,0),brightness);
                fft_roi_ = calc_fft(frame,true);
                best_score_ = sharp_score;
                best_frame_ = fft_roi_;
                best_shift_ = cv::Point2f(0,0);
                frames_ = 1;
                reset_step(cv::Point2f(0,0));
            }
            else {
                cv::Mat fft_frame = calc_fft(frame,false);
                double reg_score = 0;
                cv::Point2f shift = get_dx_dy(fft_frame,reg_score);
                BOOSTER_INFO("stacker") <<"Registration at "<< frames_ <<":" << shift << std::endl;
                bool to_add;
                if(restart_position) {
                    to_add = true;
                    reset_step(shift);
                }
                else {
                    to_add = check_step(shift);
                }
                
                if(to_add) {
                    auto start = std::chrono::high_resolution_clock::now();
                    bool sharp = check_sharp_score_and_avg_brightness(frame,shift,sharp_score,brightness);
                    bool avg   = check_avg_brightness(frame,shift,brightness);
                    bool reg   = check_reg_score(reg_score);
                    auto end = std::chrono::high_resolution_clock::now();
                    BOOSTER_INFO("stacker") << "Quality estimation took " << tdiff(start,end) << " ms";
                    to_add = sharp && reg && avg;
                }
                if(to_add) {
                    if(throw_first_frames_ && thrown_frames_ < min_size_to_judge_) {
                        added = false;
                        thrown_frames_ ++;
                        BOOSTER_INFO("stacker") << "Throwing " << thrown_frames_ << " out of " << min_size_to_judge_;
                        if(sharp_percentile_ < 1.0) {
                            if(sharp_score > best_score_) {
                                best_score_ = sharp_score;
                                best_frame_ = frame;
                                best_fft_ = fft_frame;
                                best_shift_ = shift;
                            }
                        }
                        if(thrown_frames_ == replace_1st_fame_at_) {
                            if(best_score_ != 0) {
                                reset_1st_frame();
                            }
                            best_frame_.release();
                            best_fft_.release();
                        }
                    }
                    else {
                        add_image(frame,shift);
                        frames_ ++;
                    }
                }
                else {
                    added = false;
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


        void calc_stacked_area(cv::Point shift)
        {
            int dx = round(shift.x);
            int dy = round(shift.y);
            int width  = (sum_.cols - std::abs(dx));
            int height = (sum_.rows - std::abs(dy));
            cv::Rect src_rect = cv::Rect(std::max(dx,0),std::max(dy,0),width,height);
            fully_stacked_area_ = fully_stacked_area_ & src_rect;
        }

        void add_image_pixels(cv::Mat img,cv::Point shift)
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

            float dx = round(shift.x);
            float dy = round(shift.y);

            add_image_pixels(img,cv::Point(dx,dy));

            fully_stacked_count_++;
        }

        int frames_;
        int total_count_ = 0;
        cv::Rect fully_stacked_area_;
        cv::Rect quality_roi_;
        int fully_stacked_count_ = 0;
        bool remove_satellites_ = false;
        cv::Mat frame_max_;
        cv::Mat prev_sum_,prev_frame_max_;
        int prev_fully_stacked_count_ = 0;
        cv::Point2f current_position_;

        cv::Mat stacked_res_;
        int count_frames_,missed_frames_;
        float step_sum_sq_;
        int manual_exposure_counter_ = 0;
        int exp_multiplier_;
        cv::Mat manual_frame_;
        
        // setup of filters

        float sharp_percentile_ = 1.0;
        float reg_percentile_ = 1.0;
        float brightness_sigma_ = -1.0f;

        std::vector<double> sharp_scores_;
        std::vector<double> reg_scores_;
        double avg_sum_ = 0, avg_sum2_ = 0;
        int avg_count_ = 0;
        bool throw_first_frames_ = false;
        int thrown_frames_ = 0;
        int min_size_to_judge_ = 10;
        int min_size_to_judge_reg_score_ = 10;
        int replace_1st_fame_at_ = 10;

        cv::Mat best_fft_;
        cv::Mat best_frame_;
        cv::Point2f best_shift_;
        double best_score_;

        bool rollback_on_pause_ = false;
    };

    class DynamicStacker : public StackerBase {
    public:
        DynamicStacker(int w,int h,int c) :StackerBase(w,h,c) 
        {
            int border_w = width_ / 8;
            int border_h = height_ / 8;
            move_limit_ = window_size_ / 8;
            fully_stacked_ = cv::Rect(border_w,border_h,width_-2*border_w,height_ - 2*border_h);
        }

        virtual void set_filters(bool, int ,int dynamic_delay,float,float,float) 
        {
            filter_frames_ = dynamic_delay;
            BOOSTER_INFO("stacker") << "Delay after reset " << dynamic_delay;
        }
        virtual void set_rollback_on_pause(bool) {}
        virtual void set_remove_satellites(bool) {}
        virtual void handle_pause() {}
        
        virtual int total_count() 
        {
            return total_;
        }
        virtual int stacked_count() 
        {
            return stacked_;
        }
        virtual cv::Rect fully_stacked_area() 
        {
            return fully_stacked_;
        }
        virtual cv::Mat get_raw_stacked_image() 
        {
            return sum_.clone();
        }


        void stack_first(cv::Mat frame,cv::Mat *fft = nullptr)
        {
            frame.copyTo(sum_);
            if(!fft) {
                fft_roi_ = calc_fft(frame,true); 
            }
            else {
                fft_roi_ = *fft;
                mask_fft(fft_roi_);
            }
            stacked_ = 1;
            offset_ = cv::Point2f(0,0);
            avg_dx_ = avg_dy_ = 0;
        }
        virtual bool stack_image(cv::Mat frame,double frame_timestamp,bool restart_position = false) override
        {
            total_ ++;
            if(restart_position || stacked_ == 0) {
                ignore_till_ts_ = frame_timestamp + filter_frames_;
            }

            
            if(stacked_ == 0 || restart_position || ignore_till_ts_ > frame_timestamp) {
                BOOSTER_INFO("stacker") << "Deley till next frame: " << (ignore_till_ts_ - frame_timestamp) << std::endl;
                stack_first(frame);
            }
            else {
                cv::Mat fft_frame = calc_fft(frame,false);
                double reg_score = 0;
                cv::Point2f shift = get_dx_dy(fft_frame,reg_score);
                float step_size = check_step(shift);
                if(step_size > 0) {
                    failed_count_ ++;
                    constexpr int max_fails = 5;
                    constexpr int max_fails_large = 2;
                    if(failed_count_ >= max_fails || (step_size > 3 && failed_count_ >= max_fails_large)) {
                        failed_count_ = 0;
                        ignore_till_ts_ = frame_timestamp + filter_frames_;
                        stack_first(frame,&fft_frame);
                        return true;
                    }
                    return false;
                }
                else {
                    failed_count_ = 0;
                    add_frame(frame,shift,fft_frame);
                }
            }
            return true;
        }
    private:
        float check_step(cv::Point2f shift)
        {
            if(std::abs(shift.x) > window_size_ / 8 || std::abs(shift.y) > window_size_ / 8) {
                return 1000;
            }
            float dx = offset_.x - shift.x;
            float dy = offset_.y - shift.y;
            
            char log_txt[256];
            snprintf(log_txt,sizeof(log_txt),
                    "Step size %.1f/%.1f from %.1f/%.1f -> %.1f/%.1f avg %.1f/%.1f",
                        dx,dy,
                        offset_.x,offset_.y,shift.x,shift.y,
                        avg_dx_,avg_dy_);

            float avg_diff_x = std::abs(avg_dx_ - dx);
            float avg_diff_y = std::abs(avg_dy_ - dy);
            float result = std::max(avg_diff_x,avg_diff_y) / 3.0f;
            if(result > 1.0f) {
                BOOSTER_INFO("stacker") << "Step is too big " << log_txt;
            }
            else {
                result = 0.0f;
            }
            BOOSTER_INFO("stacker") << "Step Ok " << log_txt;
            float w0 = 1.0f/(std::min(20,stacked_) + 1);
            float w1 = 1 - w0;
            avg_dx_ = dx * w0 + avg_dx_ * w1;
            avg_dy_ = dy * w0 + avg_dy_ * w1;
            
            return result;
        }
        void add_frame(cv::Mat frame,cv::Point2f shift,cv::Mat fft)
        {
            int dx = round(shift.x - offset_.x);
            int dy = round(shift.y - offset_.y);
            int width  = (width_ - std::abs(dx));
            int height = (height_ - std::abs(dy));
            cv::Rect src_rect = cv::Rect(std::max(dx,0),std::max(dy,0),width,height);
            cv::Rect img_rect = cv::Rect(std::max(-dx,0),std::max(-dy,0),width,height);

            int stacked_limit = move_limit_ / std::max(0.25f,std::max(avg_dx_,avg_dy_));
            float W0 = stacked_ >= stacked_limit ? 0.2f : (1.0f / stacked_);
            float W1 = 1 - W0;
            BOOSTER_INFO("stacker") << "Dynamic stacking frames limit=" << stacked_limit;
            frame = frame.mul(cv::Scalar::all(W0));
            cv::Mat(frame,img_rect) += cv::Mat(sum_,src_rect).mul(cv::Scalar::all(W1));

            stacked_ ++;
            sum_ = frame;
            offset_ = shift;
            int shift_limit = move_limit_ / 2;
            if(std::abs(offset_.x) > shift_limit || std::abs(offset_.y) > shift_limit) {
                BOOSTER_INFO("stacker") << "Resetting ancor frame  shift " << offset_ << " larger than " << shift_limit;
                offset_ = cv::Point2f(0,0);
                fft_roi_ = fft;
                mask_fft(fft_roi_);
            }
        }

        cv::Point2f velocity_ = cv::Point2f(0,0);
        cv::Point2f offset_ = cv::Point2f(0,0);
        int filter_frames_ = 0;
        double ignore_till_ts_ = 0;
        int stacked_ = 0;
        int total_ = 0;
        int failed_count_ = 0;
        int move_limit_;
        float avg_dy_,avg_dx_;
        cv::Rect fully_stacked_;
    };

} // namespace
