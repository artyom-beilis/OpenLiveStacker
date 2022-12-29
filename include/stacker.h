#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <booster/log.h>
namespace ols {

    struct Stacker {
    public:

        Stacker(int width,int height,int roi_x=-1,int roi_y=-1,int roi_size = -1,int exp_multiplier=1) : 
            frames_(0),
            exp_multiplier_(exp_multiplier)
        {
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
            
            sum_ = cv::Mat(height,width,CV_32FC3);
            sum_.setTo(0);
            count_ = cv::Mat(height,width,CV_32FC3);
            count_.setTo(0);
            make_fft_blur();
        }

        void set_target_gamma(float g)
        {
            if(g == -1.0f) {
                enable_stretch_ = true;
                tgt_gamma_ = 1.0f;
            }
            else {
                tgt_gamma_ = g;
                enable_stretch_ = false;
            }
        }

        void make_fft_blur()
        {
            if(window_size_ == 0)
                return;
            fft_kern_ = cv::Mat(window_size_,window_size_,CV_32FC2);
            int rad = (window_size_/16);
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

        int stacked_count()
        {
            return fully_stacked_count_;
        }
       
        cv::Mat get_raw_stacked_image()
        {
            BOOSTER_INFO("stacked") << "So far stacked " << fully_stacked_count_ << std::endl;
            return sum_ * (1.0/ fully_stacked_count_);
        }

        cv::Mat get_stacked_image()
        {
            cv::Mat tmp = get_raw_stacked_image();
            if(enable_stretch_) {
                double scale[3];
                calc_wb(tmp(fully_stacked_area_),scale);
                tmp = tmp.mul(cv::Scalar(scale[0],scale[1],scale[2]));
                tmp = cv::max(0,cv::min(1,tmp));
                double gscale=1.0;
                double goffset = 0.0;
                double mean = 0.5;
                stretch(tmp(fully_stacked_area_),gscale,goffset,mean);
                tmp += cv::Scalar(goffset,goffset,goffset);
                tmp = tmp.mul(cv::Scalar(gscale,gscale,gscale));
                tmp = cv::max(0,cv::min(1,tmp));
                float g=cv::max(1.0,cv::min(2.2,log(mean)/log(0.25)));
                printf("Mean %f gamma=%f\n",mean,g);
                cv::pow(tmp,1/g,tmp);
            }
            else {
                double max_v,min_v;
                cv::minMaxLoc(tmp,&min_v,&max_v);
                if(min_v < 0)
                    min_v = 0;
                tmp = cv::max(0,(tmp - float(min_v))*float(1.0/(max_v-min_v)));
                if(tgt_gamma_!=1.0f) {
                    cv::pow(tmp,1/tgt_gamma_,tmp);
                }
            }
            return tmp;
        }
        void get_stacked(unsigned char *rgb_img)
        {
            if(frames_ == 0)
                memset(rgb_img,0,sum_.rows*sum_.cols*3);
            else {
                cv::Mat tgt(sum_.rows,sum_.cols,CV_8UC3,rgb_img);
                cv::Mat tmp = get_stacked_image();
                tmp.convertTo(tgt,CV_8UC3,255,0);
            }
        }
        
        bool stack_image(cv::Mat frame,bool restart_position = false)
        {
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
                add_image(frame,cv::Point(0,0));
                frames_ ++;
                return true;
            }
            bool added = true;
            if(frames_ == 0) {
                add_image(frame,cv::Point(0,0));
                fft_roi_ = calc_fft(frame);
                frames_ = 1;
                reset_step(cv::Point(0,0));
            }
            else {
                cv::Mat fft_frame = calc_fft(frame);
                cv::Point shift = get_dx_dy(fft_frame);
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
            img.convertTo(tmp,CV_8UC3,255,0);
            int counters[256]={};
            int N=tmp.rows*tmp.cols;
            unsigned char *p=tmp.data;
            for(int i=0;i<N;i++) {
                unsigned R = *p++;
                unsigned G = *p++;
                unsigned B = *p++;
                unsigned char Y = unsigned(0.3f * R + 0.6f * G + 0.1f * B);
                counters[Y]++;
            }
            int sum = 0;
            int lp=0;
            for(int i=0;i<255;i++) {
                sum+=counters[i];
                if(sum*100.0f/N >= low_per_) {
                    lp = i;
                    break;
                }
            }
            sum=N;
            int hp=-1;
            for(int i=255;i>=0;i--) {
                sum-=counters[i];
                if(sum*100.0f/N <= high_per_) {
                    hp = i;
                    break;
                }
            }
            scale = 255.0/(hp - lp);
            offset = -lp/255.0;
            mean = 0;
            int total = 0;
            for(int i=0;i<lp;i++) {
                total += counters[i];
            }
            for(int i=lp;i<=hp;i++) {
                mean += (i - lp) / 255.0 * counters[i] * scale;
                total += counters[i];
            }
            for(int i=hp+1;i<255;i++) {
                mean += counters[i] * scale;
                total += counters[i];
            }
            mean = mean / total;
            printf("stretch [%d,%d] scale=%f offset=%f mean=%f\n",lp,hp,scale,offset,mean);
        }

        void calc_wb(cv::Mat img,double scale[3])
        {
            cv::Mat tmp=img.clone();
            float *p = (float*)tmp.data;
            int N = tmp.rows*tmp.cols;
            double maxV,minV;
            cv::minMaxLoc(tmp,&minV,&maxV);
            static constexpr int bins_count = 50;
            static constexpr int gap = 10;
            float top_cut = (maxV - minV) / bins_count * (bins_count-1) + minV;
            float bins[bins_count][3]={};
            int binsN[bins_count]={};
            float a = 10 / (maxV - minV);
            float b = -minV;
            for(int i=0;i<N;i++) {
                float R = *p++;
                float G = *p++;
                float B = *p++;
                // no saturated
                if(R >= top_cut || G >= top_cut || B >= top_cut)
                    continue;
                float Y = (0.3f * R + 0.6f * G + 0.1f * B);
                int binNo = std::max(0,std::min(bins_count-1,int((Y - b) * a)));
                bins[binNo][0] += R;
                bins[binNo][1] += G;
                bins[binNo][2] += B;
                binsN[binNo] ++;
            }
            int max_index = 0;
            int max_cnt = 0;
            for(int i=1;i<bins_count-1;i++) {
                if(binsN[i] >= max_cnt) {
                    max_cnt = binsN[i];
                    max_index = i;
                }
            }
            int indx_start = std::max(1,max_index-gap);
            int indx_end = std::min(bins_count - 2,max_index+gap);
            scale[0] = scale[1] = scale[2] = 0;
            int cnt = 0;
            for(int i=indx_start;i<=indx_end;i++) {
                if(binsN[i] == 0)
                    continue;
                float R = bins[i][0];
                float G = bins[i][1];
                float B = bins[i][2];
                float maxRGB_norm = std::max(R,std::max(G,B));
                scale[0] += maxRGB_norm / R * binsN[i];
                scale[1] += maxRGB_norm / G * binsN[i];
                scale[2] += maxRGB_norm / B * binsN[i];
                cnt += binsN[i];
            }
            for(int i=0;i<3;i++)
                scale[i] /= cnt;
            // normalize once again
            double smin = std::min(scale[0],std::min(scale[1],scale[2]));
            if(smin > 0) {
                for(int i=0;i<3;i++)
                    scale[i] /= smin;
            }
            else {
                for(int i=0;i<3;i++)
                    scale[i] = 1.0;
            }
            printf("WB [%f,%f,%f]\n",scale[0],scale[1],scale[2]);
            for(int i=0;i<3;i++)
                scale[i] /= maxV;
        }
                

        void reset_step(cv::Point p)
        {
            current_position_ = p;
            step_sum_sq_ = 0;
            count_frames_ = 0;
            missed_frames_ = 0;
        }
        bool check_step(cv::Point p)
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
                        "Step size %5.2f from (%d,%d) to (%d,%d) limit =%5.1f avg_step=%5.1f\n",step,
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
        cv::Point get_dx_dy(cv::Mat dft)
        {
            cv::Mat res,shift;
            cv::mulSpectrums(fft_roi_,dft,res,0,true);
            cv::Mat dspec;
#if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 5
            cv::divSpectrums(res,cv::abs(res),dspec,0);
#else
            dspec = res / cv::abs(res);
#endif            
            cv::idft(dspec,shift,cv::DFT_REAL_OUTPUT);
            cv::Point pos;
#ifdef DEBUG
            double minv,maxv;
            cv::minMaxLoc(shift,&minv,&maxv,nullptr,&pos);
            static int n=1;
            cv::Mat a,b;
            a=255*(shift-minv)/(maxv-minv);
            a.convertTo(b,CV_8UC1);
            imwritergb(std::to_string(n++) + "_shift.png",b);
#else
            cv::minMaxLoc(shift,nullptr,nullptr,nullptr,&pos);
            #if 0
                std::vector<float> vec(shift.cols);
                for(int r=0;r<shift.rows;r++) {
                    for(int c=0;c<shift.cols;c++) {
                        vec[r]+=shift.at<float>(r,c);
                    }
                }
                int maxp = 0;
                float maxv = vec[0];
                for(unsigned i=1;i<vec.size();i++) {
                    if(vec[i] > maxv) {
                        maxv = vec[i];
                        maxp=i;
                    }
                }
                pos.y = maxp;
                pos.x = 0;
                printf("Shift = %d\n",fft_pos(maxp));
            #endif
#endif        
            return cv::Point(fft_pos(pos.x),fft_pos(pos.y));
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

        void add_image(cv::Mat img,cv::Point shift)
        {
            int dx = shift.x;
            int dy = shift.y;
            int width  = (sum_.cols - std::abs(dx));
            int height = (sum_.rows - std::abs(dy));
            cv::Rect src_rect = cv::Rect(std::max(dx,0),std::max(dy,0),width,height);
            cv::Rect img_rect = cv::Rect(std::max(-dx,0),std::max(-dy,0),width,height);
            cv::Mat(sum_,src_rect) += cv::Mat(img,img_rect);
            cv::Mat(count_,src_rect) += cv::Scalar(1,1,1);
            fully_stacked_area_ = fully_stacked_area_ & src_rect;
            fully_stacked_count_++;
        }
        int frames_;
        bool has_darks_;
        cv::Rect fully_stacked_area_;
        int fully_stacked_count_ = 0;
        cv::Mat sum_;
        cv::Mat darks_;
        cv::Mat darks_gamma_corrected_;
        bool darks_corrected_ = false;
        cv::Mat count_;
        cv::Mat fft_kern_;
        cv::Mat fft_roi_;
        cv::Point current_position_;
        int count_frames_,missed_frames_;
        float step_sum_sq_;
        int dx_,dy_,window_size_;
        int manual_exposure_counter_ = 0;
        int exp_multiplier_;
        cv::Mat manual_frame_;
        float tgt_gamma_ = 1.0f;
        bool enable_stretch_ = true;
        //float low_per_= 0.5f;
        //float high_per_=99.999f;
        float low_per_= 0.05;
        float high_per_=99.999f;
    };



}
