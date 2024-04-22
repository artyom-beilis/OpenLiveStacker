#include "data_items.h"
#include "rotation.h"
#include "stacker.h"
#include "post_processor.h"
#include "tiffmat.h"
#include "common_utils.h"
#include <booster/log.h>
#include "processors.h"
#include <cppcms/json.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/hal/intrin.hpp>
#include <fstream>
#include <iomanip>
#include <chrono>
#include "util.h"

#include "simd_utils.h"

namespace ols {
    void send_message(queue_pointer_type q,std::string const &id,std::string const &err)
    {
        if(q) {
            std::shared_ptr<ErrorNotificationData> p(new ErrorNotificationData());
            p->source = id;
            p->message = err;
            q->push(p);
        }
    }

    void send_message(queue_pointer_type q,std::string const &id,std::exception const &e)
    {
        send_message(q,id,e.what());
    }

    class PreProcessor {
    public:
        constexpr static int gamma_table_size = 128;
        PreProcessor(queue_pointer_type in,queue_pointer_type out,queue_pointer_type err) :
            in_(in),out_(out),err_(err)
        {
        }
        void run()
        {
            while(true) {
                auto data_ptr = in_->pop();
                auto stop_ptr = std::dynamic_pointer_cast<ShutDownData>(data_ptr);
                if(stop_ptr) {
                    out_->push(data_ptr);
                    break;
                }
                auto video_ptr = std::dynamic_pointer_cast<CameraFrame>(data_ptr);
                if(video_ptr) {
		            auto start = std::chrono::high_resolution_clock::now();
                    bool status = handle_video(video_ptr);
		            auto done = std::chrono::high_resolution_clock::now();
                    if(status)
                        out_->push(data_ptr);
                    double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(done-start).count();
                    BOOSTER_INFO("stacker") << "Preprocessing took " << (time*1000) << "ms";
                    continue;
                }
                auto config_ptr = std::dynamic_pointer_cast<StackerControl>(data_ptr);
                if(config_ptr) {
                    handle_config(config_ptr);
                }
                out_->push(data_ptr);
            }
        }
        void darks_and_flats(cv::Mat &frame)
        {
            float *p=(float*)frame.data;
            float *f=(float*)flats_.data;
            float *d=(float*)darks_.data;
            int N = frame.rows*frame.cols*channels_;
            int i=0;
#ifdef USE_CV_SIMD
            cv::v_float32x4 zero = cv::v_setall_f32(0.0f);
            cv::v_float32x4 one = cv::v_setall_f32(1.0f);
            int limit = N/4*4;
            for(;i<limit;i+=4,p+=4,d+=4,f+=4) {
                cv::v_float32x4 v=cv::v_max(zero,cv::v_load(p) - cv::v_load(d));
                v = cv::v_min(cv::v_load(f) * v,one);
                cv::v_store(p,v);
            }
#endif            
            for(;i<N;i++) {
                float v=std::max(0.0f,*p - *d);
                v*= *f;
                *p = std::min(v,1.0f);
                p++,d++,f++;
            }
        }

        void darks_only(cv::Mat &frame)
        {
            float *p=(float*)frame.data;
            float *d=(float*)darks_.data;
            int N = frame.rows*frame.cols*channels_;
            int i=0;
#ifdef USE_CV_SIMD 
            cv::v_float32x4 zero = cv::v_setzero_f32();
            int limit = N/4*4;
            for(;i<limit;i+=4,p+=4,d+=4) {
                auto v=cv::v_max(zero,cv::v_load(p) - cv::v_load(d));
                cv::v_store(p,v);
            }
#endif            
            for(;i<N;i++) {
                float v=std::max(0.0f,*p - *d);
                *p = v;
                p++,d++;
            }
        }

        void prepare_gamma()
        {
            if(gamma_table_current_gamma_ == -1.0f || gamma_table_current_gamma_ != gamma_) {
                prepare_power_curve(gamma_table_size,gamma_table_,gamma_);
                gamma_table_current_gamma_ = gamma_;
            }
        }

        void apply_gamma(cv::Mat &frame)
        {
            prepare_gamma();

            float *p = (float*)frame.data;
            int N = frame.rows*frame.cols*channels_;
            int i=0;
#ifdef USE_CV_SIMD

            int limit = N / 4 * 4;

            for(i=0;i<limit;i+=4,p+=4) {
                cv::v_float32x4 v = cv::v_load(p);
                curve_simd(v,gamma_table_size,gamma_table_);
                cv::v_store(p,v);
            }
#endif
            for(;i<N;i++,p++) {
                float v = *p;
                v = curve_one(v,gamma_table_size,gamma_table_);
                *p = v;
            }
        }

        bool handle_video(std::shared_ptr<CameraFrame> video)
        {
            if(video->frame.rows != height_ || video->frame.cols != width_ || video->frame.channels() != channels_ ) {
                BOOSTER_ERROR("stacker") << "Invalid mat frame size, expecting " << height_ << "x" << width_ << "x" << channels_ 
                        << " got " << video->frame.rows<< "x"<<video->frame.cols << "x" << video->frame.channels();
                return false;
            }
            video->frame.convertTo(video->processed_frame,cv_type_,(1.0/synthetic_exposure_mpl_)/video->frame_dr);
            if(calibration_)
                return true;
            if(synthetic_exposure_mpl_ > 1) {
                synthetic_exposure_count_ ++;
                if(synthetic_exposure_count_ == 1) {
                    synthetic_frame_ = video->processed_frame;
                    return false;
                }
                else if(synthetic_exposure_count_ < synthetic_exposure_mpl_) {
                    synthetic_frame_ += video->processed_frame;
                    return false;
                }
                else {
                    video->processed_frame += synthetic_frame_;
                    synthetic_frame_ = cv::Mat();
                    synthetic_exposure_count_ = 0;
                }
            }
            if(gamma_ != 1.0) {
                apply_gamma(video->processed_frame);
            }
            if(apply_darks_ && apply_flats_) {
                darks_and_flats(video->processed_frame);
            }
            else {
                if(apply_darks_) {
                    darks_only(video->processed_frame);
                }
                if(apply_flats_) {
                    video->processed_frame = video->processed_frame.mul(flats_);
                }
            }
            
            if(remove_gradient_) {
                remove_gradient(video->processed_frame);
            }

            if(derotator_) {
                if(first_frame_ts_ == 0)
                    first_frame_ts_ = video->timestamp;
                else {
                    double angle = derotator_->getAngleDeg(first_frame_ts_,video->timestamp);
                    if(derotate_mirror_)
                        angle = -angle;
                    BOOSTER_INFO("stacker") << "Derotating by " << angle << " dir " << (derotate_mirror_ ? "inv" : "str");
                    auto M = cv::getRotationMatrix2D(cv::Point2f(width_/2,height_/2),angle,1.0f);
                    cv::Mat frame_rotated;
                    cv::warpAffine(video->processed_frame,frame_rotated,M,cv::Size(width_,height_));
                    video->processed_frame = frame_rotated;
                }
            }
            return true;
        }
        void calc_xv_yv(int rows,int cols)
        {
            if(xv_.cols != cols) {
                xv_.create(1,cols,CV_32FC1);
                float *p=(float*)(xv_.data);
                float s2=0;
                for(int i=0;i<cols;i++) {
                    float v = (i + 0.5)/cols - 0.5;
                    s2 += v*v;
                    p[i] = v;
                }
                float nv = 1.0f/sqrt(rows * s2);
                xv_ *= nv;
            }
            if(yv_.rows != rows) {
                yv_.create(rows,1,CV_32FC1);
                float *p=(float*)(yv_.data);
                float s2=0;
                for(int i=0;i<rows;i++) {
                    float v = (i + 0.5)/rows - 0.5;
                    s2 += v*v;
                    p[i] = v;
                }
                float nv = 1.0f/sqrt(cols * s2);
                yv_ *= nv;
            }
        }
        void remove_gradient_mono(cv::Mat &img)
        {
            int rows = img.rows;
            int cols = img.cols;
            calc_xv_yv(rows,cols);
            float Ax = 0;
            float Ay = 0;
            float *p;
            p = (float*)(img.data);
            float *xv = (float*)xv_.data;
            float *yv = (float*)yv_.data;
            for(int r = 0;r<rows;r++) {
                float yv_r = yv[r];
                int c=0;
                #ifdef USE_CV_SIMD
                int limit = cols / 4 * 4;
                cv::v_float32x4 vec_y = cv::v_setall_f32(yv_r);
                cv::v_float32x4 Ay_v = cv::v_setzero_f32();
                cv::v_float32x4 Ax_v = cv::v_setzero_f32();
                for(;c<limit;c+=4,p+=4) {
                    cv::v_float32x4 val = cv::v_load(p);
                    cv::v_float32x4 vec_x = cv::v_load(xv + c);
                    Ay_v += val * vec_y;
                    Ax_v += val * vec_x;
                }
                Ay += cv::v_reduce_sum(Ay_v);
                Ax += cv::v_reduce_sum(Ax_v);
                #endif
                for(;c<cols;c++) {
                    float val = *p++;
                    Ax += xv[c] * val;
                    Ay += yv_r * val;
                }
            }
            
            float off = calc_grad_offset(Ay,Ax,rows,cols);

            p = (float*)(img.data);
            for(int r = 0;r<rows;r++) {
                float row_p = yv[r] * Ay + off;
                int c=0;
                #ifdef USE_CV_SIMD
                int limit = cols/4*4;
                cv::v_float32x4 Ax_v = cv::v_setall_f32(Ax);
                cv::v_float32x4 row_p_v = cv::v_setall_f32(row_p);
                cv::v_float32x4 v_0 = cv::v_setzero_f32();
                cv::v_float32x4 v_1 = cv::v_setall_f32(1.0f);
                for(;c<limit;c+=4,p+=4) {
                    cv::v_float32x4 plane_v = cv::v_load(xv + c) * Ax_v + row_p_v;
                    cv::v_float32x4 val_v = cv::v_load(p);
                    val_v = cv::v_max(v_0,cv::v_min(v_1,val_v - plane_v));
                    cv::v_store(p,val_v);
                }
                #endif
                for(;c<cols;c++) {
                    float plane = xv[c] * Ax + row_p;
                    float val = *p;
                    val = std::max(0.0f,std::min(1.0f,val - plane));
                    *p++ = val;
                }
            }
        }
        float calc_grad_offset(float Ay,float Ax,int rows,int cols)
        {
            float *xv = (float*)xv_.data;
            float *yv = (float*)yv_.data;
            float p00 = yv[     0] * Ay  + xv[       0] * Ax;
            float p10 = yv[rows-1] * Ay  + xv[       0] * Ax;
            float p01 = yv[     0] * Ay  + xv[cols - 1] * Ax;
            float p11 = yv[rows-1] * Ay  + xv[cols - 1] * Ax;
            float min_point = std::min(std::min(p00,p01),std::min(p10,p11));
            float off =  - min_point;
            return off;
        }
        
        void remove_gradient_color(cv::Mat &img)
        {
            int rows = img.rows;
            int cols = img.cols;
            calc_xv_yv(rows,cols);
            float ARx=0,AGx=0,ABx = 0;
            float ARy=0,AGy=0,ABy = 0;
            float *p;
            p = (float*)(img.data);
            float *xv = (float*)xv_.data;
            float *yv = (float*)yv_.data;
            for(int r = 0;r<rows;r++) {
                float yv_r = yv[r];
                int c=0;
                #ifdef USE_CV_SIMD
                int limit = cols / 4 * 4;
                cv::v_float32x4 vec_y = cv::v_setall_f32(yv_r);
                cv::v_float32x4 ARy_v = cv::v_setzero_f32();
                cv::v_float32x4 ARx_v = cv::v_setzero_f32();
                cv::v_float32x4 AGy_v = cv::v_setzero_f32();
                cv::v_float32x4 AGx_v = cv::v_setzero_f32();
                cv::v_float32x4 ABy_v = cv::v_setzero_f32();
                cv::v_float32x4 ABx_v = cv::v_setzero_f32();
                for(;c<limit;c+=4,p+=12) {
                    cv::v_float32x4 valR,valG,valB;
                    cv::v_load_deinterleave(p,valR,valG,valB);
                    cv::v_float32x4 vec_x = cv::v_load(xv + c);

                    ARy_v += valR * vec_y;
                    ARx_v += valR * vec_x;

                    AGy_v += valG * vec_y;
                    AGx_v += valG * vec_x;

                    ABy_v += valB * vec_y;
                    ABx_v += valB * vec_x;
                }

                ARy += cv::v_reduce_sum(ARy_v);
                ARx += cv::v_reduce_sum(ARx_v);

                AGy += cv::v_reduce_sum(AGy_v);
                AGx += cv::v_reduce_sum(AGx_v);
                
                ABy += cv::v_reduce_sum(ABy_v);
                ABx += cv::v_reduce_sum(ABx_v);
                #endif
                for(;c<cols;c++) {
                    float valR = *p++;
                    float valG = *p++;
                    float valB = *p++;
                    ARx += xv[c] * valR;
                    ARy += yv_r * valR;
                    AGx += xv[c] * valG;
                    AGy += yv_r * valG;
                    ABx += xv[c] * valB;
                    ABy += yv_r * valB;
                }
            }
           
            float offR = calc_grad_offset(ARy,ARx,rows,cols); 
            float offG = calc_grad_offset(AGy,AGx,rows,cols); 
            float offB = calc_grad_offset(ABy,ABx,rows,cols); 

            p = (float*)(img.data);
            for(int r = 0;r<rows;r++) {
                float rowR_p = yv[r] * ARy + offR;
                float rowG_p = yv[r] * AGy + offG;
                float rowB_p = yv[r] * ABy + offB;
                int c=0;
                #ifdef USE_CV_SIMD
                int limit = cols/4*4;

                cv::v_float32x4 ARx_v = cv::v_setall_f32(ARx);
                cv::v_float32x4 AGx_v = cv::v_setall_f32(AGx);
                cv::v_float32x4 ABx_v = cv::v_setall_f32(ABx);

                cv::v_float32x4 rowR_p_v = cv::v_setall_f32(rowR_p);
                cv::v_float32x4 rowG_p_v = cv::v_setall_f32(rowG_p);
                cv::v_float32x4 rowB_p_v = cv::v_setall_f32(rowB_p);

                cv::v_float32x4 v_0 = cv::v_setzero_f32();
                cv::v_float32x4 v_1 = cv::v_setall_f32(1.0f);

                for(;c<limit;c+=4,p+=12) {
                    cv::v_float32x4 planeR_v = cv::v_load(xv + c) * ARx_v + rowR_p_v;
                    cv::v_float32x4 planeG_v = cv::v_load(xv + c) * AGx_v + rowG_p_v;
                    cv::v_float32x4 planeB_v = cv::v_load(xv + c) * ABx_v + rowB_p_v;
                    cv::v_float32x4 valR_v,valG_v,valB_v;
                    cv::v_load_deinterleave(p,valR_v,valG_v,valB_v);
                    valR_v = cv::v_max(v_0,cv::v_min(v_1,valR_v - planeR_v));
                    valG_v = cv::v_max(v_0,cv::v_min(v_1,valG_v - planeG_v));
                    valB_v = cv::v_max(v_0,cv::v_min(v_1,valB_v - planeB_v));
                    cv::v_store_interleave(p,valR_v,valG_v,valB_v);
                }
                #endif
                for(;c<cols;c++) {
                    float planeR = xv[c] * ARx + rowR_p;
                    float planeG = xv[c] * AGx + rowG_p;
                    float planeB = xv[c] * ABx + rowB_p;
                    float valR = p[0];
                    float valG = p[1];
                    float valB = p[2];
                    valR = std::max(0.0f,std::min(1.0f,valR - planeR));
                    valG = std::max(0.0f,std::min(1.0f,valG - planeG));
                    valB = std::max(0.0f,std::min(1.0f,valB - planeB));
                    *p++ = valR;
                    *p++ = valG;
                    *p++ = valB;
                }
            }
        }
    
        #if 0
        void add_gradient(cv::Mat &img)
        {
            int step = mono_ ? 1 : 3;
            srand(1);
            for(int col=0;col<step;col++) {
                float *p=(float*)img.data + col;
                float scale = 0.05f;
                float dx=scale * ((float)rand()/RAND_MAX * 2 - 1);
                float dy=scale * ((float)rand()/RAND_MAX * 2 - 1);
                float o0 = std::max(-dx,0.0f) + std::max(-dy,0.0f);
                for(int r=0;r<img.rows;r++) {
                    for(int c=0;c<img.cols;c++) {
                        float plane = o0 + dx * c / img.cols + dy * r / img.rows;
                        *p = std::min(1.0f,plane + *p);
                        p+=step;
                    }
                }
            }
        }
        #endif
        void remove_gradient(cv::Mat &img)
        {
            auto start = std::chrono::high_resolution_clock::now();
            

            if(mono_)
                remove_gradient_mono(img);
            else
                remove_gradient_color(img);
            auto end = std::chrono::high_resolution_clock::now();
            double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(end-start).count();
            BOOSTER_INFO("stacker") << "Gradient removal took " << (1e3*time) << " ms";
        }

        void handle_config(std::shared_ptr<StackerControl> ctl)
        {
            switch(ctl->op) {
            case StackerControl::ctl_init:
                width_ = ctl->width;
                height_ = ctl->height;
                mono_ = ctl->mono;
                channels_ = mono_ ? 1 : 3;
                cv_type_ = mono_ ? CV_32FC1 : CV_32FC3;
                calibration_ = ctl->calibration;
                synthetic_exposure_mpl_ = 1;
                synthetic_frame_ = cv::Mat();
                synthetic_exposure_count_ = 0;
                remove_gradient_ = false;
                if(calibration_)
                    break;
                gamma_ = ctl->source_gamma;
                synthetic_exposure_mpl_ = ctl->synthetic_exposure_mpl;
                if(ctl->derotate) {
                    derotator_.reset(new Derotator(ctl->lon,ctl->lat,ctl->ra,ctl->de));
                    first_frame_ts_ = 0;
                    derotate_mirror_ = ctl->derotate_mirror;
                }
                else
                    derotator_.reset();
                apply_darks_ = false;
                remove_gradient_ = ctl->remove_gradient;
                if(!ctl->darks_path.empty()) 
                    load_darks(ctl->darks_path);
                apply_flats_ = false;
                if(!ctl->flats_path.empty())
                    load_flats(ctl->flats_path,ctl->dark_flats_path);
                break;
            case StackerControl::ctl_pause:
                // drop frames in case of pause
                if(synthetic_exposure_mpl_ > 1) {
                    BOOSTER_INFO("stacker") << "Dropping "<< synthetic_exposure_count_ << " non-synthetic frames due to pause";
                    synthetic_frame_ = cv::Mat();
                    synthetic_exposure_count_ = 0;  
                }
                break;
            default:
                /// not much to do
                ;
            }
        }
    private:
        bool check_file(cv::Mat &m,std::string const &type,std::string const &path)
        {
            if(m.rows != height_ || m.cols != width_ || m.channels() != channels_) {
                BOOSTER_ERROR("stacker") << "Invalid " << type  << "format, expecting " << height_ <<"x" << width_ << "x" <<channels_
                    << " got " << m.rows<<"x" <<m.cols<<"x"<<m.channels()<< " for file " << path;
                return false;    
            }
            return true;

        }
        void load_flats(std::string flats_path,std::string dark_flats_path)
        {
            try {
                apply_flats_=false;
                cv::Mat flats = load_tiff(flats_path);
                if(!check_file(flats,"flats",flats_path)) {
                    return;
                }
                if(!dark_flats_path.empty()) {
                    cv::Mat dark_flats = load_tiff(dark_flats_path);
                    if(!check_file(dark_flats,"dark flats",dark_flats_path)) {
                        return;
                    }
                    flats = cv::max(0.0f,flats - dark_flats);
                }
                cv::Mat gray_flats;
                if(flats.channels() > 1)
                    cv::cvtColor(flats,gray_flats,cv::COLOR_BGR2GRAY);
                else
                    gray_flats = flats;
                double minV,maxV;
                cv::minMaxLoc(gray_flats,&minV,&maxV);
                if(minV <= 0 || maxV / minV > 100) {
                    std::ostringstream ss;
                    ss << "Flats have too grate min/max range, suspecting collection issue - disableing flats minValue=" << minV << " maxValue="<<maxV;
                    BOOSTER_ERROR("stacker") << ss.str();
                    send_message(err_,"flats",ss.str());
                    return;
                }
                gray_flats = maxV/gray_flats;
                if(flats.channels() > 1)
                    cv::cvtColor(gray_flats,flats,cv::COLOR_GRAY2BGR);
                else
                    flats = gray_flats;
                flats_ = flats;
                apply_flats_=true;
            }
            catch(std::exception const &e) {
                std::ostringstream ss;
                ss <<  "Failed to load flats from " << flats_path << " and dark flats from " << dark_flats_path << ": " << e.what();
                BOOSTER_ERROR("stacker") << ss.str();
                send_message(err_,"flats",ss.str());
                apply_flats_ = false;
            }
        }
        void load_darks(std::string darks_path)
        {
            try {
                darks_ = load_tiff(darks_path);
                if(check_file(darks_,"darks",darks_path)) {
                    apply_darks_ = true;
                    cv::Mat tmp;
                    if(gamma_ != 1.0) {
                        cv::pow(darks_,gamma_,tmp);
                        darks_ = tmp;
                    }
                }
                else {
                    apply_darks_ = false;
                }
            }
            catch(std::exception const &e) {
                std::ostringstream ss;
                ss << "Failed to load darks from " << darks_path << ": " << e.what();
                send_message(err_,"darks",ss.str());
                BOOSTER_ERROR("stacker") << ss.str();
                apply_darks_ = false;
            }
        }

        queue_pointer_type in_,out_,err_;
        int width_,height_;
        bool mono_;
        int channels_;
        int cv_type_;
        bool calibration_;
        float gamma_;
        float gamma_table_[gamma_table_size+1];
        float gamma_table_current_gamma_ = -1;
        std::unique_ptr<Derotator> derotator_;
        double first_frame_ts_;
        bool derotate_mirror_;
        bool remove_gradient_;
        cv::Mat darks_;
        cv::Mat flats_;
        cv::Mat xv_,yv_;
        bool apply_darks_;
        bool apply_flats_;
        int synthetic_exposure_mpl_ = 1;
        int synthetic_exposure_count_ = 0;
        cv::Mat synthetic_frame_;
    };

    std::thread start_preprocessor(queue_pointer_type in,queue_pointer_type out,queue_pointer_type err)
    {
        std::shared_ptr<PreProcessor> p(new PreProcessor(in,out,err));
        return std::thread([=]() { p->run(); });
    }
    
    class StackerProcessor {
    public:
        StackerProcessor(queue_pointer_type in,queue_pointer_type out) :
            in_(in),
            out_(out)
        {
        }
        void run()
        {
            while(true) {
                auto data_ptr = in_->pop();
                auto stop_ptr = std::dynamic_pointer_cast<ShutDownData>(data_ptr);
                if(stop_ptr) {
                    if(out_)
                        out_->push(data_ptr);
                    break;
                }
                auto video_ptr = std::dynamic_pointer_cast<CameraFrame>(data_ptr);
                if(video_ptr) {
                    std::shared_ptr<StackedFrame> res = handle_video(video_ptr);
                    if(out_) {
                        out_->push_or_replace(res);
                    }
                    continue;
                }
                auto config_ptr = std::dynamic_pointer_cast<StackerControl>(data_ptr);
                if(config_ptr) {
                    try {
                        handle_config(config_ptr);
                    }
                    catch(std::exception const &e) {
                        send_message(out_,"Config",e);
                        BOOSTER_ERROR("stacker") << "Config change error:" <<e.what();
                    }
                }
                if(out_)
                    out_->push(data_ptr);
            }
        }

        void send_updated_image()
        {
            if(out_) {
                if(stacker_->stacked_count() > 0) {
                    std::shared_ptr<StackedFrame> res(new StackedFrame());
                    res->frame = stacker_->get_raw_stacked_image();
                    res->roi = stacker_->fully_stacked_area();
                    create_stats(res);
                    out_->push(res);
                }
            }
        }

        void create_stats(std::shared_ptr<StatsBase> stats)
        {
            if(calibration_) {
                stats->stacked = cframe_count_;
            }
            else {
                stats->stacked = stacker_->stacked_count();
                stats->missed  = stacker_->total_count() - stats->stacked;
            }
            stats->dropped = dropped_count_;
        }

        std::shared_ptr<StackedFrame> handle_video(std::shared_ptr<CameraFrame> video)
        {
            std::shared_ptr<StackedFrame> res(new StackedFrame());
		    auto start = std::chrono::high_resolution_clock::now();
            try {
                dropped_count_ += video->dropped;
                if(calibration_) {
                    cframe_ +=  video->processed_frame;
                    cframe_count_ ++;
                    res->frame = cframe_.mul(cv::Scalar::all(1.0 / cframe_count_));
                    auto end = std::chrono::high_resolution_clock::now();
                    double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(end-start).count();
                    BOOSTER_INFO("stacker") << "Stacking took " << (1e3*time) << " ms, calibration frame #" << cframe_count_;
                }
                else {
                    if(stacker_->stack_image(video->processed_frame,restart_)) {
                        restart_ = false;
		                auto p1 = std::chrono::high_resolution_clock::now();
                        double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(p1-start).count();
                        res->frame = stacker_->get_raw_stacked_image();
                        res->roi = stacker_->fully_stacked_area();
                        BOOSTER_INFO("stacker") << "Stacking took " << (1e3*time) << " ms";
                    }
                    else {
                        BOOSTER_INFO("stacker") << "Failed to stack frame";
                        auto end = std::chrono::high_resolution_clock::now();
                        double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(end-start).count();
                        BOOSTER_INFO("stacker") << "Stacking took " << (1e3*time) << " ms";
                        return nullptr;
                    }
                }
                create_stats(res);
            }
            catch(std::exception const &e) {
                BOOSTER_ERROR("stacker") << "Stacking Failed:" << e.what();
                return nullptr;
            }
            return res; 
        }
        void handle_config(std::shared_ptr<StackerControl> ctl)
        {
            switch(ctl->op) {
            case StackerControl::ctl_init:
                width_ = ctl->width;
                height_ = ctl->height;
                mono_ = ctl->mono;
                version_ = 0;
                channels_ = mono_ ? 1 : 3;
                cv_type_ = mono_ ? CV_32FC1 : CV_32FC3;
                calibration_ = ctl->calibration;
                dropped_count_ = 0;
                stacker_.reset();
                if(calibration_) {
                    cframe_ = cv::Mat(height_,width_,cv_type_);
                    cframe_.setTo(0);
                    cframe_count_ = 0;
                }
                else {
                    stacker_.reset(new Stacker(width_,height_,channels_));
                    stacker_->set_remove_satellites(ctl->remove_satellites);
                    stacker_->set_rollback_on_pause(ctl->rollback_on_pause);
                    restart_ = true;
                }
                break;
            case StackerControl::ctl_pause:
                restart_ = true;
                if(stacker_) {
                    stacker_->handle_pause();
                    send_updated_image();
                }
                break;
            case StackerControl::ctl_cancel:
                if(stacker_) {
                    stacker_.reset();
                }
                else if(calibration_) {
                    calibration_ = false;
                }
                break;
            default:
                /// not much to do
                ;
            }
        }
    private:
        queue_pointer_type in_,out_;
        int width_,height_;
        bool mono_;
        int version_;
        int channels_,cv_type_;
        bool calibration_=false;
        cv::Mat cframe_;
        int cframe_count_;
        int dropped_count_ = 0;
        std::unique_ptr<Stacker> stacker_;
        bool restart_;
    };

    std::thread start_stacker(queue_pointer_type in,queue_pointer_type out)
    {
        std::shared_ptr<StackerProcessor> p(new StackerProcessor(in,out));
        return std::thread([=]() { p->run(); });
    }

    class PostProcessorProcessor {
    public:
        PostProcessorProcessor(queue_pointer_type in,queue_pointer_type out,queue_pointer_type stats,queue_pointer_type plate_solving,std::string data_dir) :
            in_(in),
            out_(out),
            stats_(stats),
            plate_solving_(plate_solving),
            data_dir_(data_dir),
            pp_(new PostProcessor())
        {
        }
        void run()
        {
            while(true) {
                auto data_ptr = in_->pop();
                auto stop_ptr = std::dynamic_pointer_cast<ShutDownData>(data_ptr);
                if(stop_ptr) {
                    if(out_)
                        out_->push(data_ptr);
                    break;
                }
                auto video_ptr = std::dynamic_pointer_cast<StackedFrame>(data_ptr);
                if(video_ptr) {
                    auto frames = handle_video(video_ptr);
                    auto res = frames.first;
                    auto ps  = frames.second;
                    if(res) {
                        if(out_)
                            out_->push(res);
                    }
                    if(ps && plate_solving_) {
                        plate_solving_->push(ps);
                    }
                    continue;
                }
                auto config_ptr = std::dynamic_pointer_cast<StackerControl>(data_ptr);
                if(config_ptr) {
                    try {
                        handle_config(config_ptr);
                    }
                    catch(std::exception const &e) {
                        send_message(stats_,"Config",e);
                        BOOSTER_ERROR("stacker") << "Config change error:" <<e.what();
                    }
                }
                auto error_ptr = std::dynamic_pointer_cast<ErrorNotificationData>(data_ptr);
                if(error_ptr){
                    if(stats_) {
                        stats_->push(error_ptr);
                        continue;
                    }
                }
                if(out_)
                    out_->push(data_ptr);
            }
        }

        void save_stretch(StretchInfo const &stretch)
        {
            cppcms::json::value s;
            s["gain"]   = stretch.gain;
            s["cut"]    = stretch.cut;
            s["gamma" ] = stretch.gamma;
            s["auto_stretch"] = stretch.auto_stretch;
            std::string fname = data_dir_ + "/stretch.json";
            std::ofstream f(fname);
            if(!f) {
                BOOSTER_ERROR("stacker") << " Failed to save info to " << fname;
                return;
            }
            s.save(f,cppcms::json::readable);
            f.close();
        }
        

        std::pair<std::shared_ptr<CameraFrame>,std::shared_ptr<CameraFrame> > generate_output_frame(std::pair<cv::Mat,StretchInfo> data,bool create_ps_frame=true)
        {
            cv::Mat img = data.first;
            save_stretch(data.second);
            cv::Mat img8;
            img.convertTo(img8,CV_8UC3,255);
            std::shared_ptr<CameraFrame> frame(new CameraFrame());
            std::shared_ptr<CameraFrame> plate_solving_frame;
            frame->format.width = img8.cols;
            frame->format.height = img8.rows;
            std::vector<unsigned char> buf;
            cv::imencode(".jpeg",img8,buf);
            frame->jpeg_frame = std::shared_ptr<VideoFrame>(new VideoFrame(buf.data(),buf.size()));
            if(plate_solving_ && create_ps_frame) {
                plate_solving_frame.reset(new CameraFrame());
                plate_solving_frame->format.width = img8.cols;
                plate_solving_frame->format.height = img8.rows;
                plate_solving_frame->frame = img8;
                plate_solving_frame->frame_dr = 255;
            }
            return std::make_pair(frame,plate_solving_frame);
        }

        cv::Mat to16bit(cv::Mat m)
        {
            cv::Mat m2 = cv::max(0,m);
            double max_v;
            cv::minMaxLoc(m2,nullptr,&max_v);
            cv::Mat res;
            m2.convertTo(res,m.channels() == 3 ? CV_16UC3 : CV_16UC1,65535/max_v);
            return res;
        }


        void create_meta(std::ostream &m)
        {
            int frames = last_frame_->stacked;
            double exp = get_exp_s();
            m << "Name            " << name_ << std::endl;
            m << "Time            " << timestamp() << std::endl;
            m << "Frames          " << frames  << std::endl;
            m << "True Frames     " << frames * stack_info_.synthetic_exposure_mpl << std::endl;
            m << "Integration (s) " << frames * exp << std::endl;
            m << "Frame:"<<std::endl;
            m << " - format " << stack_info_.format << std::endl;
            m << " - width  " << stack_info_.width << std::endl;
            m << " - height " << stack_info_.height << std::endl;
            m << " - bin    " << stack_info_.bin << std::endl;
            m << " - mono   " << stack_info_.mono << std::endl;
            m << "Stackig:"<<std::endl;
            m << " - remove_satellites  " << stack_info_.remove_satellites << std::endl;
            m << " - derotate           " << stack_info_.derotate << std::endl;
            m << " - derotate_mirror    " << stack_info_.derotate_mirror << std::endl;
            m << " - remove_gradient    " << stack_info_.remove_gradient << std::endl;
            m << "Target:" << std::endl;
            m << " - RA " << stack_info_.ra << std::endl;
            m << " - DE " << stack_info_.de << std::endl;
            m << "Geolocation" << std::endl;
            m << " - lat " << stack_info_.lat << std::endl;
            m << " - lon " << stack_info_.lon << std::endl;
            m << "Camera Settings:"<<std::endl;
            for(auto op:stack_info_.camera_config) {
                m << " - " << cam_option_id_to_name(op.first) << ": " << op.second << std::endl;
            }
        }
        void save_stacked_image_and_send()
        {
            version_++;
            std::string base_name = output_path_ + "_stacked_v" + std::to_string(version_);
            std::string path = base_name + ".jpeg";
            std::string ipath = base_name + ".txt";
            std::string tpath = base_name + ".tiff";
            save_tiff(to16bit(last_frame_->frame),tpath);
            auto img = pp_->post_process_image(last_frame_->frame,last_frame_->roi);
            auto frames = generate_output_frame(img,false);
            auto frame = frames.first;
            std::ofstream f(path,std::ofstream::binary);
            f.write((char*)frame->jpeg_frame->data(),frame->jpeg_frame->size());
            f.close();
            if(out_)
                out_->push(frame);
            std::ofstream log(ipath);
            create_meta(log);
        }

        std::string timestamp()
        {
            return ftime("%Y-%m-%d %H:%M:%S",time(nullptr));
        }

        double get_exp_s()
        {
            return stack_info_.camera_config[opt_exp] * 1e-3 * stack_info_.synthetic_exposure_mpl; // ms to s
        }

        std::shared_ptr<StatsData> create_stats()
        {
            std::shared_ptr<StatsData> stats(new StatsData(*last_frame_));
            stats->since_saved_s = get_exp_s() * (stats->stacked - saved_count_);
            if(stats->stacked>0 && !calibration_) {
                stats->histogramm = std::move(pp_->get_histogramm());
            }
            return stats;
        }

        std::pair<std::shared_ptr<CameraFrame>,std::shared_ptr<CameraFrame> > handle_video(std::shared_ptr<StackedFrame> video)
        {
            last_frame_ = video;
            std::shared_ptr<CameraFrame> res;
            std::shared_ptr<CameraFrame> ps;
            try {
                if(out_ && last_frame_) {
                    auto p1 = std::chrono::high_resolution_clock::now();
                    std::pair<cv::Mat,StretchInfo> img;
                    if(calibration_)
                        img.first=last_frame_->frame;
                    else
                        img = pp_->post_process_image(last_frame_->frame,last_frame_->roi);
                    auto p2 = std::chrono::high_resolution_clock::now();
                    auto frames = generate_output_frame(img);
                    res=frames.first;
                    ps=frames.second;
                    auto p3 = std::chrono::high_resolution_clock::now();
                    double gtime = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(p2-p1).count();
                    double jtime = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(p3-p2).count();
                    double total_time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(p3-p1).count();
                    BOOSTER_INFO("stacker") << "post processing took " << (1e3 *total_time) <<"ms. proccessing " << (1e3*gtime) << " ms, jpeg took=" << (1e3*jtime);
                }
                if(stats_ && last_frame_) {
                    stats_->push(create_stats());
                }
            }
            catch(std::exception const &e) {
                send_message(stats_,"Stacking",e);
                BOOSTER_ERROR("stacker") << "Stacking Failed:" << e.what();
            }
            return std::make_pair(res,ps);
        }
        void save_calibration()
        {
            cv::Mat calib = last_frame_->frame;
            double minV,maxV;
            cv::minMaxLoc(calib,&minV,&maxV);
            std::string tiff_path = output_path_ + "/" + name_ + ".tiff";
            std::string db_path = output_path_ + "/index.json";
            BOOSTER_INFO("stacker") << "Saving calibration frame to " << tiff_path << " frame " << last_frame_->stacked << " maxv=" << maxV << " minv=" << minV;
            cppcms::json::value setup;
            setup["id"] = name_;
            setup["path"] = name_ + ".tiff";
            setup["date"] = timestamp();
            setup["frames"] = last_frame_->stacked;
            setup["width"] = calib.cols;
            setup["height"] = calib.rows;
            BOOSTER_INFO("stacker") << "Saving calibration frame to " << tiff_path;
            save_tiff(calib,tiff_path);
            cppcms::json::value db;
            std::ifstream indx(db_path);
            bool found=false;
            if(!db.load(indx,true)) {
                BOOSTER_ERROR("stacker") << "Error parsing " << db_path << " darks DB file " << std::endl;
                send_message(stats_,"save calibration","Error parsing calibrarion index file:" + db_path);
                db = cppcms::json::value();
                db[0] = setup;
                found = true;
            }
            cppcms::json::array &ar=db.array();
            for(size_t i=0;!found && i<ar.size();i++) {
                if(ar[i].get<std::string>("id") == name_) {
                    ar[i] = setup;
                    found = true;
                    break;
                }
            }
            if(!found)
                ar.push_back(setup);
            std::ofstream res(db_path);
            db.save(res,cppcms::json::readable);
            res.close();
        }
        std::shared_ptr<CameraFrame> generate_dummy_frame()
        {
            return ols::generate_dummy_frame(width_,height_,mono_ ? 1 : 3);
        }

        void handle_config(std::shared_ptr<StackerControl> ctl)
        {
            switch(ctl->op) {
            case StackerControl::ctl_init:
                width_ = ctl->width;
                height_ = ctl->height;
                mono_ = ctl->mono;
                saved_count_ = 0;
                version_ = 0;
                calibration_ = ctl->calibration;
                output_path_ = ctl->output_path;
                name_ = ctl->name;
                dropped_count_ = 0;
                stack_info_ = *ctl;
                last_frame_ = nullptr;
                pp_->set_stretch(ctl->auto_stretch,ctl->stretch_low,ctl->stretch_high,ctl->stretch_gamma);
                if(out_) {
                    out_->push(generate_dummy_frame());
                }
                if(stats_) {
                    std::shared_ptr<StatsData> stats(new StatsData());
                    stats_->push(stats);
                }
                break;
            case StackerControl::ctl_cancel:
                last_frame_ = nullptr;
                break;
            case StackerControl::ctl_save:
                if(last_frame_) {
                    if(calibration_) {
                        save_calibration();
                    }
                    else {
                        save_stacked_image_and_send();
                    }
                    saved_count_ = last_frame_->stacked;
                }
                if(stats_) {
                    stats_->push(create_stats());
                }
                break;
            case StackerControl::ctl_update:
                if(!calibration_) {
                    pp_->set_stretch(ctl->auto_stretch,ctl->stretch_low,ctl->stretch_high,ctl->stretch_gamma);
                    BOOSTER_INFO("stacker") << "Getting to stretch settings in stacker auto="<<ctl->auto_stretch << " low="<<ctl->stretch_low << " high=" << ctl->stretch_high << " gamma=" << ctl->stretch_gamma;
                    if(last_frame_) {
                        auto frames = handle_video(last_frame_);
                        auto res = frames.first;
                        auto ps  = frames.second;
                        if(res && out_) {
                            out_->push(res);
                        }
                        if(ps && plate_solving_) {
                            plate_solving_->push(ps);
                        }
                    }
                    
                }
                break;
            default:
                /// not much to do
                ;
            }
        }
    private:
        queue_pointer_type in_,out_,stats_,plate_solving_;
        std::string data_dir_;
        int width_,height_;
        bool mono_;
        int version_;
        bool calibration_=false;
        std::string output_path_,name_;
        cv::Mat cframe_;
        int cframe_count_;
        int dropped_count_ = 0;
        std::unique_ptr<PostProcessor> pp_;
        std::shared_ptr<StackedFrame> last_frame_;
        int saved_count_ = 0;
        StackerControl stack_info_;
    };

    std::thread start_post_processor(queue_pointer_type in,queue_pointer_type out,queue_pointer_type stats,queue_pointer_type plate_solving,std::string data_dir)
    {
        std::shared_ptr<PostProcessorProcessor> p(new PostProcessorProcessor(in,out,stats,plate_solving,data_dir));
        return std::thread([=]() { p->run(); });
    }
 
    
    
    class DebugSaver {
    public:
        DebugSaver(queue_pointer_type in,queue_pointer_type err,std::string output_dir) :
            in_(in),
            err_(err),
            out_(output_dir),
            counter_(0),
            save_(false)
        {
        }
        void run()
        {
            while(true) {
                auto data_ptr = in_->pop();
                auto stop_ptr = std::dynamic_pointer_cast<ShutDownData>(data_ptr);
                if(stop_ptr) {
                    break;
                }
                auto video_ptr = std::dynamic_pointer_cast<CameraFrame>(data_ptr);
                if(save_ && video_ptr) {
                    try {
                        handle_video(video_ptr);
                    }
                    catch(std::exception const &e) {
                        send_message(err_,"debug save",e);
                    }
                    continue;
                }
                auto config_ptr = std::dynamic_pointer_cast<StackerControl>(data_ptr);
                if(config_ptr) {
                    try {
                        handle_config(config_ptr);
                    }
                    catch(std::exception const &e) {
                        send_message(err_,"debug config",e);
                    }

                }
            }
        }
        std::string log_file()
        {
            return dirname_ + "/log.txt";
        }
        void handle_video(std::shared_ptr<CameraFrame> video)
        {
            char fname[256];
            snprintf(fname,sizeof(fname),"frame_%08d",counter_);
            std::string base_name = dirname_ + "/" + fname;
            if(video->format.format == stream_mjpeg) {
                std::ofstream f(base_name + ".jpeg",std::ofstream::binary);
                f.write((char*)video->source_frame->data(),video->source_frame->size());
                if(!f) {
                    BOOSTER_ERROR("stacker") << "Failed to save jpeg to " << base_name << ".jpeg ";
                    send_message(err_,"save jpeg","Failed to save jpeg to "  + base_name + ".jpeg");
                }
                f.close();
            }
            else {
                try {
                    save_tiff(video->raw,base_name + ".tiff");
                }
                catch(std::exception const &e){
                    send_message(err_,"save tiff",e.what());
                    BOOSTER_ERROR("stacker") << "Failed to save tiff to " << base_name << ".tiff: " << e.what();
                }
            }
            std::ofstream log(log_file(),std::ofstream::app);
            log << counter_ <<"," << std::fixed << std::setprecision(3) << video->timestamp << std::endl;
            counter_++;
            if(counter_ == 1) {
                cppcms::json::value v;
                std::ifstream info_r(dirname_ + "/info.json");
                if(v.load(info_r,true)) {
                    info_r.close();
                    v["format"] = stream_type_to_str(video->format.format);
                    v["bayer"] = bayer_type_to_str(video->bayer);
                    std::ofstream info(dirname_ + "/info.json");
                    if(!info) {
                        BOOSTER_ERROR("stacker") << "Failed to update JSON info from "<< dirname_ << "/info.json";
                        send_message(err_,"update info","Failed to update JSON info from " + dirname_ + "/info.json");
                    }
                    else {
                        v.save(info,cppcms::json::readable);
                    }
                }
                else {
                    BOOSTER_ERROR("stacker") << "Failed to read JSON info from "<< dirname_ << "/info.json";
                    send_message(err_,"read info","Failed to read info from " + dirname_ + "/info.json");
                }
            }
        }
        void handle_config(std::shared_ptr<StackerControl> ctl)
        {
            switch(ctl->op) {
            case StackerControl::ctl_init:
                {
                    save_ = ctl->save_inputs;
                    if(!save_)
                        return;
                    dirname_ = out_ + "/" + ctl->name;
                    counter_ = 0;
                    make_dir(dirname_);
                    cppcms::json::value v;
                    v["name"]=ctl->name;
                    v["width"] = ctl->width;
                    v["height"] = ctl->height;
                    v["mono"] = ctl->mono;
                    v["synthetic_exposure_mpl"] = ctl->synthetic_exposure_mpl;
                    v["darks"] = ctl->darks_path;
                    v["flats"] = ctl->flats_path;
                    v["dark_flats"] = ctl->dark_flats_path;
                    v["calibration"] = ctl->calibration;
                    v["derotate"] = ctl->derotate;
                    v["derotate_mirror"] = ctl->derotate_mirror;
                    v["remove_gradient"] = ctl->remove_gradient;
                    v["ra"] = ctl->ra;
                    v["de"] = ctl->de;
                    v["lat"] = ctl->lat;
                    v["lon"] = ctl->lon;
                    v["source_gamma"] = ctl->source_gamma;
                    v["auto_stretch"] = ctl->auto_stretch;
                    v["stretch_low"] = ctl->stretch_low;
                    v["stretch_high"] = ctl->stretch_high;
                    v["stretch_gamma"] = ctl->stretch_gamma;
                    v["remove_satellites" ] = ctl->remove_satellites;
                    std::ofstream info(dirname_ + "/info.json");
                    v.save(info,cppcms::json::readable);
                }
                break;
            case StackerControl::ctl_update:
                {
                    if(!save_)
                        return;
                    std::ifstream info_r(dirname_ + "/info.json");
                    cppcms::json::value v;
                    if(v.load(info_r,true)) {
                        info_r.close();
                        std::ofstream info(dirname_ + "/info.json");
                        v["auto_stretch"] = ctl->auto_stretch;
                        v["stretch_low"] = ctl->stretch_low;
                        v["stretch_high"] = ctl->stretch_high;
                        v["stretch_gamma"] = ctl->stretch_gamma;
                        v.save(info,cppcms::json::readable);
                    }
                }
                break;
            case StackerControl::ctl_pause:
                {
                    if(!save_)
                        return;
                    std::ofstream log(log_file(),std::ofstream::app);
                    log << "PAUSE,0" << std::endl;
                }
                break;

            default:
                /// not much to do
                ;
            }
        }
    private:

        queue_pointer_type in_,err_;
        std::string out_;
        std::string dirname_;
        int counter_;
        bool save_;
    };

    std::thread start_debug_saver(queue_pointer_type in,queue_pointer_type err,std::string debug_dir)
    {
        std::shared_ptr<DebugSaver> p(new DebugSaver(in,err,debug_dir));
        return std::thread([=]() { p->run(); });
    }

}
