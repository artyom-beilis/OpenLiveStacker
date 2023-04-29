#include "data_items.h"
#include "rotation.h"
#include "stacker.h"
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
    class PreProcessor {
    public:
        constexpr static int gamma_table_size = 128;
        PreProcessor(queue_pointer_type in,queue_pointer_type out) :
            in_(in),out_(out)
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
            int limit = N/4*4;
            for(;i<limit;i+=4,p+=4,d+=4,f+=4) {
                cv::v_float32x4 v=cv::v_max(zero,cv::v_load(p) - cv::v_load(d));
                v *= cv::v_load(f);
                cv::v_store(p,v);
            }
#endif            
            for(;i<N;i++) {
                float v=std::max(0.0f,*p - *d);
                v*= *f;
                *p = v;
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
            video->frame.convertTo(video->processed_frame,cv_type_,1.0/video->frame_dr);
            if(calibration_)
                return true;
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
                if(calibration_)
                    break;
                gamma_ = ctl->source_gamma;
                if(ctl->derotate) {
                    derotator_.reset(new Derotator(ctl->lon,ctl->lat,ctl->ra,ctl->de));
                    first_frame_ts_ = 0;
                    derotate_mirror_ = ctl->derotate_mirror;
                }
                else
                    derotator_.reset();
                apply_darks_ = false;
                if(!ctl->darks_path.empty()) 
                    load_darks(ctl->darks_path);
                apply_flats_ = false;
                if(!ctl->flats_path.empty())
                    load_flats(ctl->flats_path,ctl->dark_flats_path);
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
                    BOOSTER_ERROR("stacker") << " Flats have too grate min/max range, suspecting collection issue - disableing flats minValue=" << minV << " maxValue="<<maxV;
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
                BOOSTER_ERROR("stacker") << "Failed to load flats from " << flats_path << " and dark flats from " << dark_flats_path << ": " << e.what();
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
                BOOSTER_ERROR("stacker") << "Failed to load darks from " << darks_path << ": " << e.what();
                apply_darks_ = false;
            }
        }

        queue_pointer_type in_,out_;
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
        cv::Mat darks_;
        cv::Mat flats_;
        bool apply_darks_;
        bool apply_flats_;
    };

    std::thread start_preprocessor(queue_pointer_type in,queue_pointer_type out)
    {
        std::shared_ptr<PreProcessor> p(new PreProcessor(in,out));
        return std::thread([=]() { p->run(); });
    }
    
    class StackerProcessor {
    public:
        StackerProcessor(queue_pointer_type in,queue_pointer_type out,queue_pointer_type stats,queue_pointer_type plate_solving,std::string data_dir) :
            in_(in),
            out_(out),
            stats_(stats),
            plate_solving_(plate_solving),
            data_dir_(data_dir)
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
                    handle_config(config_ptr);
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
            std::string fname = data_dir_ + "/stretch.json";
            std::ofstream f(fname);
            if(!f) {
                BOOSTER_ERROR("stacker") << " Failed to save info to " << fname;
                return;
            }
            s.save(f,cppcms::json::readable);
            f.close();
        }
        
        std::shared_ptr<CameraFrame> generate_dummy_frame()
        {
            return ols::generate_dummy_frame(width_,height_,channels_);
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

        void send_updated_image()
        {
            if(out_) {
                if(stacker_->stacked_count() > 0) {
                    auto frames = generate_output_frame(stacker_->get_stacked_image());
                    out_->push(frames.first);
                    if(plate_solving_)
                        plate_solving_->push(frames.second);
                }
                else
                    out_->push(generate_dummy_frame());
            }
        }

        cv::Mat to16bit(cv::Mat m)
        {
            cv::Mat m2 = cv::max(0,m);
            double max_v;
            cv::minMaxLoc(m2,nullptr,&max_v);
            cv::Mat res;
            m2.convertTo(res,channels_ == 3 ? CV_16UC3 : CV_16UC1,65535/max_v);
            std::cerr << "Channels to 16bit" << channels_ << std::endl;
            return res;
        }

        void save_stacked_image_and_send()
        {
            std::string path = output_path_ + "_stacked.jpeg";
            std::string ipath = output_path_ + "_stacked.txt";
            std::string tpath = output_path_ + "_stacked.tiff";
            save_tiff(to16bit(stacker_->get_raw_stacked_image()),tpath);
            auto frames = generate_output_frame(stacker_->get_stacked_image(),false);
            auto frame = frames.first;
            std::ofstream f(path,std::ofstream::binary);
            f.write((char*)frame->jpeg_frame->data(),frame->jpeg_frame->size());
            f.close();
            if(out_)
                out_->push(frame);
            std::ofstream log(ipath);
            log << "Object: " << name_ << "\n";
            log << "When: " <<timestamp() << "\n";
            log << "Stacked: " << stacker_->stacked_count() << std::endl;
        }

        std::string timestamp()
        {
            return ftime("%Y-%m-%d %H:%M:%S",time(nullptr));
        }


        std::pair<std::shared_ptr<CameraFrame>,std::shared_ptr<CameraFrame> > handle_video(std::shared_ptr<CameraFrame> video)
        {
            std::shared_ptr<CameraFrame> res;
            std::shared_ptr<CameraFrame> ps;
            std::shared_ptr<StatsData> stats(new StatsData());
		    auto start = std::chrono::high_resolution_clock::now();
            try {
                dropped_count_ += video->dropped;
                stats->dropped = dropped_count_;
                if(calibration_) {
                    cframe_ +=  video->processed_frame;
                    cframe_count_ ++;
                    stats->stacked = cframe_count_;
                    res = video;
                    auto end = std::chrono::high_resolution_clock::now();
                    double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(end-start).count();
                    BOOSTER_INFO("stacker") << "Stacking took " << (1e3*time) << " ms, calibration frame #" << cframe_count_;
                }
                else {
                    if(stacker_->stack_image(video->processed_frame,restart_)) {
                        restart_ = false;
		                auto p1 = std::chrono::high_resolution_clock::now();
                        double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(p1-start).count();
                        double gtime = 0,jtime = 0;
                        if(out_) {
                            auto img = stacker_->get_stacked_image();
                            auto p2 = std::chrono::high_resolution_clock::now();
                            auto frames = generate_output_frame(img);
                            res=frames.first;
                            ps=frames.second;
                            auto p3 = std::chrono::high_resolution_clock::now();
                            gtime = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(p2-p1).count();
                            jtime = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(p3-p2).count();
                        }
                        BOOSTER_INFO("stacker") << "Stacking took " << (1e3*time) << " ms, generation " << (1e3*gtime) << " ms, jpeg took=" << (1e3*jtime);
                    }
                    else {
                        BOOSTER_INFO("stacker") << "Failed to stack frame";
                        auto end = std::chrono::high_resolution_clock::now();
                        double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(end-start).count();
                        BOOSTER_INFO("stacker") << "Stacking took " << (1e3*time) << " ms";
                    }
                    stats->stacked = stacker_->stacked_count();
                    stats->missed  = stacker_->total_count() - stats->stacked;
                }
                if(stats_)
                    stats_->push(stats);
            }
            catch(std::exception const &e) {
                BOOSTER_ERROR("stacker") << "Stacking Failed:" << e.what();
            }
            return std::make_pair(res,ps);
        }
        void save_calibration()
        {
            double factor = 1.0 / cframe_count_;
            cv::Mat calib = cframe_.mul(cv::Scalar::all(factor)); 
            double minV,maxV;
            cv::minMaxLoc(calib,&minV,&maxV);
            std::string tiff_path = output_path_ + "/" + name_ + ".tiff";
            std::string db_path = output_path_ + "/index.json";
            BOOSTER_INFO("stacker") << "Saving calibration frame to " << tiff_path << " frame " << cframe_count_ << " maxv=" << maxV << " minv=" << minV;
            cppcms::json::value setup;
            setup["id"] = name_;
            setup["path"] = name_ + ".tiff";
            setup["date"] = timestamp();
            setup["frames"] = cframe_count_;
            setup["width"] = calib.cols;
            setup["height"] = calib.rows;
            BOOSTER_INFO("stacker") << "Saving calibration frame to " << tiff_path;
            save_tiff(calib,tiff_path);
            cppcms::json::value db;
            std::ifstream indx(db_path);
            bool found=false;
            if(!db.load(indx,true)) {
                BOOSTER_ERROR("stacker") << "Error parsing " << db_path << " darks DB file " << std::endl;
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
                output_path_ = ctl->output_path;
                name_ = ctl->name;
                dropped_count_ = 0;
                stacker_.reset();
                if(calibration_) {
                    cframe_ = cv::Mat(height_,width_,cv_type_);
                    cframe_.setTo(0);
                    cframe_count_ = 0;
                }
                else {
                    stacker_.reset(new Stacker(width_,height_,channels_));
                    stacker_->set_stretch(ctl->auto_stretch,ctl->stretch_low,ctl->stretch_high,ctl->stretch_gamma);
                    stacker_->set_remove_satellites(ctl->remove_satellites);
                    restart_ = true;
                }
                if(out_)
                    out_->push(generate_dummy_frame());
                if(stats_) {
                    std::shared_ptr<StatsData> stats(new StatsData());
                    stats_->push(stats);
                }
                break;
            case StackerControl::ctl_pause:
                restart_ = true;
                break;
            case StackerControl::ctl_cancel:
                if(stacker_) {
                    stacker_.reset();
                }
                else if(calibration_) {
                    calibration_ = false;
                }
                break;
            case StackerControl::ctl_save:
                if(stacker_) {
                    save_stacked_image_and_send();
                    stacker_.reset();
                }
                else if(calibration_) {
                    save_calibration();
                    calibration_ = false;
                }
                break;
            case StackerControl::ctl_update:
                if(stacker_) {
                    stacker_->set_stretch(ctl->auto_stretch,ctl->stretch_low,ctl->stretch_high,ctl->stretch_gamma);
                    BOOSTER_INFO("stacker") << "Getting to stretch settings in stacker auto="<<ctl->auto_stretch << " low="<<ctl->stretch_low << " high=" << ctl->stretch_high << " gamma=" << ctl->stretch_gamma;
                    send_updated_image();
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
        int channels_,cv_type_;
        bool calibration_=false;
        std::string output_path_,name_;
        cv::Mat cframe_;
        int cframe_count_;
        int dropped_count_ = 0;
        std::unique_ptr<Stacker> stacker_;
        bool restart_;
    };

    std::thread start_stacker(queue_pointer_type in,queue_pointer_type out,queue_pointer_type stats,queue_pointer_type plate_solving,std::string data_dir)
    {
        std::shared_ptr<StackerProcessor> p(new StackerProcessor(in,out,stats,plate_solving,data_dir));
        return std::thread([=]() { p->run(); });
    }

    class DebugSaver {
    public:
        DebugSaver(queue_pointer_type in,std::string output_dir) :
            in_(in),
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
                    handle_video(video_ptr);
                    continue;
                }
                auto config_ptr = std::dynamic_pointer_cast<StackerControl>(data_ptr);
                if(config_ptr) {
                    handle_config(config_ptr);
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
                }
                f.close();
            }
            else {
                try {
                    save_tiff(video->frame,base_name + ".tiff");
                }
                catch(std::exception const &e){
                    BOOSTER_ERROR("stacker") << "Failed to save tiff to " << base_name << ".tiff: " << e.what();
                }
            }
            std::ofstream log(log_file(),std::ofstream::app);
            log << counter_ <<"," << std::fixed << std::setprecision(3) << video->timestamp << std::endl;
            counter_++;
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
                    v["darks"] = ctl->darks_path;
                    v["flats"] = ctl->flats_path;
                    v["dark_flats"] = ctl->dark_flats_path;
                    v["calibration"] = ctl->calibration;
                    v["derotate"] = ctl->derotate;
                    v["derotate_mirror"] = ctl->derotate_mirror;
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

        queue_pointer_type in_;
        std::string out_;
        std::string dirname_;
        int counter_;
        bool save_;
    };

    std::thread start_debug_saver(queue_pointer_type in,std::string debug_dir)
    {
        std::shared_ptr<DebugSaver> p(new DebugSaver(in,debug_dir));
        return std::thread([=]() { p->run(); });
    }

}
