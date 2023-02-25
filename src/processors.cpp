#include "data_items.h"
#include "rotation.h"
#include "stacker.h"
#include "tiffmat.h"
#include <booster/log.h>
#include "processors.h"
#include <cppcms/json.h>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iomanip>
#include <chrono>
#include "util.h"

namespace ols {
    class PreProcessor {
    public:
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
                    if(handle_video(video_ptr))
                        out_->push(data_ptr);
                    continue;
                }
                auto config_ptr = std::dynamic_pointer_cast<StackerControl>(data_ptr);
                if(config_ptr) {
                    handle_config(config_ptr);
                }
                out_->push(data_ptr);
            }
        }
        bool handle_video(std::shared_ptr<CameraFrame> video)
        {
            if(video->frame.rows != height_ || video->frame.cols != width_) {
                BOOSTER_ERROR("stacker") << "Invalid mat frame size, expecting " << height_ << "x" << width_ << " got " << video->frame.rows<< "x"<<video->frame.cols;
                return false;
            }
            video->frame.convertTo(video->processed_frame,CV_32FC3,1/(255.0));
            if(calibration_)
                return true;
            if(gamma_ != 1.0)
                cv::pow(video->processed_frame,gamma_,video->processed_frame);
            if(apply_darks_) {
                video->processed_frame = cv::max(0,video->processed_frame - darks_);
            }
            if(apply_flats_) {
                video->processed_frame = video->processed_frame.mul(flats_);
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
        void load_flats(std::string flats_path,std::string dark_flats_path)
        {
            try {
                bool using_dark_flats = false;
                cv::Mat flats = load_tiff(flats_path);
                if(!dark_flats_path.empty()) {
                    cv::Mat dark_flats = load_tiff(dark_flats_path);
                    flats = cv::max(1e-16f,flats - dark_flats);
                    using_dark_flats = true;
                }
                cv::Mat gray_flats;
                cv::cvtColor(flats,gray_flats,cv::COLOR_BGR2GRAY);
                double minV,maxV;
                cv::minMaxLoc(gray_flats,&minV,&maxV);
                gray_flats = maxV/gray_flats;
                cv::cvtColor(gray_flats,flats,cv::COLOR_GRAY2BGR);
                flats_ = flats;
                if(flats_.rows == height_ && flats_.cols == width_) {
                    apply_flats_ = true;
                    BOOSTER_INFO("stacker") << "Using flats from " << flats_path << ( using_dark_flats ? (" with dark flats " + dark_flats_path) : " without dark flats");
                }
                else {
                    BOOSTER_ERROR("stacker") << "Failed to load flats from " << flats_path << ": size mistmatch";
                    apply_flats_ = false;
                }
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
                if(darks_.rows == height_ && darks_.cols == width_) {
                    apply_darks_ = true;
                    cv::Mat tmp;
                    if(gamma_ != 1.0) {
                        cv::pow(darks_,gamma_,tmp);
                        darks_ = tmp;
                    }
                }
                else {
                    apply_darks_ = false;
                    BOOSTER_ERROR("stacker") << "Failed to load darks from " << darks_path << ": size mistmatch";
                }
            }
            catch(std::exception const &e) {
                BOOSTER_ERROR("stacker") << "Failed to load darks from " << darks_path << ": " << e.what();
                apply_darks_ = false;
            }
        }

        queue_pointer_type in_,out_;
        int width_,height_;
        bool calibration_;
        float gamma_;
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
        StackerProcessor(queue_pointer_type in,queue_pointer_type out,std::string data_dir) :
            in_(in),
            out_(out),
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
                    auto res = handle_video(video_ptr);
                    if(res) {
                        if(out_)
                            out_->push(res);
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

        std::shared_ptr<CameraFrame> generate_output_frame(std::pair<cv::Mat,StretchInfo> data)
        {
            cv::Mat img = data.first;
            save_stretch(data.second);
            cv::Mat img8;
            img.convertTo(img8,CV_8UC3,255);
            std::shared_ptr<CameraFrame> frame(new CameraFrame());
            frame->format.width = img8.cols;
            frame->format.height = img8.rows;
            std::vector<unsigned char> buf;
            cv::imencode(".jpeg",img8,buf);
            frame->jpeg_frame = std::shared_ptr<VideoFrame>(new VideoFrame(buf.data(),buf.size()));
            return frame;
        }

        void send_updated_image()
        {
            if(out_) {
                out_->push(generate_output_frame(stacker_->get_stacked_image()));
            }
        }

        cv::Mat to16bit(cv::Mat m)
        {
            cv::Mat m2 = cv::max(0,m);
            double max_v;
            cv::minMaxLoc(m2,nullptr,&max_v);
            cv::Mat res;
            m2.convertTo(res,CV_16UC3,65535/max_v);
            return res;
        }

        void save_stacked_image_and_send(bool final_image)
        {
            std::string path = output_path_;
            std::string ipath = output_path_;
            std::string tpath = output_path_;
            if(final_image) {
                path += "_stacked.jpeg";
                ipath += "_stacked.txt";
                tpath += "_stacked.tiff";
            }
            else {
                std::string suffix = "_interm_" + std::to_string(stacker_->stacked_count());
                path += suffix+ ".jpeg";    
                ipath += suffix + ".txt";
                tpath += suffix + "_stacked.tiff";
            }
            save_tiff(to16bit(stacker_->get_raw_stacked_image()),tpath);
            auto frame = generate_output_frame(stacker_->get_stacked_image());
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


        std::shared_ptr<CameraFrame> handle_video(std::shared_ptr<CameraFrame> video)
        {
            std::shared_ptr<CameraFrame> res;
		    auto start = std::chrono::high_resolution_clock::now();
            try {
                if(calibration_) {
                    cframe_ +=  video->processed_frame;
                    cframe_count_ ++;
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
                        double gtime = 0;
                        if(out_) {
                            res = generate_output_frame(stacker_->get_stacked_image());
                            auto p2 = std::chrono::high_resolution_clock::now();
                            gtime = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(p2-p1).count();
                        }
                        BOOSTER_INFO("stacker") << "Stacking took " << (1e3*time) << " ms, generation " << (1e3*gtime) << " ms";
                    }
                    else {
                        BOOSTER_INFO("stacker") << "Failed to stack frame";
                        auto end = std::chrono::high_resolution_clock::now();
                        double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(end-start).count();
                        BOOSTER_INFO("stacker") << "Stacking took " << (1e3*time) << " ms";
                    }
                }
            }
            catch(std::exception const &e) {
                BOOSTER_ERROR("stacker") << "Stacking Failed:" << e.what();
            }
            return res;
        }
        void save_calibration()
        {
            double factor = 1.0 / cframe_count_;
            cv::Mat calib = cframe_.mul(cv::Scalar(factor,factor,factor)); 
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
                calibration_ = ctl->calibration;
                output_path_ = ctl->output_path;
                name_ = ctl->name;
                stacker_.reset();
                if(calibration_) {
                    cframe_ = cv::Mat(height_,width_,CV_32FC3);
                    cframe_count_ = 0;
                }
                else {
                    stacker_.reset(new Stacker(width_,height_));
                    stacker_->set_stretch(ctl->auto_stretch,ctl->stretch_low,ctl->stretch_high,ctl->stretch_gamma);
                    restart_ = true;
                }
                break;
            case StackerControl::ctl_pause:
                restart_ = true;
                break;
            case StackerControl::ctl_save_and_continue:
                if(stacker_)
                    save_stacked_image_and_send(false);
                break;
            case StackerControl::ctl_save:
                if(stacker_) {
                    save_stacked_image_and_send(true);
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
        queue_pointer_type in_,out_;
        std::string data_dir_;
        int width_,height_;
        bool calibration_=false;
        std::string output_path_,name_;
        cv::Mat cframe_;
        int cframe_count_;
        std::unique_ptr<Stacker> stacker_;
        bool restart_;
    };

    std::thread start_stacker(queue_pointer_type in,queue_pointer_type out,std::string data_dir)
    {
        std::shared_ptr<StackerProcessor> p(new StackerProcessor(in,out,data_dir));
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
