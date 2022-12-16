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
#include <sys/types.h>
#include <sys/stat.h>

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
                BOOSTER_ERROR("stacker") << "Invalid mat frame size";
                return false;
            }
            video->frame.convertTo(video->processed_frame,CV_32FC3,1/(255.0));
            if(calibration_)
                return true;
            if(gamma_ != 1.0)
                cv::pow(video->processed_frame,gamma_,video->processed_frame);
            if(apply_darks_)
                video->processed_frame -= darks_;
            if(derotator_) {
                if(first_frame_ts_ == 0)
                    first_frame_ts_ = video->timestamp;
                else {
                    double angle = derotator_->getAngleDeg(first_frame_ts_,video->timestamp);
                    if(derotate_mirror_)
                        angle = -angle;
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
                break;
            default:
                /// not much to do
                ;
            }
        }
    private:
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
        bool apply_darks_;
    };

    std::thread start_preprocessor(queue_pointer_type in,queue_pointer_type out)
    {
        std::shared_ptr<PreProcessor> p(new PreProcessor(in,out));
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
                    out_->push(data_ptr);
                    break;
                }
                auto video_ptr = std::dynamic_pointer_cast<CameraFrame>(data_ptr);
                if(video_ptr) {
                    auto res = handle_video(video_ptr);
                    if(res)
                        out_->push(res);
                    continue;
                }
                auto config_ptr = std::dynamic_pointer_cast<StackerControl>(data_ptr);
                if(config_ptr) {
                    handle_config(config_ptr);
                }
                out_->push(data_ptr);
            }
        }

        std::shared_ptr<CameraFrame> generate_output_frame(cv::Mat img)
        {
            std::shared_ptr<CameraFrame> frame(new CameraFrame());
            frame->format.width = img.cols;
            frame->format.height = img.rows;
            std::vector<unsigned char> buf;
            cv::imencode(".jpeg",img,buf);
            frame->jpeg_frame = std::shared_ptr<VideoFrame>(new VideoFrame(buf.data(),buf.size()));
            return frame;
        }

        void send_updated_image()
        {
            out_->push(generate_output_frame(stacker_->get_stacked_image()));
        }


        void save_stacked_image_and_send(bool final_image)
        {
            std::string path = output_path_;
            std::string ipath = output_path_;
            if(final_image) {
                path += "_stacked.jpeg";
                ipath += "_stacked.json";
            }
            else {
                std::string suffix = "_interm_" + std::to_string(stacker_->stacked_count());
                path += suffix+ ".jpeg";    
                ipath += suffix + ".json";
            }
            std::ofstream f(path,std::ofstream::binary);
            auto frame = generate_output_frame(stacker_->get_stacked_image());;
            f.write((char*)frame->jpeg_frame->data(),frame->jpeg_frame->size());
            f.close();
            out_->push(frame);
            std::ofstream log(ipath);
            log << "Object: " << name_ << "\n";
            log << "When: " <<timestamp() << "\n";
            log << "Stacked: " << stacker_->stacked_count() << std::endl;
        }

        std::string timestamp()
        {
            time_t now;
            time(&now);
            char ts[256];
            strftime(ts,sizeof(ts),"%Y-%m-%d %H:%M:%S",localtime(&now));
            return ts;
        }


        std::shared_ptr<CameraFrame> handle_video(std::shared_ptr<CameraFrame> video)
        {
            try {
                if(calibration_) {
                    cframe_ +=  video->processed_frame;
                    cframe_count_ ++;
                    return video;
                }
                if(stacker_->stack_image(video->processed_frame,restart_)) {
                    restart_ = false;
                    return generate_output_frame(stacker_->get_stacked_image());
                }
                else {
                    BOOSTER_INFO("stacker") << "Failed to stack frame";
                }
            }
            catch(std::exception const &e) {
                BOOSTER_ERROR("stacker") << "Stacking Failed";
            }
            return std::shared_ptr<CameraFrame>();
        }
        void save_calibration()
        {
            double factor = 1.0 / cframe_count_;
            cv::Mat calib = cframe_.mul(cv::Scalar(factor,factor,factor)); 
            std::string tiff_path = output_path_ + "/" + name_ + ".tiff";
            std::string db_path = output_path_ + "/index.json";
            cppcms::json::value setup;
            setup["id"] = name_;
            setup["path"] = name_ + ".tiff";
            setup["date"] = timestamp();
            setup["frames"] = cframe_count_;
            save_tiff(calib,tiff_path);
            cppcms::json::value db;
            bool found = false;
            {
                std::ifstream indx(db_path);
                if(!indx) {
                    db[0] = setup;
                    found = true;
                }
                else {
                    if(!db.load(indx,true)) {
                        BOOSTER_ERROR("stacker") << "Error parsing " << db_path << " darks DB file " << std::endl;
                        db = cppcms::json::value();
                        db[0] = setup;
                        found = true;
                    }
                }
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
                if(calibration_) {
                    cframe_ = cv::Mat(height_,width_,CV_32FC3);
                    cframe_count_ = 0;
                }
                else {
                    stacker_.reset(new Stacker(width_,height_));
                    stacker_->set_target_gamma(ctl->auto_stretch ? -1 : ctl->strech_gamma);
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
                    stacker_->set_target_gamma(ctl->auto_stretch ? -1 : ctl->strech_gamma);
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
        int width_,height_;
        bool calibration_=false;
        std::string output_path_,name_;
        cv::Mat cframe_;
        int cframe_count_;
        std::unique_ptr<Stacker> stacker_;
        bool restart_;
    };

    std::thread start_stacker(queue_pointer_type in,queue_pointer_type out)
    {
        std::shared_ptr<StackerProcessor> p(new StackerProcessor(in,out));
        return std::thread([=]() { p->run(); });
    }

    class DebugSaver {
    public:
        DebugSaver(queue_pointer_type in,std::string output_dir) :
            in_(in),
            out_(output_dir),
            counter_(0)
        {
            mkdir(out_.c_str(),0777);
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
                if(video_ptr) {
                    handle_video(video_ptr);
                    continue;
                }
                auto config_ptr = std::dynamic_pointer_cast<StackerControl>(data_ptr);
                if(config_ptr) {
                    handle_config(config_ptr);
                }
            }
        }
        void handle_video(std::shared_ptr<CameraFrame> video)
        {
            char fname[256];
            snprintf(fname,sizeof(fname),"frame_%08d",counter_);
            std::string base_name = dirname_ + "/" + fname;
            if(video->format.format == stream_mjpeg) {
                std::ofstream f(base_name + ".jpeg",std::ofstream::binary);
                f.write((char*)video->source_frame->data(),video->source_frame->size());
                f.close();
            }
            else {
                try {
                    save_tiff(video->frame,base_name + ".tiff");
                }
                catch(std::exception const &e){
                    BOOSTER_ERROR("stacker") << "Failed to saved tiff to " << base_name << ".tiff: " << e.what();
                }
            }
            std::ofstream log(dirname_ + "/log.txt",std::ofstream::app);
            log << counter_ <<"," << std::fixed << std::setprecision(3) << video->timestamp << std::endl;
        }
        void handle_config(std::shared_ptr<StackerControl> ctl)
        {
            switch(ctl->op) {
            case StackerControl::ctl_init:
                {
                    dirname_ = out_ + "/" + ctl->name;
                    counter_ = 0;
                    mkdir(dirname_.c_str(),0777);
                    cppcms::json::value v;
                    v["name"]=ctl->name;
                    v["width"] = ctl->width;
                    v["height"] = ctl->height;
                    v["darks"] = ctl->darks_path;
                    v["calibration"] = ctl->calibration;
                    v["derote"] = ctl->derotate;
                    v["derotate_mirror"] = ctl->derotate_mirror;
                    v["ra"] = ctl->ra;
                    v["de"] = ctl->de;
                    v["lat"] = ctl->lat;
                    v["lon"] = ctl->lon;
                    v["source_gamma"] = ctl->source_gamma;
                    v["auto_stretch"] = ctl->auto_stretch;
                    v["strech_gamma"] = ctl->strech_gamma;
                    v["strech_low"] = ctl->strech_low;
                    v["strech_high"] = ctl->strech_high;
                    std::ofstream info(dirname_ + "/info.json");
                    v.save(info,cppcms::json::readable);
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
    };

    std::thread start_debug_saver(queue_pointer_type in,std::string debug_dir)
    {
        std::shared_ptr<DebugSaver> p(new DebugSaver(in,debug_dir));
        return std::thread([=]() { p->run(); });
    }

}
