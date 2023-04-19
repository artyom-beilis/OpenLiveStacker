#include "camera.h"
#include <mutex>
#include <sstream>
#include <iostream>
#include <atomic>
#include <map>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <thread>
#include <fstream>
#include "util.h"
#include "tiffmat.h"
#include <opencv2/imgcodecs.hpp>

namespace ols {
    class SIMError : public CamError {
    public:
        SIMError(std::string const &msg) : CamError(msg)
        {
        }
    };
    class SIMCamera : public Camera {
    public:
        SIMCamera(std::string const &dir) : dir_(dir)
        {
            stream_active_ = 0;
            load_frames();
        }

        std::string file_name(int frame_id,std::string const &ext)
        {
            char name[256];
            snprintf(name,sizeof(name),"/frame_%08d.%s",frame_id,ext.c_str());
            return dir_ + name;
        }

        void load_frames()
        {
            std::ifstream data(dir_  + "/log.txt");
            if(!data)
                throw SIMError("Failed to read log file");
            std::string str;
            while(std::getline(data,str)) {
                size_t pos = str.find(',');
                std::string op = str.substr(0,pos);
                if(op == "PAUSE") {
                    continue;
                }
                int frame_id = atoi(op.c_str());
                double timestamp = atof(str.substr(pos+1).c_str());
                if(frames_.empty()) {
                    is_jpeg_ = exists(file_name(frame_id,"jpeg"));
                }
                std::string path = file_name(frame_id,is_jpeg_ ? "jpeg" : "tiff");
                if(frames_.empty()) {
                    cv::Mat img;
                    if(is_jpeg_)
                        img = cv::imread(path);
                    else
                        img = load_tiff(path);
                    width_ = img.cols;
                    height_ = img.rows;
                    if(is_jpeg_)
                        stream_ = stream_mjpeg;
                    else if(img.elemSize1() == 1 && img.channels() == 3)
                        stream_ = stream_rgb24;
                    else if(img.elemSize1() == 2 && img.channels() == 3)
                        stream_ = stream_rgb48;
                    else
                        throw SIMError("Unsupported image format for " + path);
                }
                frames_.push_back(std::make_pair(timestamp,path));
            }
        }

        virtual ~SIMCamera()
        {
            try {
                CamErrorCode e;
                stop_stream(e);
                e.check();
            }
            catch(std::exception const &e) {
                fprintf(stderr,"Failed to close stream %s\n",e.what());
            }
            catch(...) {
                fprintf(stderr,"Failed to close stream\n");
            }
        }
        /// Camera name
        virtual std::string name(CamErrorCode &) 
        {
            return "Simulation";
        }
        /// Return list of suppored video formats
        virtual std::vector<CamStreamFormat> formats(CamErrorCode &)
        {
            std::vector<CamStreamFormat> res;
            CamStreamFormat fmt;
            fmt.format = stream_;
            fmt.width = width_;
            fmt.height = height_;
            fmt.framerate = -1;
            res.push_back(fmt);
            return res;
        }

        void handle_frame(int index)
        {
            CamFrame frm;
            frm.unix_timestamp = frames_.at(index).first;
            frm.width = width_;
            frm.height = height_;
            std::vector<char> buf;
            cv::Mat img;
            if(is_jpeg_) {
                std::ifstream f(frames_.at(index).second,std::ifstream::binary);
                f.seekg(0,std::ifstream::end);
                size_t size = f.tellg();
                buf.resize(size);
                f.seekg(0);
                f.read(buf.data(),size);
                frm.data = buf.data();
                frm.data_size = size;
            }
            else {
                img = load_tiff(frames_.at(index).second);
                frm.data = img.data;
                frm.data_size = img.rows * img.cols * img.elemSize();
            }

            frm.frame_counter = frame_counter_;
            frm.format = stream_;
            {
                std::unique_lock<std::mutex> guard(lock_);
                if(callback_) {
                    try {
                        callback_(frm);
                    }
                    catch(std::exception const &e) {
                        fprintf(stderr,"Exception in callback %s\n",e.what());
                    }
                    catch(...) {
                        fprintf(stderr,"Unknown exception in callback\n");
                    }
                }
            }
        }

        double timestamp()
        {
            struct timeval tv;
            gettimeofday(&tv,nullptr);
            double unix_timestamp = tv.tv_sec;
            unix_timestamp += tv.tv_usec * 1e-6;
            return unix_timestamp;
        }

        /// Start a video stream with provided callback 
        virtual void start_stream(CamStreamFormat format,frame_callback_type callback,CamErrorCode &e) 
        {
            try {
                if(stream_active_ != 0) {
                    stop_stream(e);
                    e.check();
                }
                if(format.format != stream_ || format.width != width_ || format.height != height_)
                    throw SIMError("Invalid format");
                {
                    std::unique_lock<std::mutex> guard(lock_);
                    callback_ = callback;
                }
                stream_active_ = 1;
                thread_ = std::move(std::thread([=]() {
                    int size = frames_.size();
                    double next_time = timestamp() + exposure_ * 1e-3;
                    while(stream_active_) {
                        double remains;
                        while((remains = (next_time - timestamp()))>= 1e-3) {
                            double ms = std::min(remains * 1000,500.0);
                            std::this_thread::sleep_for(std::chrono::milliseconds(int(ms)));
                            if(stream_active_ != 1)
                                return;
                        }
                        next_time += exposure_ * 1e-3;
                        handle_frame(current_index_);
                        if(current_dir_ == 1) {
                            if(current_index_ >= size - 1) {
                                current_index_ = std::max(0,current_index_ - 1);
                                current_dir_ = -1;
                            }
                            else {
                                current_index_ ++;
                            }
                        }
                        else {
                            if(current_index_ <= 0) {
                                current_index_ = std::min(size - 1,current_index_ + 1);
                                current_dir_ = +1;
                            }
                            else {
                                current_index_ --;
                            }
                        }
                    }
                }));
            }
            catch(std::exception const &err) {
                e = CamErrorCode(err);
            }
        }

        /// stop the stream - once function ends callback will not be called any more
        virtual void stop_stream(CamErrorCode &e)
        {
            try {
                if(stream_active_ == 0)
                    return;
                {
                    std::unique_lock<std::mutex> guard(lock_);
                    callback_ = nullptr;
                }
                stream_active_=0;
                thread_.join();
            }
            catch(std::exception const &err) {
                e = CamErrorCode(err);
            }
        }

        /// list of camera controls that the camera supports
        virtual std::vector<CamOptionId> supported_options(CamErrorCode &)
        {
            std::vector<CamOptionId> opts = {opt_exp,opt_gamma};
            return opts;
        }
        /// get camera control
        virtual CamParam get_parameter(CamOptionId id,bool /*current_only*/,CamErrorCode &e) 
        {
            CamParam r;
            memset(&r,0,sizeof(r));
            r.option = id;
            switch(id) {
            case opt_exp:
                r.type = type_msec;
                r.min_val = 10;
                r.max_val = 10000;
                r.def_val = 1000;
                r.cur_val = exposure_;
                r.step_size = 1;
                break;
            case opt_gamma:
                r.type = type_number;
                r.min_val = 0.75;
                r.max_val = 3.0;
                r.def_val = 1.0;
                r.cur_val = gamma_;
                r.step_size = 0.01;
                break;
            default:
                e=CamErrorCode("Unimplemented: " + cam_option_id_to_name(id));
            }
            return r;
        }
        /// set camera control
        virtual void set_parameter(CamOptionId id,double value,CamErrorCode &e)
        {
            try {
                switch(id) {
                case opt_exp:
                    if(value < 10 || value > 10000)
                        throw SIMError("Invalid range");
                    exposure_ = value;
                    break;
                case opt_gamma:
                    if(value < 0.75 || value > 3.0)
                        throw SIMError("Invalid range");
                    gamma_ = value;
                    break;
                default:
                    throw SIMError("Unimplemented" +  cam_option_id_to_name(id));
                }
            }
            catch(std::exception const &err) {
                e=CamErrorCode(err);
            }
        }
    private:
        std::string dir_;
        int exposure_ = 1000;
        double gamma_ = 1.0;
        CamStreamType stream_ = stream_mjpeg;
        bool is_jpeg_ = true;
        int width_,height_;

        std::atomic<int> stream_active_;
        int frame_counter_ = 0;
        int current_index_ = 0;
        int current_dir_ = +1;
        // protected by mutex
        std::mutex lock_;
        frame_callback_type callback_; 
        std::thread thread_;
        std::vector<std::pair<double,std::string> > frames_;
    };
 


    class SIMCameraDriver : public CameraDriver {
    public:
        virtual std::vector<std::string> list_cameras(CamErrorCode &) 
        {
            return std::vector<std::string>{"Simulation"};
        }
        virtual std::unique_ptr<Camera> open_camera(int id,CamErrorCode &e) 
        {
            try {
                if(id!=0)
                    throw SIMError("No such camera " + std::to_string(id));
                std::string dir = data_dir;
                char *dir_override = getenv("OLS_SIM_DIR");
                if(dir_override)
                    dir = dir_override;
                std::unique_ptr<Camera> cam(new SIMCamera(dir));
                return cam;
            }
            catch(std::exception const &err) {
                e=CamErrorCode(err);
                return std::unique_ptr<Camera>();
            }
        }
        static std::string data_dir;
    };
    std::string SIMCameraDriver::data_dir = "./sim";
}

extern "C" {
    int ols_set_sim_driver_config(char const *str)
    {
        ols::SIMCameraDriver::data_dir = str;
        return 0;
    }
    ols::CameraDriver *ols_get_sim_driver(int )
    {
        return new ols::SIMCameraDriver();
    }
}
