#include "camera.h"
#include <mutex>
#include <sstream>
#include <iostream>
#include <atomic>
#include <map>
#include <string.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>
#include <thread>
#include <fstream>
#include <regex>
#include "util.h"
#include "tiffmat.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <booster/log.h>
#include <booster/regex.h>
#include <cppcms/json.h>

#ifdef WITH_LIBRAW
#include "libraw_wrapper.h"
#endif

#ifdef WITH_CFITSIO
#include "fitsmat.h"
#endif

namespace ols {
    class WDIRError : public CamError {
    public:
        WDIRError(std::string const &msg) : CamError(msg)
        {
        }
    };




    class WDIRCamera : public Camera {
    public:
        WDIRCamera(std::string const &dir,int width,int height,CamStreamType stream,CamBayerType bayer = bayer_na) 
            : dir_(dir),
              width_(width),
              height_(height),
              stream_(stream),
              bayer_(bayer)
        {
            stream_active_ = 0;
            switch(stream_) {
            case stream_mono16:
            case stream_mono8:
                bpp_ = stream_ == stream_mono8 ? 8 : 16;
                mono_ = true;
                raw_ = false;
                break;
            case stream_rgb24:
            case stream_rgb48:
                bpp_ = stream_ == stream_rgb24 ? 8 : 16;
                mono_ = false;
                raw_ = false;
                break;
            case stream_raw16:
                bpp_ = 16;
                mono_ = false;
                raw_ = true;
                break;
            default:
                throw WDIRError("Unsupported stream format " + stream_type_to_str(stream));
            }
        }

        virtual ~WDIRCamera()
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
            return "Directory Watcher";
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

        bool ends_with(std::string const &name,char const *pattern)
        {
            int N = strlen(pattern);
            int pos = int(name.size()) - N;
            if(pos < 0)
                return false;
            if(strcasecmp(name.c_str() + pos,pattern) == 0)
                return true;
            return false;
        }

        void handle_frame(std::string const &fname)
        {
            CamFrame frm;
            frm.unix_timestamp = timestamp();
            frm.bayer = bayer_;
            struct stat st;
            if(stat(fname.c_str(),&st)!=0) {
                BOOSTER_INFO("stacker") << "Failed to stat " << fname;
                return;
            }
            if(!(st.st_mode & S_IFREG)) {
                BOOSTER_INFO("stacker") << "Not regular file, skipping:" << fname;
                return;
            }

            cv::Mat img;
            if(ends_with(fname,".tiff") || ends_with(fname,".tif"))
                img = load_tiff(fname);
#ifdef WITH_LIBRAW
            else if(ends_with(fname,".dng") || ends_with(fname,".raw")) {
                auto res = load_libraw(fname);
                img = res.first; 
                frm.bayer  = res.second;
            }
#endif 
#ifdef WITH_CFITSIO
            else if(ends_with(fname,".fits")) {
                auto res = load_fits(fname);
                img = res.first; 
                frm.bayer  = res.second;
            }
#endif                                       
            else
                img = cv::imread(fname);
            if(img.cols != width_ || img.rows != height_) {
                BOOSTER_INFO("stacker") << "Image size is not correct got " << img.rows<<"x"<<img.cols << " expecting " << height_<<"x"<<width_;
                return;
            }
            if(int(img.elemSize() / img.elemSize1()) != ((mono_ || raw_) ? 1 : 3)) {
                BOOSTER_INFO("stack") << "file color space invalid, skpiing: " << fname;
                return;
            }
            if(int(img.elemSize1()) != (bpp_ + 7) / 8) {
                BOOSTER_INFO("stack") << "file depth invalid, skippng: " << fname;
                return;
            }
            if(raw_ && frm.bayer == bayer_na) {
                BOOSTER_INFO("stack") << "No bayer pattern info for raw image, skipping: " << fname;
                return;
            }
            frm.data = img.data;
            frm.data_size = img.rows * img.cols * img.elemSize();
            frm.height = height_;
            frm.width = width_;
            frm.format = stream_;
            frm.frame_counter = frame_counter_++;

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
                if(format.format != stream_ || format.width != width_ || format.height != height_) {
                    std::ostringstream ss;
                    ss << "Invalid format, got: " << stream_type_to_str(format.format) <<"@"<<format.width << "x"<<format.height << " expected: "
                        <<stream_type_to_str(stream_)<<"@"<<width_<<"x"<<height_;
                    throw WDIRError(ss.str());
                }
                {
                    std::unique_lock<std::mutex> guard(lock_);
                    callback_ = callback;
                }
                stream_active_ = 1;
                wd_.reset(new DirWatch(dir_));
                thread_ = std::move(std::thread([=]() {
                    while(stream_active_ != 0) {
                        std::string fname;
                        try {
                            fname = wd_->wait_for_new_file(500);
                        }
                        catch(std::exception const &e) {
                            BOOSTER_ERROR("stacker") << "Failed to wait:" << e.what();
                            return;
                        }
                        if(!fname.empty()) {
                            try {
                                handle_frame(fname);
                            }
                            catch(std::exception const &e) {
                                BOOSTER_ERROR("stacker") << "Failed to read:" << fname << " ->" << e.what();
                            }
                            if(!keep_images_)
                                std::remove(fname.c_str());
                        }
                        if(stream_active_ == 0)
                            break;
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
                wd_.reset();
            }
            catch(std::exception const &err) {
                e = CamErrorCode(err);
                wd_.reset();
            }
        }

        /// list of camera controls that the camera supports
        virtual std::vector<CamOptionId> supported_options(CamErrorCode &)
        {
            std::vector<CamOptionId> opts = {opt_keep_images};
            return opts;
        }
        /// get camera control
        virtual CamParam get_parameter(CamOptionId id,bool /*current_only*/,CamErrorCode &e) 
        {
            CamParam r;
            memset(&r,0,sizeof(r));
            r.option = id;
            if(id == opt_keep_images) {
                r.option = id;
                r.type = type_bool;
                r.min_val = 0;
                r.max_val = 1;
                r.step_size = 1;
                r.cur_val = keep_images_;
                r.def_val = 0;
            }
            else {
                e=CamErrorCode("Unimplemented: " + cam_option_id_to_name(id));
            }
            return r;
        }
        /// set camera control
        virtual void set_parameter(CamOptionId id,double value,CamErrorCode &e)
        {
            if(id == opt_keep_images) {
                keep_images_ = !!value;
            }
            else {
                e = "Unimplemented" +  cam_option_id_to_name(id);
            }
        }
    private:
        std::string dir_;
        int width_,height_;
        CamStreamType stream_;
        CamBayerType bayer_;
        bool keep_images_ = false;

        int bpp_;
        bool mono_;
        bool raw_;

        std::atomic<int> stream_active_;
        int frame_counter_ = 0;
        // protected by mutex
        std::mutex lock_;
        frame_callback_type callback_; 
        std::thread thread_;
        std::unique_ptr<DirWatch> wd_;
    };
 


    class WDIRCameraDriver : public CameraDriver {
    public:
        virtual std::vector<std::string> list_cameras(CamErrorCode &) 
        {
            return {"Directory Watcher"};
        }
        void parse_params(std::string &dir,CamStreamType &stream,CamBayerType &bayer,int &width,int &height)
        {
            cppcms::json::value v;
            std::istringstream ss(watch_info);
            if(!v.load(ss,true))
                throw WDIRError("Parsing of inputs failed");
            dir = v.get<std::string>("path");
            stream  = stream_type_from_str(v.get<std::string>("format"));
            bayer = bayer_type_from_str(v.get<std::string>("bayer","NA"));
            width = v.get<int>("width");
            height = v.get<int>("height");
        }
        virtual std::unique_ptr<Camera> open_camera(int id,CamErrorCode &e) 
        {
            try {
                if(id!=0)
                    throw WDIRError("No such camera " + std::to_string(id));
                int width,height;
                std::string dir;
                CamStreamType stream;
                CamBayerType bayer;

                parse_params(dir,stream,bayer,width,height);
                std::unique_ptr<Camera> cam(new WDIRCamera(dir,width,height,stream,bayer));
                return cam;
            }
            catch(std::exception const &err) {
                e=CamErrorCode(err);
                return std::unique_ptr<Camera>();
            }
        }
        static std::string watch_info;
    };
    std::string WDIRCameraDriver::watch_info;
}

extern "C" {
    int ols_set_wdir_driver_config(char const *str)
    {
        ols::WDIRCameraDriver::watch_info = str;
        return 0;
    }
    ols::CameraDriver *ols_get_wdir_driver(int )
    {
        return new ols::WDIRCameraDriver();
    }
}
