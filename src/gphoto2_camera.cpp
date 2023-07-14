#include "camera.h"
#include <gphoto2/gphoto2.h>
#include <mutex>
#include <sstream>
#include <iostream>
#include <atomic>
#include <map>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <thread>
#include <algorithm>

namespace ols {

    void set_error(CamErrorCode &e,int err)
    {
        e = CamErrorCode(gp_result_as_string(err));
    }
    
    class GP2Camera : public Camera {
    public:
        /// for Android
        GP2Camera(Camera *c,GPContext *ctx):
            cam_(c),ctx_(ctx)
        {
            stream_active_ = 0;
        }
        virtual ~GP2Camera()
        {
            CamErrorCode e;
            stop_stream(e);
            if(e) {
                fprintf(stderr,"Failed to close stream %s\n",e.message().c_str());
            }
            GP2CloseCamera(info_.CameraID);
        }
        /// Camera name
        virtual std::string name(CamErrorCode &) 
        {
            return info_.Name;
        }
        /// Return list of suppored video formats
        virtual std::vector<CamStreamFormat> formats(CamErrorCode &)
        {
            std::vector<CamStreamFormat> res;
            std::vector<GP2_IMG_TYPE> video_formats;
            for(unsigned i=0;info_.SupportedVideoFormat[i] != GP2_IMG_END && i<sizeof(info_.SupportedVideoFormat)/sizeof(info_.SupportedVideoFormat[0]);i++) {
                video_formats.push_back(info_.SupportedVideoFormat[i]);
            }
            std::stable_sort(video_formats.begin(),video_formats.end(),[](GP2_IMG_TYPE a,GP2_IMG_TYPE b) {
                int ind_a = a == GP2_IMG_RAW16 ? 0 : 1;
                int ind_b = b == GP2_IMG_RAW16 ? 0 : 1;
                return ind_a<ind_b;
            });
            for(auto video_format: video_formats) {
                CamStreamFormat fmt;
                switch(video_format) {
                case GP2_IMG_RAW8:
                    fmt.format = info_.IsColorCam ? stream_raw8 : stream_mono8;
                    break;
                case GP2_IMG_RGB24:
                    fmt.format = stream_rgb24;
                    break;
                case GP2_IMG_RAW16:
                    fmt.format = info_.IsColorCam ? stream_raw16 : stream_mono16;
                    break;
                case GP2_IMG_Y8:
                    fmt.format = stream_mono8;
                    break;
                default:
                    continue;
                }
                fmt.framerate = -1;
                for(unsigned j=0;info_.SupportedBins[j]!=0 && j < sizeof(info_.SupportedBins)/sizeof(info_.SupportedBins[0]);j++) {
                    int bin = info_.SupportedBins[j];
                    if(info_.MaxWidth % bin == 0 && info_.MaxHeight % bin == 0) {
                        fmt.width = info_.MaxWidth / bin;
                        fmt.height = info_.MaxHeight / bin;
                        fmt.bin = bin;
                        res.push_back(fmt);
                        for(int scale = 2;scale < 8;scale++) {
                            int newW = (info_.MaxWidth / bin) / scale / 8 * 8;
                            int newH = (info_.MaxHeight / bin) / scale / 8 * 8; // actually can be /2 * 2 but for symmetry of square frames I keep it this way
                            if(std::min(newW,newH) < 480)
                                break;
                            fmt.width = newW;
                            fmt.height = newH;
                            fmt.bin = bin;
                            res.push_back(fmt);
                        }
                    }
                }
            }
            return res;
        }

        void handle_frame(CamStreamFormat format,std::vector<unsigned char> &buf)
        {
            struct timeval tv;
            gettimeofday(&tv,nullptr);
            CamFrame frm;
            frm.unix_timestamp = tv.tv_sec;
            frm.unix_timestamp += tv.tv_usec * 1e-6;
            frm.width = format.width;
            frm.height = format.height;
            frm.data = buf.data();
            frm.data_size = buf.size();
            frm.frame_counter = frame_counter_++;
            frm.format = format.format;
            if((format.format == stream_raw16 || format.format == stream_raw8) && info_.IsColorCam) {
                switch(info_.BayerPattern) {
                case GP2_BAYER_RG : frm.bayer = bayer_rg; break;
                case GP2_BAYER_GR : frm.bayer = bayer_gr; break;
                case GP2_BAYER_BG : frm.bayer = bayer_bg; break;
                case GP2_BAYER_GB : frm.bayer = bayer_gb; break;
                }
            }
            {
                std::unique_lock<std::mutex> guard(lock_);
                if(callback_) {
                    callback_(frm);
                }
            }
        }

        /// Start a video stream with provided callback 
        virtual void start_stream(CamStreamFormat format,frame_callback_type callback,CamErrorCode &e) 
        {
            if(stream_active_ != 0) {
                stop_stream(e);
                if(e)
                    return;
            }
            GP2_IMG_TYPE img_type;
            int bpp=-1;
            switch(format.format) {
            case stream_rgb24:
                img_type = GP2_IMG_RGB24;
                bpp=3;
                break;
            case stream_raw16:
            case stream_mono16:
                img_type = GP2_IMG_RAW16;
                bpp=2;
                break;
            case stream_raw8:
                img_type = GP2_IMG_RAW8;
                bpp=1;
                break;
            case stream_mono8:
                img_type = GP2_IMG_Y8;
                bpp=1;
                break;
            default:
                e = "Invalid format";
                return;
            }
            if(format.width*format.bin > info_.MaxWidth || format.height * format.bin > info_.MaxHeight) {
                e = "Unsupported binning/size combination";
                return;
            }

            int bin = 0;
            for(auto supported_bin : info_.SupportedBins) {
                if(supported_bin == format.bin || supported_bin == 0) {
                    bin = supported_bin;
                    break;
                }
            }
            if(bin <=0) {
                e = "Unsupported image binning " + std::to_string(format.bin);
                return;
            }

            GP2_ERROR_CODE code  = GP2SetROIFormat(info_.CameraID, format.width, format.height, bin, img_type);
            if(code != GP2_SUCCESS) {
                std::ostringstream ss;
                ss << "Failed to set ROI of " << format.width << "x" << format.height;
                e = make_message(ss.str(),code);
                return;
            }
            code = GP2StartVideoCapture(info_.CameraID);
            if(code != GP2_SUCCESS) {
                e = make_message("Failed to start video",code);
                return;
            }
            {
                std::unique_lock<std::mutex> guard(lock_);
                callback_ = callback;
            }
                
            stream_active_ = 1;
            thread_ = std::move(std::thread([=]() {
                std::vector<unsigned char> buf(format.width*format.height*bpp);
                while(true) {
                    GP2_ERROR_CODE status = GP2GetVideoData(info_.CameraID,buf.data(),buf.size(),500); 
                    if(status == GP2_SUCCESS) {
                        handle_frame(format,buf);
                    }
                    else if(status == GP2_ERROR_TIMEOUT) {
                        if(stream_active_ == 0)
                            break;
                        continue;
                    }
                    else {
                        break;
                    }
                }
            }));
            
        }

        /// stop the stream - once function ends callback will not be called any more
        virtual void stop_stream(CamErrorCode &e)
        {
            if(stream_active_ == 0)
                return;
            {
                std::unique_lock<std::mutex> guard(lock_);
                callback_ = nullptr;
            }
            GP2_ERROR_CODE code = GP2StopVideoCapture(info_.CameraID);
            if(code != GP2_SUCCESS) {
                e = make_message("Failed to stop stream",code);
                return;
            }
            stream_active_=0;
            thread_.join();
        }

        /// list of camera controls that the camera supports
        virtual std::vector<CamOptionId> supported_options(CamErrorCode &e)
        {
            std::vector<CamOptionId> opts;
            int N=0;
            GP2_ERROR_CODE code = GP2GetNumOfControls(info_.CameraID,&N);
            if(code) {
                e=make_message("Failed to get controls",code);
                return opts;
            }
            bool auto_exp=false,auto_wb=false;
            for(int i=0;i<N;i++) {
                GP2_CONTROL_CAPS cap;
                code = GP2GetControlCaps(info_.CameraID,i,&cap);
                if(code) {
                    e = make_message("Failed to get control info",code);
                    return opts;
                }


                std::cerr << cap.Name << " min=" << cap.MinValue << " max=" << cap.MaxValue << " def=" << cap.DefaultValue << " auto=" << cap.IsAutoSupported << " readonly=" << !cap.IsWritable<<" :" << cap.Description<< std::endl;
                CamOptionId opt_id = opt_count;
                switch(cap.ControlType) {
                case GP2_GAIN:
                case GP2_EXPOSURE:
                    if(cap.IsAutoSupported)
                        auto_exp=true;
                    break;
                case GP2_WB_R:
                case GP2_WB_B:
                    if(cap.IsAutoSupported)
                        auto_wb=true;
                    break;
                default:
                    ;
                }
                switch(cap.ControlType) {
                case GP2_GAIN: opt_id = opt_gain; break;
                case GP2_EXPOSURE: opt_id = opt_exp; break;
                case GP2_WB_R: opt_id = opt_wb_r; break;
                case GP2_WB_B: opt_id = opt_wb_b; break;
                case GP2_TEMPERATURE: opt_id = opt_temperature; break;
                case GP2_TARGET_TEMP: opt_id = opt_cooler_target; break;
                case GP2_COOLER_ON: opt_id = opt_cooler_on; break;
                case GP2_FAN_ON: opt_id = opt_fan_on; break;
                case GP2_COOLER_POWER_PERC: opt_id = opt_cooler_power_perc; break;
                default:
                    continue;
                }
                opts.push_back(opt_id);
                ops_map_[opt_id] = cap;
            }
            if(auto_exp) {
                opts.insert(opts.begin(),opt_auto_exp);
                ops_map_[opt_auto_exp] = GP2_CONTROL_CAPS();
            }
            if(auto_wb) {
                opts.insert(opts.begin(),opt_auto_wb);
                ops_map_[opt_auto_wb] = GP2_CONTROL_CAPS();
            }

            return opts;
        }
        /// get camera control
        virtual CamParam get_parameter(CamOptionId id,bool /*current_only*/,CamErrorCode &e) 
        {
            CamParam r;
            auto p = ops_map_.find(id);
            if(p==ops_map_.end()) {
                e="Unsupported option";
                return r;
            }
            auto cap = p->second;
            memset(&r,0,sizeof(r));
            r.option = id;
            long val;
            GP2_BOOL auto_val;
            if(id == opt_auto_exp || id==opt_auto_wb) {
                GP2_ERROR_CODE code = GP2GetControlValue(info_.CameraID,id == opt_auto_exp ? GP2_EXPOSURE : GP2_WB_R,&val,&auto_val);
                if(code) {
                    e = make_message("Failed to get control value",code);
                    return r;
                }
                r.type = type_bool;
                r.min_val = 0;
                r.max_val = 1;
                r.step_size = 1;
                r.cur_val = auto_val ? 1 : 0;
                r.def_val = r.cur_val; // no way to query if auto is default
                return r;
            }
            GP2_ERROR_CODE code = GP2GetControlValue(info_.CameraID,cap.ControlType,&val,&auto_val);
            if(code) {
                e = make_message("Failed to get control value",code);
                return r;
            }
            switch(id) {
            case opt_exp:
                {
                    r.type = type_msec;
                    r.min_val = cap.MinValue * 1e-3; 
                    r.max_val = cap.MaxValue * 1e-3; 
                    r.def_val = cap.DefaultValue * 1e-3;
                    r.cur_val = val * 1e-3;
                    r.step_size = 1e-3;
                }
                break;
            case opt_gain:
            case opt_wb_r:
            case opt_wb_b:
                {
                    r.type = type_number;
                    r.min_val = cap.MinValue;
                    r.max_val = cap.MaxValue;
                    r.def_val = cap.DefaultValue;
                    r.cur_val = val;
                    r.step_size = 1;
                }
                break;
            case opt_temperature:
                {
                    r.type = type_celsius;
                    r.read_only = !cap.IsWritable;
                    r.min_val = cap.MinValue * 0.1;
                    r.max_val = cap.MaxValue * 0.1;
                    r.def_val = cap.DefaultValue * 0.1;
                    r.cur_val = val * 0.1;
                    r.step_size = 0.1;
                }
                break;
            case opt_cooler_target:
                {
                    r.type = type_celsius;
                    r.min_val = cap.MinValue;
                    r.max_val = cap.MaxValue;
                    r.def_val = cap.DefaultValue;
                    r.cur_val = val;
                    r.step_size = 1;
                }
                break;
            case opt_cooler_on:
            case opt_fan_on:
                {
                    bool is_boolean = cap.MinValue ==0 && cap.MaxValue == 1;
                    r.type = is_boolean ? type_bool : type_number;
                    r.min_val = cap.MinValue;
                    r.max_val = cap.MaxValue;
                    r.def_val = cap.DefaultValue;
                    r.cur_val = val;
                    r.step_size = 1;
                }
                break;
            case opt_cooler_power_perc:
                {
                    r.type = type_percent;
                    r.min_val = cap.MinValue;
                    r.max_val = cap.MaxValue;
                    r.def_val = cap.DefaultValue;
                    r.cur_val = val;
                    r.step_size = 1;
                    r.read_only = !cap.IsWritable;
                }
                break;
            default:
                e = "Internal error can't get control";
                return r;
            }
            return r;
        }
        /// set camera control
        virtual void set_parameter(CamOptionId id,double value,CamErrorCode &e)
        {
            GP2_ERROR_CODE code;
            GP2_BOOL auto_value;

            auto p = ops_map_.find(id);
            if(p==ops_map_.end()) {
                e="Unsupported option";
                return;
            }
            auto cap = p->second;
            
            switch(id) {
            case opt_auto_exp:
                {
                    long exp_val,gain_val;
                    GP2_BOOL aval = value ? GP2_TRUE : GP2_FALSE;
                    if( (code = GP2GetControlValue(info_.CameraID,GP2_EXPOSURE,&exp_val,&auto_value))
                        || (code = GP2GetControlValue(info_.CameraID,GP2_GAIN,&gain_val,&auto_value))
                        || (code = GP2SetControlValue(info_.CameraID,GP2_EXPOSURE,exp_val,aval))
                        || (code = GP2SetControlValue(info_.CameraID,GP2_GAIN,gain_val,aval)))
                    {
                        break;
                    }
                }
                break;
            case opt_auto_wb:
                {
                    long r_val,b_val;
                    GP2_BOOL aval = value ? GP2_TRUE : GP2_FALSE;
                    if( (code = GP2GetControlValue(info_.CameraID,GP2_WB_R,&r_val,&auto_value))
                        || (code = GP2GetControlValue(info_.CameraID,GP2_WB_B,&b_val,&auto_value))
                        || (code = GP2SetControlValue(info_.CameraID,GP2_WB_R,r_val,aval))
                        || (code = GP2SetControlValue(info_.CameraID,GP2_WB_B,b_val,aval)))
                    {
                        break;
                    }
                }
                break;
            case opt_exp:
                code = GP2SetControlValue(info_.CameraID,GP2_EXPOSURE,long(value*1000),GP2_FALSE);
                break;
            case opt_gain:
            case opt_wb_r:
            case opt_wb_b:
            case opt_cooler_target:
            case opt_cooler_on:
            case opt_fan_on:
            case opt_cooler_power_perc:
                code = GP2SetControlValue(info_.CameraID,cap.ControlType,long(value),GP2_FALSE);
                break;
            case opt_temperature:
                e = "Temperature is read only";
                return;
            default:
                e = "Unimplemented";
                return;
            }
            if(code) {
                e = make_message("Failed to set option", code);
                return;
            }
        }
    private:
        GP2_CAMERA_INFO info_;
        std::map<CamOptionId,GP2_CONTROL_CAPS> ops_map_;
        std::atomic<int> stream_active_;
        int frame_counter_ = 0;
        // protected by mutex
        std::mutex lock_;
        frame_callback_type callback_; 
        std::thread thread_;
    };
 

    extern "C" {
        static ctx_error_func(GPContext *context, char const *msg, void *)
        {
            fprintf(stderr,"GP2 Error:%s\n",msg);
            fflush(stderr);
        }
        static ctx_status_func(GPContext *context, char const *msg, void *)
        {
            fprintf(stderr,"GP2 Status:%s\n",msg);
            fflush(stderr);
        }
    }

    class GP2CameraDriver : public CameraDriver {
    public:
        virtual std::vector<std::string> list_cameras(CamErrorCode &e) 
        {
            if(!camera_list_.empty())
                return camera_list_;
            gp_camera_init(&camera_);
            int ret = gp_camera_init(camera_,ctx_);
            if(ret != GP_OK) {
                set_error(e,ret);
                gp_camera_free(camera_);
                camera_ = nullptr;
                return camera_list_;
            }
            camera_list_.push_back("Camera");
            return camera_list_;
        }
        virtual std::unique_ptr<Camera> open_camera(int id,CamErrorCode &e) 
        {
            if(size_t(id) >= camera_list_.size()) {
                e = "No such camera " + std::to_string(id);
                return std::unique_ptr<Camera>();
            }
            std::unique_ptr<Camera> cam(new GP2Camera(camera_,ctx_));
            return cam;
        }
        GP2CameraDriver() 
        {
            ctx_ = gp_context_new();
            gp_context_set_error_func(ctx_,ctx_error_func,nullptr);
            gp_context_set_status_func(ctx_,ctx_status_func,nullptr);
        }
        ~GP2CameraDriver()
        {
            if(camera_)
                gp_camera_exit(camera_,ctx_);
        }
    private:
        GPContext *ctx_ = nullptr;
        Camera *camera_ = nullptr;
        bool has_camera_ = false;
        std::vector<std::string> camera_list_;        
    };
}

extern "C" {
    ols::CameraDriver *ols_get_gphoto2_driver(int cam_id = -1)
    {
        ols::CamErrorCode e;
        std::unique_ptr<ols::CameraDriver> p;
        //if(cam_id != -1)
        //    p.reset(new ols::SingleGP2CameraDriver(cam_id,e));
        //else
            p.reset(new ols::GP2CameraDriver());
        if(e) {
            p.reset();
        }
        return p.release();
    }
}
