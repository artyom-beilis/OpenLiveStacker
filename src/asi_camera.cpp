#include "camera.h"
#include "ASICamera2.h"
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
    static std::string make_message(std::string const &msg,ASI_ERROR_CODE code)
    {
        std::string str;
        switch(code) {
        case ASI_SUCCESS: str="Success"; break;
        case ASI_ERROR_INVALID_INDEX:  str="no camera connected or index value out of boundary"; break; 
        case ASI_ERROR_INVALID_ID:  str="invalid ID"; break; 
        case ASI_ERROR_INVALID_CONTROL_TYPE:  str="invalid control type"; break; 
        case ASI_ERROR_CAMERA_CLOSED:  str="camera didn't open"; break; 
        case ASI_ERROR_CAMERA_REMOVED:  str="failed to find the camera, maybe the camera has been removed"; break; 
        case ASI_ERROR_INVALID_PATH:  str="cannot find the path of the file"; break; 
        case ASI_ERROR_INVALID_FILEFORMAT: str="invalid file format"; break;
        case ASI_ERROR_INVALID_SIZE:  str="wrong video format size"; break; 
        case ASI_ERROR_INVALID_IMGTYPE:  str="unsupported image format"; break; 
        case ASI_ERROR_OUTOF_BOUNDARY:  str="the startpos is out of boundary"; break; 
        case ASI_ERROR_TIMEOUT:  str="timeout"; break; 
        case ASI_ERROR_INVALID_SEQUENCE: str="stop capture first"; break; 
        case ASI_ERROR_BUFFER_TOO_SMALL:  str="buffer size is not big enough"; break; 
        case ASI_ERROR_VIDEO_MODE_ACTIVE: str="video mode active"; break;
        case ASI_ERROR_EXPOSURE_IN_PROGRESS: str="exposure in process"; break;
        case ASI_ERROR_GENERAL_ERROR: str="general error"; break; 
        case ASI_ERROR_INVALID_MODE: str="the current mode is wrong"; break; 
        default:
            str="unknown code";
        };
        return msg + ": " + str;
    }
    
    class ASICamera : public Camera {
    public:
        /// for Android
        ASICamera(int id,CamErrorCode &e)
        {
            stream_active_ = 0;
            ASI_ERROR_CODE code = ASIGetCameraPropertyByID(id,&info_);
            if(code != ASI_SUCCESS) {
                e = make_message("Failed to access camera",code);
                return;
            }
            code = ASIInitCamera(info_.CameraID);
            if(code) {
                e = make_message("Failed to init camera",code);
            }
        }
        ASICamera(ASI_CAMERA_INFO info,CamErrorCode &e) : info_(info)
        {
            stream_active_ = 0;
            ASI_ERROR_CODE code = ASIOpenCamera(info_.CameraID);
            if(code) {
                e = make_message("Failed to open camera",code);
                return;
            }
            code = ASIInitCamera(info_.CameraID);
            if(code) {
                ASICloseCamera(info_.CameraID);
                e = make_message("Failed to init camera",code);
            }
        }
        virtual ~ASICamera()
        {
            CamErrorCode e;
            stop_stream(e);
            if(e) {
                fprintf(stderr,"Failed to close stream %s\n",e.message().c_str());
            }
            ASICloseCamera(info_.CameraID);
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
            std::vector<ASI_IMG_TYPE> video_formats;
            for(unsigned i=0;info_.SupportedVideoFormat[i] != ASI_IMG_END && i<sizeof(info_.SupportedVideoFormat)/sizeof(info_.SupportedVideoFormat[0]);i++) {
                video_formats.push_back(info_.SupportedVideoFormat[i]);
            }
            std::stable_sort(video_formats.begin(),video_formats.end(),[](ASI_IMG_TYPE a,ASI_IMG_TYPE b) {
                int ind_a = a == ASI_IMG_RAW16 ? 0 : 1;
                int ind_b = b == ASI_IMG_RAW16 ? 0 : 1;
                return ind_a<ind_b;
            });
            for(auto video_format: video_formats) {
                CamStreamFormat fmt;
                switch(video_format) {
                case ASI_IMG_RAW8:
                    fmt.format = info_.IsColorCam ? stream_raw8 : stream_mono8;
                    break;
                case ASI_IMG_RGB24:
                    fmt.format = stream_rgb24;
                    break;
                case ASI_IMG_RAW16:
                    fmt.format = info_.IsColorCam ? stream_raw16 : stream_mono16;
                    break;
                case ASI_IMG_Y8:
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
                        fmt.roi_den = 1;
                        fmt.roi_num = 1;
                        res.push_back(fmt);
                        for(int scale = 1;scale < 8;scale++) {
                            int newW,newH;
                            int num,den;
                            if(scale == 1) {
                                if(info_.MaxWidth <= 1280 || info_.MaxHeight <= 960) { // filter ASI120 that has stricter limits
                                    continue;
                                }
                                /// make 2/3s ROI
                                newW = (info_.MaxWidth / bin) * 2 / 3 / 8 * 8;
                                newH = (info_.MaxHeight / bin) * 2 / 3 / 8 * 8;
                                num = 2;
                                den = 3;
                            }
                            else {
                                /// rest 1/N th ROI
                                newW = (info_.MaxWidth / bin) / scale / 8 * 8;
                                newH = (info_.MaxHeight / bin) / scale / 8 * 8; // actually can be /2 * 2 but for symmetry of square frames I keep it this way
                                num = 1;
                                den = scale;
                            }
                            if(std::min(newW,newH) < 480)
                                break;
                            fmt.width = newW;
                            fmt.height = newH;
                            fmt.bin = bin;
                            fmt.roi_num = num;
                            fmt.roi_den = den;
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
                case ASI_BAYER_RG : frm.bayer = bayer_rg; break;
                case ASI_BAYER_GR : frm.bayer = bayer_gr; break;
                case ASI_BAYER_BG : frm.bayer = bayer_bg; break;
                case ASI_BAYER_GB : frm.bayer = bayer_gb; break;
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
            ASI_IMG_TYPE img_type;
            int bpp=-1;
            switch(format.format) {
            case stream_rgb24:
                img_type = ASI_IMG_RGB24;
                bpp=3;
                break;
            case stream_raw16:
            case stream_mono16:
                img_type = ASI_IMG_RAW16;
                bpp=2;
                break;
            case stream_raw8:
                img_type = ASI_IMG_RAW8;
                bpp=1;
                break;
            case stream_mono8:
                img_type = ASI_IMG_Y8;
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

            ASI_ERROR_CODE code  = ASISetROIFormat(info_.CameraID, format.width, format.height, bin, img_type);
            if(code != ASI_SUCCESS) {
                std::ostringstream ss;
                ss << "Failed to set ROI of " << format.width << "x" << format.height;
                e = make_message(ss.str(),code);
                return;
            }
            code = ASIStartVideoCapture(info_.CameraID);
            if(code != ASI_SUCCESS) {
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
                    ASI_ERROR_CODE status = ASIGetVideoData(info_.CameraID,buf.data(),buf.size(),500); 
                    if(status == ASI_SUCCESS) {
                        handle_frame(format,buf);
                    }
                    else if(status == ASI_ERROR_TIMEOUT) {
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
            ASI_ERROR_CODE code = ASIStopVideoCapture(info_.CameraID);
            if(code != ASI_SUCCESS) {
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
            ASI_ERROR_CODE code = ASIGetNumOfControls(info_.CameraID,&N);
            if(code) {
                e=make_message("Failed to get controls",code);
                return opts;
            }
            bool auto_exp=false,auto_wb=false;
            for(int i=0;i<N;i++) {
                ASI_CONTROL_CAPS cap;
                code = ASIGetControlCaps(info_.CameraID,i,&cap);
                if(code) {
                    e = make_message("Failed to get control info",code);
                    return opts;
                }


                std::cerr << cap.Name << " min=" << cap.MinValue << " max=" << cap.MaxValue << " def=" << cap.DefaultValue << " auto=" << cap.IsAutoSupported << " readonly=" << !cap.IsWritable<<" :" << cap.Description<< std::endl;
                CamOptionId opt_id = opt_count;
                switch(cap.ControlType) {
                case ASI_GAIN:
                case ASI_EXPOSURE:
                    if(cap.IsAutoSupported)
                        auto_exp=true;
                    break;
                case ASI_WB_R:
                case ASI_WB_B:
                    if(cap.IsAutoSupported)
                        auto_wb=true;
                    break;
                default:
                    ;
                }
                switch(cap.ControlType) {
                case ASI_GAIN: opt_id = opt_gain; break;
                case ASI_EXPOSURE: opt_id = opt_exp; break;
                case ASI_WB_R: opt_id = opt_wb_r; break;
                case ASI_WB_B: opt_id = opt_wb_b; break;
                case ASI_TEMPERATURE: opt_id = opt_temperature; break;
                case ASI_TARGET_TEMP: opt_id = opt_cooler_target; break;
                case ASI_COOLER_ON: opt_id = opt_cooler_on; break;
                case ASI_FAN_ON: opt_id = opt_fan_on; break;
                case ASI_COOLER_POWER_PERC: opt_id = opt_cooler_power_perc; break;
                default:
                    continue;
                }
                opts.push_back(opt_id);
                ops_map_[opt_id] = cap;
            }
            if(auto_exp) {
                opts.insert(opts.begin(),opt_auto_exp);
                ops_map_[opt_auto_exp] = ASI_CONTROL_CAPS();
            }
            if(auto_wb) {
                opts.insert(opts.begin(),opt_auto_wb);
                ops_map_[opt_auto_wb] = ASI_CONTROL_CAPS();
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
            ASI_BOOL auto_val;
            if(id == opt_auto_exp || id==opt_auto_wb) {
                ASI_ERROR_CODE code = ASIGetControlValue(info_.CameraID,id == opt_auto_exp ? ASI_EXPOSURE : ASI_WB_R,&val,&auto_val);
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
            ASI_ERROR_CODE code = ASIGetControlValue(info_.CameraID,cap.ControlType,&val,&auto_val);
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
            ASI_ERROR_CODE code;
            ASI_BOOL auto_value;

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
                    ASI_BOOL aval = value ? ASI_TRUE : ASI_FALSE;
                    if( (code = ASIGetControlValue(info_.CameraID,ASI_EXPOSURE,&exp_val,&auto_value))
                        || (code = ASIGetControlValue(info_.CameraID,ASI_GAIN,&gain_val,&auto_value))
                        || (code = ASISetControlValue(info_.CameraID,ASI_EXPOSURE,exp_val,aval))
                        || (code = ASISetControlValue(info_.CameraID,ASI_GAIN,gain_val,aval)))
                    {
                        break;
                    }
                }
                break;
            case opt_auto_wb:
                {
                    long r_val,b_val;
                    ASI_BOOL aval = value ? ASI_TRUE : ASI_FALSE;
                    if( (code = ASIGetControlValue(info_.CameraID,ASI_WB_R,&r_val,&auto_value))
                        || (code = ASIGetControlValue(info_.CameraID,ASI_WB_B,&b_val,&auto_value))
                        || (code = ASISetControlValue(info_.CameraID,ASI_WB_R,r_val,aval))
                        || (code = ASISetControlValue(info_.CameraID,ASI_WB_B,b_val,aval)))
                    {
                        break;
                    }
                }
                break;
            case opt_exp:
                code = ASISetControlValue(info_.CameraID,ASI_EXPOSURE,long(value*1000),ASI_FALSE);
                break;
            case opt_gain:
            case opt_wb_r:
            case opt_wb_b:
            case opt_cooler_target:
            case opt_cooler_on:
            case opt_fan_on:
            case opt_cooler_power_perc:
                code = ASISetControlValue(info_.CameraID,cap.ControlType,long(value),ASI_FALSE);
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
        ASI_CAMERA_INFO info_;
        std::map<CamOptionId,ASI_CONTROL_CAPS> ops_map_;
        std::atomic<int> stream_active_;
        int frame_counter_ = 0;
        // protected by mutex
        std::mutex lock_;
        frame_callback_type callback_; 
        std::thread thread_;
    };
 


    class SingleASICameraDriver : public CameraDriver {
    public:
        SingleASICameraDriver(int id,CamErrorCode &e) : id_(id)
        {
            ASI_CAMERA_INFO info;
            ASI_ERROR_CODE code = ASIGetCameraPropertyByID(id_,&info);
            if(code != ASI_SUCCESS) {
                e = make_message("Failed to access camera",code);
                return;
            }
            name_ = info.Name;
        }
        virtual std::vector<std::string> list_cameras(CamErrorCode &) 
        {
            return {name_};
        }
        virtual std::unique_ptr<Camera> open_camera(int id,CamErrorCode &e)
        {
            std::unique_ptr<Camera> cam;
            if(id!=0) {
                e = "No such camera " + std::to_string(id);
                return cam;
            }
            cam = std::unique_ptr<Camera>(new ASICamera(id_,e));
            if(e)
                cam = nullptr;
            return cam;
        }
    private:
        int id_;
        std::string name_;
    };


    class ASICameraDriver : public CameraDriver {
    public:
        virtual std::vector<std::string> list_cameras(CamErrorCode &e) 
        {
            int N = ASIGetNumOfConnectedCameras();
            ASI_CAMERA_INFO info;
            ASI_ERROR_CODE r;
            for(int i=0;i<N;i++) {
                r=ASIGetCameraProperty(&info,i);
                if(r!=ASI_SUCCESS) {
                    e = make_message("Failed to get camera details",r);
                    return names_;
                }
                std::string name = std::string(info.Name) + "/" + std::to_string(i);
                names_.push_back(name);
                cams_.push_back(info);
            }
            return names_;
        }
        virtual std::unique_ptr<Camera> open_camera(int id,CamErrorCode &e) 
        {
            if(size_t(id) >= cams_.size()) {
                e = "No such camera " + std::to_string(id);
                return std::unique_ptr<Camera>();
            }
            std::unique_ptr<Camera> cam(new ASICamera(cams_.at(id),e));
            if(e)
                cam = nullptr;
            return cam;
        }
    private:
        std::vector<ASI_CAMERA_INFO> cams_;
        std::vector<std::string> names_;
        
    };
}

extern "C" {
    ols::CameraDriver *ols_get_asi_driver(int cam_id,ols::CamErrorCode *e)
    {
        std::unique_ptr<ols::CameraDriver> p;
        if(cam_id != -1)
            p.reset(new ols::SingleASICameraDriver(cam_id,*e));
        else
            p.reset(new ols::ASICameraDriver());
        if(*e) {
            p.reset();
        }
        return p.release();
    }
}
