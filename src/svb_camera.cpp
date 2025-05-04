#include "camera.h"
#include "SVBCameraSDK.h"
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
    static std::string make_message(std::string const &msg,SVB_ERROR_CODE code)
    {
        std::string str;
        switch(code) {
        case SVB_SUCCESS: str="Success"; break;
        case SVB_ERROR_INVALID_INDEX:  str="no camera connected or index value out of boundary"; break; 
        case SVB_ERROR_INVALID_ID:  str="invalid ID"; break; 
        case SVB_ERROR_INVALID_CONTROL_TYPE:  str="invalid control type"; break; 
        case SVB_ERROR_CAMERA_CLOSED:  str="camera didn't open"; break; 
        case SVB_ERROR_CAMERA_REMOVED:  str="failed to find the camera, maybe the camera has been removed"; break; 
        case SVB_ERROR_INVALID_PATH:  str="cannot find the path of the file"; break; 
        case SVB_ERROR_INVALID_FILEFORMAT: str="invalid file format"; break;
        case SVB_ERROR_INVALID_SIZE:  str="wrong video format size"; break; 
        case SVB_ERROR_INVALID_IMGTYPE:  str="unsupported image format"; break; 
        case SVB_ERROR_OUTOF_BOUNDARY:  str="the startpos is out of boundary"; break; 
        case SVB_ERROR_TIMEOUT:  str="timeout"; break; 
        case SVB_ERROR_INVALID_SEQUENCE: str="stop capture first"; break; 
        case SVB_ERROR_BUFFER_TOO_SMALL:  str="buffer size is not big enough"; break; 
        case SVB_ERROR_VIDEO_MODE_ACTIVE: str="video mode active"; break;
        case SVB_ERROR_EXPOSURE_IN_PROGRESS: str="exposure in process"; break;
        case SVB_ERROR_GENERAL_ERROR: str="general error"; break; 
        case SVB_ERROR_INVALID_MODE: str="the current mode is wrong"; break; 
        default:
            str="unknown code";
        };
        return msg + ": " + str;
    }
    
    class SVBCamera : public Camera {
    public:
        SVBCamera(SVB_CAMERA_INFO info,CamErrorCode &e) : info_(info)
        {
            stream_active_ = 0;
            SVB_ERROR_CODE code = SVBOpenCamera(info_.CameraID);
            if(code) {
                e = make_message("Failed to open camera",code);
                return;
            }
            code = SVBGetCameraProperty(info_.CameraID,&props_);
            if(code) {
                SVBCloseCamera(info_.CameraID);
                e = make_message("Failed to get camera properties",code);
                return;
            }
        }
        virtual ~SVBCamera()
        {
            CamErrorCode e;
            stop_stream(e);
            if(e) {
                fprintf(stderr,"Failed to close stream %s\n",e.message().c_str());
            }
            SVBCloseCamera(info_.CameraID);
        }
        /// Camera name
        virtual std::string name(CamErrorCode &) 
        {
            return info_.FriendlyName;
        }
        /// Return list of suppored video formats
        virtual std::vector<CamStreamFormat> formats(CamErrorCode &)
        {
            std::vector<CamStreamFormat> res;
            std::vector<SVB_IMG_TYPE> video_formats;
            for(unsigned i=0;props_.SupportedVideoFormat[i] != SVB_IMG_END && i<sizeof(props_.SupportedVideoFormat)/sizeof(props_.SupportedVideoFormat[0]);i++) {
                video_formats.push_back(props_.SupportedVideoFormat[i]);
            }
            std::stable_sort(video_formats.begin(),video_formats.end(),[](SVB_IMG_TYPE a,SVB_IMG_TYPE b) {
                int ind_a = a == SVB_IMG_RAW16 ? 0 : 1;
                int ind_b = b == SVB_IMG_RAW16 ? 0 : 1;
                return ind_a<ind_b;
            });
            for(auto video_format: video_formats) {
                CamStreamFormat fmt;
                switch(video_format) {
                case SVB_IMG_RAW8:
                    fmt.format = props_.IsColorCam ? stream_raw8 : stream_mono8;
                    break;
                case SVB_IMG_RGB24:
                    fmt.format = stream_rgb24;
                    break;
                case SVB_IMG_RAW16:
                    fmt.format = props_.IsColorCam ? stream_raw16 : stream_mono16;
                    break;
                case SVB_IMG_Y8:
                    fmt.format = stream_mono8;
                    break;
                default:
                    continue;
                }
                fmt.framerate = -1;
                for(unsigned j=0;props_.SupportedBins[j]!=0 && j < sizeof(props_.SupportedBins)/sizeof(props_.SupportedBins[0]);j++) {
                    int bin = props_.SupportedBins[j];
                    if(props_.MaxWidth % bin == 0 && props_.MaxHeight % bin == 0) {
                        fmt.width = props_.MaxWidth / bin;
                        fmt.height = props_.MaxHeight / bin;
                        fmt.bin = bin;
                        fmt.roi_den = 1;
                        fmt.roi_num = 1;
                        res.push_back(fmt);
                        for(int scale = 1;scale < 8;scale++) {
                            int newW,newH;
                            int num,den;
                            if(scale == 1) {
                                if(props_.MaxWidth <= 1280 || props_.MaxHeight <= 960) { 
                                    continue;
                                }
                                /// make 2/3s ROI
                                newW = (props_.MaxWidth / bin) * 2 / 3 / 8 * 8;
                                newH = (props_.MaxHeight / bin) * 2 / 3 / 8 * 8;
                                num = 2;
                                den = 3;
                            }
                            else {
                                /// rest 1/N th ROI
                                newW = (props_.MaxWidth / bin) / scale / 8 * 8;
                                newH = (props_.MaxHeight / bin) / scale / 8 * 8; // actually can be /2 * 2 but for symmetry of square frames I keep it this way
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
            if((format.format == stream_raw16 || format.format == stream_raw8) && props_.IsColorCam) {
                switch(props_.BayerPattern) {
                case SVB_BAYER_RG : frm.bayer = bayer_rg; break;
                case SVB_BAYER_GR : frm.bayer = bayer_gr; break;
                case SVB_BAYER_BG : frm.bayer = bayer_bg; break;
                case SVB_BAYER_GB : frm.bayer = bayer_gb; break;
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
            SVB_IMG_TYPE img_type;
            int bpp=-1;
            switch(format.format) {
            case stream_rgb24:
                img_type = SVB_IMG_RGB24;
                bpp=3;
                break;
            case stream_raw16:
            case stream_mono16:
                img_type = SVB_IMG_RAW16;
                bpp=2;
                break;
            case stream_raw8:
                img_type = SVB_IMG_RAW8;
                bpp=1;
                break;
            case stream_mono8:
                img_type = SVB_IMG_Y8;
                bpp=1;
                break;
            default:
                e = "Invalid format";
                return;
            }
            if(format.width*format.bin > props_.MaxWidth || format.height * format.bin > props_.MaxHeight) {
                e = "Unsupported binning/size combination";
                return;
            }

            int bin = 0;
            for(auto supported_bin : props_.SupportedBins) {
                if(supported_bin == format.bin || supported_bin == 0) {
                    bin = supported_bin;
                    break;
                }
            }
            if(bin <=0) {
                e = "Unsupported image binning " + std::to_string(format.bin);
                return;
            }
            int start_x = (props_.MaxHeight/bin - format.height) / 2;
            int start_y = (props_.MaxWidth/bin  - format.width) / 2;
            start_x -= start_x % 8;
            start_y -= start_y % 2;
            SVB_ERROR_CODE code  = SVBSetROIFormatEx(info_.CameraID, start_x, start_y, format.width, format.height, bin, img_type);
            if(code != SVB_SUCCESS) {
                std::ostringstream ss;
                ss << "Failed to set ROI of " << format.width << "x" << format.height;
                e = make_message(ss.str(),code);
                return;
            }
            code = SVBStartVideoCapture(info_.CameraID);
            if(code != SVB_SUCCESS) {
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
                    SVB_ERROR_CODE status = SVBGetVideoData(info_.CameraID,buf.data(),buf.size(),500); 
                    if(status == SVB_SUCCESS) {
                        handle_frame(format,buf);
                    }
                    else if(status == SVB_ERROR_TIMEOUT) {
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
            SVB_ERROR_CODE code = SVBStopVideoCapture(info_.CameraID);
            if(code != SVB_SUCCESS) {
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
            SVB_ERROR_CODE code = SVBGetNumOfControls(info_.CameraID,&N);
            if(code) {
                e=make_message("Failed to get controls",code);
                return opts;
            }
            bool auto_exp=false,auto_wb=false;
            for(int i=0;i<N;i++) {
                SVB_CONTROL_CAPS cap;
                code = SVBGetControlCaps(info_.CameraID,i,&cap);
                if(code) {
                    e = make_message("Failed to get control info",code);
                    return opts;
                }


                std::cerr << cap.Name << " min=" << cap.MinValue << " max=" << cap.MaxValue << " def=" << cap.DefaultValue << " auto=" << cap.IsAutoSupported << " readonly=" << !cap.IsWritable<<" :" << cap.Description<< std::endl;
                CamOptionId opt_id = opt_count;
                switch(cap.ControlType) {
                case SVB_GAIN:
                case SVB_EXPOSURE:
                    if(cap.IsAutoSupported)
                        auto_exp=true;
                    break;
                case SVB_WB_R:
                case SVB_WB_B:
                    if(cap.IsAutoSupported)
                        auto_wb=true;
                    break;
                default:
                    ;
                }
                switch(cap.ControlType) {
                case SVB_GAIN: opt_id = opt_gain; break;
                case SVB_BLACK_LEVEL: opt_id = opt_black_level; break;
                case SVB_EXPOSURE: opt_id = opt_exp; break;
                case SVB_WB_R: opt_id = opt_wb_r; break;
                case SVB_WB_B: opt_id = opt_wb_b; break;
                case SVB_CURRENT_TEMPERATURE: opt_id = opt_temperature; break;
                case SVB_TARGET_TEMPERATURE: opt_id = opt_cooler_target; break;
                case SVB_COOLER_ENABLE: opt_id = opt_cooler_on; break;
                case SVB_COOLER_POWER: opt_id = opt_cooler_power_perc; break;
                default:
                    continue;
                }
                opts.push_back(opt_id);
                ops_map_[opt_id] = cap;
            }
            if(auto_exp) {
                opts.insert(opts.begin(),opt_auto_exp);
                ops_map_[opt_auto_exp] = SVB_CONTROL_CAPS();
            }
            if(auto_wb) {
                opts.insert(opts.begin(),opt_auto_wb);
                ops_map_[opt_auto_wb] = SVB_CONTROL_CAPS();
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
            SVB_BOOL auto_val;
            if(id == opt_auto_exp || id==opt_auto_wb) {
                SVB_ERROR_CODE code = SVBGetControlValue(info_.CameraID,id == opt_auto_exp ? SVB_EXPOSURE : SVB_WB_R,&val,&auto_val);
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
            SVB_ERROR_CODE code = SVBGetControlValue(info_.CameraID,cap.ControlType,&val,&auto_val);
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
            case opt_black_level:
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
            SVB_ERROR_CODE code;
            SVB_BOOL auto_value;

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
                    SVB_BOOL aval = value ? SVB_TRUE : SVB_FALSE;
                    if( (code = SVBGetControlValue(info_.CameraID,SVB_EXPOSURE,&exp_val,&auto_value))
                        || (code = SVBGetControlValue(info_.CameraID,SVB_GAIN,&gain_val,&auto_value))
                        || (code = SVBSetControlValue(info_.CameraID,SVB_EXPOSURE,exp_val,aval))
                        || (code = SVBSetControlValue(info_.CameraID,SVB_GAIN,gain_val,aval)))
                    {
                        break;
                    }
                }
                break;
            case opt_auto_wb:
                {
                    long r_val,b_val;
                    SVB_BOOL aval = value ? SVB_TRUE : SVB_FALSE;
                    if( (code = SVBGetControlValue(info_.CameraID,SVB_WB_R,&r_val,&auto_value))
                        || (code = SVBGetControlValue(info_.CameraID,SVB_WB_B,&b_val,&auto_value))
                        || (code = SVBSetControlValue(info_.CameraID,SVB_WB_R,r_val,aval))
                        || (code = SVBSetControlValue(info_.CameraID,SVB_WB_B,b_val,aval)))
                    {
                        break;
                    }
                }
                break;
            case opt_exp:
                code = SVBSetControlValue(info_.CameraID,SVB_EXPOSURE,long(value*1000),SVB_FALSE);
                break;
            case opt_gain:
            case opt_wb_r:
            case opt_wb_b:
            case opt_cooler_target:
            case opt_cooler_on:
            case opt_black_level:
            case opt_cooler_power_perc:
                code = SVBSetControlValue(info_.CameraID,cap.ControlType,long(value),SVB_FALSE);
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
        SVB_CAMERA_INFO info_;
        SVB_CAMERA_PROPERTY props_;
        std::map<CamOptionId,SVB_CONTROL_CAPS> ops_map_;
        std::atomic<int> stream_active_;
        int frame_counter_ = 0;
        // protected by mutex
        std::mutex lock_;
        frame_callback_type callback_; 
        std::thread thread_;
    };
 


    class SVBCameraDriver : public CameraDriver {
    public:
        virtual std::vector<std::string> list_cameras(CamErrorCode &e) 
        {
            int N = SVBGetNumOfConnectedCameras();
            SVB_CAMERA_INFO info;
            SVB_ERROR_CODE r;
            for(int i=0;i<N;i++) {
                r=SVBGetCameraInfo(&info,i);
                if(r!=SVB_SUCCESS) {
                    e = make_message("Failed to get camera details",r);
                    return names_;
                }
                std::string name = std::string(info.FriendlyName) + "/" + std::to_string(i);
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
            std::unique_ptr<Camera> cam(new SVBCamera(cams_.at(id),e));
            if(e)
                cam = nullptr;
            return cam;
        }
    private:
        std::vector<SVB_CAMERA_INFO> cams_;
        std::vector<std::string> names_;
        
    };
}

extern "C" {
    ols::CameraDriver *ols_get_svb_driver(int ,ols::CamErrorCode *)
    {
        return new ols::SVBCameraDriver();
    }
}
