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
    class ASIError : public CamError {
    public:
        ASIError(std::string const &msg) : CamError(msg)
        {
        }
        ASIError(std::string const &msg, ASI_ERROR_CODE code) : CamError(make_message(msg,code))
        {
        }
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
    };
    class ASICamera : public Camera {
    public:
        /// for Android
        ASICamera(int id)
        {
            stream_active_ = 0;
            ASI_ERROR_CODE code = ASIGetCameraPropertyByID(id,&info_);
            if(code != ASI_SUCCESS)
                throw ASIError("Failed to access camera",code);
            code = ASIInitCamera(info_.CameraID);
            if(code) {
                ASICloseCamera(info_.CameraID);
                throw ASIError("Failed to init camera",code);
            }
        }
        ASICamera(ASI_CAMERA_INFO info) : info_(info)
        {
            stream_active_ = 0;
            ASI_ERROR_CODE code = ASIOpenCamera(info_.CameraID);
            if(code)
                throw ASIError("Failed to open camera",code);
            code = ASIInitCamera(info_.CameraID);
            if(code) {
                ASICloseCamera(info_.CameraID);
                throw ASIError("Failed to init camera",code);
            }
        }
        virtual ~ASICamera()
        {
            try {
                CamErrorCode e;
                stop_stream(e);
                if(e) {
                    fprintf(stderr,"Failed to close stream %s\n",e.message().c_str());
                }
            }
            catch(...) {
                fprintf(stderr,"Failed to close stream\n");
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
                        res.push_back(fmt);
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

        /// Start a video stream with provided callback 
        virtual void start_stream(CamStreamFormat format,frame_callback_type callback,CamErrorCode &e) 
        {
            try {
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
                    throw ASIError("Invalid format");
                }
                int bin_w = info_.MaxWidth / format.width;
                int bin_h = info_.MaxHeight / format.height;
                if(bin_w != bin_h || bin_w < 1 || info_.MaxWidth % bin_w != 0 || info_.MaxWidth % bin_h !=0)
                    throw ASIError("Invalid frame dimensions");
                int bin = 0;
                for(auto supported_bin : info_.SupportedBins) {
                    if(supported_bin == bin_w || supported_bin == 0) {
                        bin = supported_bin;
                        break;
                    }
                }
                if(bin <=0)
                    throw ASIError("Unsupported image binning " + std::to_string(bin_w));

                ASI_ERROR_CODE code  = ASISetROIFormat(info_.CameraID, format.width, format.height, bin, img_type);
                if(code != ASI_SUCCESS) {
                    std::ostringstream ss;
                    ss << "Failed to set ROI of " << format.width << "x" << format.height;
                    throw ASIError(ss.str(),code);
                }
                code = ASIStartVideoCapture(info_.CameraID);
                if(code != ASI_SUCCESS)
                    throw ASIError("Failed to start video",code);
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
            catch(std::exception const &err) {
                e=CamErrorCode(err);
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
                ASI_ERROR_CODE code = ASIStopVideoCapture(info_.CameraID);
                if(code != ASI_SUCCESS)
                    throw ASIError("Failed to stop stream",code);
                stream_active_=0;
                thread_.join();
            }
            catch(std::exception const &err) {
                e=CamErrorCode(err);
            }
        }

        /// list of camera controls that the camera supports
        virtual std::vector<CamOptionId> supported_options(CamErrorCode &e)
        {
            std::vector<CamOptionId> opts;
            try {
                int N=0;
                ASI_ERROR_CODE code = ASIGetNumOfControls(info_.CameraID,&N);
                if(code)
                    throw ASIError("Failed to get controls",code);
                for(int i=0;i<N;i++) {
                    ASI_CONTROL_CAPS cap;
                    code = ASIGetControlCaps(info_.CameraID,i,&cap);
                    if(code)
                        throw ASIError("Failed to get control info",code);
                    std::cerr << cap.Name << " min=" << cap.MinValue << " max=" << cap.MaxValue << " def=" << cap.DefaultValue << " auto=" << cap.IsAutoSupported <<" :" << cap.Description<< std::endl;
                    switch(cap.ControlType) {
                    case ASI_GAIN:
                        opts.push_back(opt_gain);
                        ops_map_[opt_gain] = cap;
                        break;
                    case ASI_EXPOSURE:
                        opts.push_back(opt_exp);
                        ops_map_[opt_exp] = cap;
                        break;
                    default:
                        continue;
                    }
                }
            }
            catch(std::exception const &err) {
                e=CamErrorCode(err);
            }
            return opts;
        }
        /// get camera control
        virtual CamParam get_parameter(CamOptionId id,bool /*current_only*/,CamErrorCode &e) 
        {
            CamParam r;
            auto p = ops_map_.find(id);
            try {
                if(p==ops_map_.end())
                    throw ASIError("Option " +  cam_option_id_to_name(id));
                auto cap = p->second;
                memset(&r,0,sizeof(r));
                r.option = id;
                long val;
                ASI_BOOL auto_val;
                ASI_ERROR_CODE code = ASIGetControlValue(info_.CameraID,cap.ControlType,&val,&auto_val);
                if(code)
                    throw ASIError("Failed to get control value",code);
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
                    {
                        r.type = type_number;
                        r.min_val = cap.MinValue;
                        r.max_val = cap.MaxValue;
                        r.def_val = cap.DefaultValue;
                        r.cur_val = val;
                        r.step_size = 1;
                    }
                    break;
                default:
                    throw ASIError("Internal error can't get control");
                }
            }
            catch(std::exception const &err) {
                e=CamErrorCode(err);
            }
            return r;
        }
        /// set camera control
        virtual void set_parameter(CamOptionId id,double value,CamErrorCode &e)
        {
            try {
                ASI_ERROR_CODE code;
                switch(id) {
                case opt_exp:
                    code = ASISetControlValue(info_.CameraID,ASI_EXPOSURE,long(value*1000),ASI_FALSE);
                    break;
                case opt_gain:
                    code = ASISetControlValue(info_.CameraID,ASI_GAIN,long(value),ASI_FALSE);
                    break;
                default:
                    throw ASIError("Unimplemented");
                }
                if(code)
                    throw ASIError("Failed to set option", code);
            }
            catch(std::exception const &err) {
                e=CamErrorCode(err);
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
        SingleASICameraDriver(int id) : id_(id)
        {
            ASI_CAMERA_INFO info;
            ASI_ERROR_CODE code = ASIGetCameraPropertyByID(id_,&info);
            if(code != ASI_SUCCESS)
                throw ASIError("Failed to access camera",code);
            name_ = info.Name;
        }
        virtual std::vector<std::string> list_cameras(CamErrorCode &) 
        {
            return {name_};
        }
        virtual std::unique_ptr<Camera> open_camera(int id,CamErrorCode &e)
        {
            try {
                if(id!=0)
                    throw ASIError("No such camera " + std::to_string(id));
                std::unique_ptr<Camera> cam(new ASICamera(id_));
                return cam;
            }
            catch(std::exception const &err) {
                e=CamErrorCode(err);
                return std::unique_ptr<Camera>();
            }
        }
    private:
        int id_;
        std::string name_;
    };


    class ASICameraDriver : public CameraDriver {
    public:
        virtual std::vector<std::string> list_cameras(CamErrorCode &e) 
        {
            try {
                int N = ASIGetNumOfConnectedCameras();
                ASI_CAMERA_INFO info;
                ASI_ERROR_CODE r;
                for(int i=0;i<N;i++) {
                    r=ASIGetCameraProperty(&info,i);
                    if(r!=ASI_SUCCESS)
                        throw ASIError("Failed to get camera details",r);
                    std::string name = std::string(info.Name) + "/" + std::to_string(i);
                    names_.push_back(name);
                    cams_.push_back(info);
                }
            }
            catch(std::exception const &err) {
                e=CamErrorCode(err);
            }
            return names_;
        }
        virtual std::unique_ptr<Camera> open_camera(int id,CamErrorCode &e) 
        {
            try {
                if(size_t(id) >= cams_.size())
                    throw ASIError("No such camera " + std::to_string(id));
                std::unique_ptr<Camera> cam(new ASICamera(cams_.at(id)));
                return cam;
            }
            catch(std::exception const &err) {
                e=CamErrorCode(err);
                return std::unique_ptr<Camera>();
            }
        }
    private:
        std::vector<ASI_CAMERA_INFO> cams_;
        std::vector<std::string> names_;
        
    };
}

extern "C" {
    ols::CameraDriver *ols_get_asi_driver(int cam_id = -1)
    {
        try {
            if(cam_id != -1)
                return new ols::SingleASICameraDriver(cam_id);
            else
                return new ols::ASICameraDriver();
        }
        catch(std::exception const &) {
            return nullptr;
        }
    }
}
