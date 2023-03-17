#include "camera.h"

#ifdef ANDROID_SUPPORT
#include "libusb-1.0/libusb.h"
#endif

#include "libuvc/libuvc.h"

#include <mutex>
#include <iostream>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

namespace ols {
    class UVCError : public CamError {
    public:
        UVCError(std::string const &msg) : CamError(msg) {}
        UVCError(std::string const &msg,uvc_error_t err) : CamError(msg + ":" + uvc_strerror(err)) {}
    };


    extern "C" void ols_uvccamera_callback(uvc_frame_t *frame,void *obj);

    class UVCCamera : public Camera {
    public:
        UVCCamera(uvc_device_handle_t *devh,std::string const &name) :
            name_(name),
            devh_(devh)
        {
        }
        UVCCamera(uvc_device_t *dev,std::string const &name) :
            name_(name)
        {
            uvc_error_t res = uvc_open(dev,&devh_);
            if(res < 0)
                throw UVCError("Failed to open camera",res);
        }
        ~UVCCamera()
        {
            if(stream_active_)
                uvc_stop_streaming(devh_);
            uvc_close(devh_);
        }
        virtual std::string name(CamErrorCode &)
        {
            return name_;
        }
        virtual std::vector<CamStreamFormat> formats(CamErrorCode &e) 
        {
            try {
                read_formats();
            }
            catch(std::exception const &err) {
                e = CamErrorCode(err);
            }
            return formats_;
        }
        void set_error(CamFrame &frm,char const *msg)
        {
            frm.data = msg;
            frm.format = stream_error;
            frm.data_size = strlen(msg);
        }
        void callback(uvc_frame_t *frame)
        {
            struct timeval tv;
            gettimeofday(&tv,nullptr);
            CamFrame frm;
            frm.unix_timestamp = tv.tv_sec;
            frm.unix_timestamp += tv.tv_usec * 1e-6;
            frm.width = frame->width;
            frm.height = frame->height;
            frm.data = frame->data;
            frm.data_size = frame->data_bytes;
            frm.frame_counter = frame_counter_ ++;

            if(frame->frame_format == UVC_COLOR_FORMAT_MJPEG) {
                frm.format = stream_mjpeg;
            }
            else if(frame->frame_format == UVC_COLOR_FORMAT_YUYV) {
                frm.format = stream_yuv2;
                if(size_t(frm.width * frm.height * 2) != frame->data_bytes)
                    set_error(frm,"Partial data in frame");
            }
            else {
                set_error(frm,"Invalid frame format");
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
        virtual void start_stream(CamStreamFormat fmt,frame_callback_type cb,CamErrorCode &e) 
        {
            try {
                if(stream_active_) {
                    stop_stream(e);
                    if(e)
                        return;
                }

                int index = -1;
                for(size_t i=0;i<formats_.size();i++) {
                    CamStreamFormat &ref_fmt = formats_[i];
                    if(ref_fmt.width == fmt.width 
                       && ref_fmt.height == fmt.height
                       && ref_fmt.format == fmt.format
                       && (fmt.framerate < 0 || ref_fmt.framerate == fmt.framerate))
                    {
                        index = i;
                        break;
                    }
                }
                if(index == -1)
                    throw UVCError("No apropriate stream format for camera");

                int tries = 0;
                uvc_error_t res;
                while(tries < 5) {
                    res = uvc_get_stream_ctrl_format_size(devh_,
                            &ctrl_,
                            (fmt.format == stream_yuv2 ? UVC_FRAME_FORMAT_YUYV : UVC_FRAME_FORMAT_MJPEG),
                            fmt.width,fmt.height,
                            formats_[index].framerate);
                    if(res == 0)
                        break;
                    tries ++;
                }
                if(res < 0) {
                    throw UVCError("Failed to create stram control",res);
                }
                {
                    std::unique_lock<std::mutex> guard(lock_);
                    callback_ = cb;
                }

                res = uvc_start_streaming(devh_,&ctrl_,ols_uvccamera_callback,this,0);
                if(res < 0) {
                    throw UVCError("Failed to start stream : " + std::to_string(res),res);
                }
                stream_active_ = true;
            }
            catch(std::exception const &err) {
                e = CamErrorCode(err);
            }
        }
        virtual void stop_stream(CamErrorCode &e) 
        {
            try {
                if(!stream_active_)
                    return;
                {
                    std::unique_lock<std::mutex> guard(lock_);
                    callback_ = nullptr;
                }
                uvc_stop_streaming(devh_);
                stream_active_ = false;
            }
            catch(std::exception const &err) {
                e = CamErrorCode(err);
            }
        }
        virtual std::vector<CamOptionId> supported_options(CamErrorCode &) 
        {
            std::vector<CamOptionId> controls;
            uvc_input_terminal_t const *ct = uvc_get_camera_terminal(devh_);
            uvc_processing_unit_t const *pu = uvc_get_processing_units(devh_); 
            uint32_t u32;
            uint16_t u16;
            int16_t i16;

            if(ct && (ct->bmControls & ((1<<UVC_CT_AE_MODE_CONTROL) | (1<<UVC_CT_AE_PRIORITY_CONTROL))))
                controls.push_back(opt_auto_exp);
            if(pu && (pu->bmControls & ((1<<UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL) | (1<<UVC_PU_WHITE_BALANCE_COMPONENT_CONTROL))))
                controls.push_back(opt_auto_wb);
            //if(ct && (ct->bmControls & (1<<UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL)) && uvc_get_exposure_abs(devh_,&u32,UVC_GET_CUR) >= 0)
            if(uvc_get_exposure_abs(devh_,&u32,UVC_GET_CUR) >= 0)
                controls.push_back(opt_exp);
            if(pu && (pu->bmControls & (1<<UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL)) && uvc_get_white_balance_temperature(devh_,&u16,UVC_GET_CUR)>=0)
                controls.push_back(opt_wb);
            if(pu && (pu->bmControls & (1<<UVC_PU_GAIN_CONTROL)) && uvc_get_gain(devh_,&u16,UVC_GET_CUR)>=0)
                controls.push_back(opt_gain);
            if(pu && (pu->bmControls & (1<<UVC_PU_GAMMA_CONTROL)) && uvc_get_gamma(devh_,&u16,UVC_GET_CUR)>=0)
                controls.push_back(opt_gamma);
            if(pu && (pu->bmControls & (1<<UVC_PU_BRIGHTNESS_CONTROL)) && uvc_get_brightness(devh_,&i16,UVC_GET_CUR)>=0)
                controls.push_back(opt_brightness);
            if(pu && (pu->bmControls & (1<<UVC_PU_CONTRAST_CONTROL)) && uvc_get_contrast(devh_,&u16,UVC_GET_CUR)>=0)
                controls.push_back(opt_contrast);
            return controls;
        }
        template<typename ItemType,typename FunctionType>
        void update_parameters(CamParam &r,FunctionType func,double scale,bool current_only)
        {
            ItemType vals[5];
            uvc_req_code codes[5]={UVC_GET_CUR,UVC_GET_MIN,UVC_GET_DEF,UVC_GET_RES,UVC_GET_MAX};
            int limit = current_only ? 1 : 5;
            for(int i=0;i<limit;i++) {
                int attempts = 0;
                uvc_error_t res;
                do {
                    res = func(devh_,vals + i,codes[i]);
                    attempts ++;
                    if(res < 0)
                        usleep(100000);
                } while(attempts < 5 && res < 0);
                if(res < 0)
                    throw UVCError("Failed to read value of " + cam_option_id_to_name(r.option) + " for " + std::to_string(i),res);
            }
            r.cur_val   = vals[0]*scale;
            r.min_val   = vals[1]*scale;
            r.def_val   = vals[2]*scale;
            r.step_size = vals[3]*scale;
            r.max_val   = vals[4]*scale;
        }
        virtual CamParam get_parameter(CamOptionId op_id,bool current_only,CamErrorCode &e)
        {
            CamParam r;
            memset(&r,0,sizeof(r));
            r.option = op_id;
            try {
                switch(op_id) {
                case opt_auto_exp:
                    {
                        r.type = type_bool;
                        uint8_t cur,def = 0;
                        uvc_error_t res = uvc_get_ae_mode(devh_,&cur,UVC_GET_CUR);
                        if(res < 0)
                            throw UVCError("AE query failed",res);
                        if(!current_only) {
                            uvc_error_t res = uvc_get_ae_mode(devh_,&def,UVC_GET_DEF);
                            if(res < 0)
                                throw UVCError("AE query failed",res);
                        }
                        
                        r.step_size = r.max_val =  1;
                        r.min_val = 0;
                        r.cur_val = cur == 2 || cur == 8;
                        r.def_val = def == 2 || def == 8;
                    }
                    break;
                case opt_auto_wb:
                    {
                        r.type = type_bool;
                        uint8_t cur,def = 0;
                        uvc_error_t res = uvc_get_white_balance_temperature_auto(devh_,&cur,UVC_GET_CUR);
                        if(res < 0) {
                            res = uvc_get_white_balance_component_auto(devh_,&cur,UVC_GET_CUR);
                            if(res < 0)
                                throw UVCError("WB query failed",res);
                        }
                        if(!current_only) {
                            uvc_error_t res = uvc_get_white_balance_temperature_auto(devh_,&def,UVC_GET_DEF);
                            if(res < 0) {
                                res = uvc_get_white_balance_component_auto(devh_,&def,UVC_GET_DEF);
                                if(res < 0)
                                    throw UVCError("WB query failed",res);
                            }
                        }
                        
                        r.step_size = r.max_val =  1;
                        r.min_val = 0;
                        r.cur_val = cur != 0;
                        r.def_val = def != 0;
                    }
                    break;
                case opt_exp:
                    r.type = type_msec;
                    update_parameters<uint32_t>(r,uvc_get_exposure_abs,0.1,current_only);
                    break;
                case opt_wb:
                    r.type = type_kelvin;
                    update_parameters<uint16_t>(r,uvc_get_white_balance_temperature,1.0,current_only);
                    break;
                case opt_gamma:
                    r.type = type_number;
                    update_parameters<uint16_t>(r,uvc_get_gamma,1e-2,current_only);
                    break;
                case opt_gain:
                    r.type = type_number;
                    update_parameters<uint16_t>(r,uvc_get_gain,1,current_only);
                    break;
                case opt_brightness:
                    r.type = type_number;
                    update_parameters<int16_t>(r,uvc_get_brightness,1,current_only);
                    break;
                case opt_contrast:
                    r.type = type_number;
                    update_parameters<uint16_t>(r,uvc_get_contrast,1,current_only);
                    break;
                default:
                    throw UVCError("Option not supported" + cam_option_id_to_name(op_id));
                }
            }
            catch(std::exception const &err) {
                e=CamErrorCode(err);
            }
            return r;
        }
        virtual void set_parameter(CamOptionId opt_id,double value,CamErrorCode &e)
        {
            try {
                uvc_error_t res;
                switch(opt_id) {
                case opt_auto_exp:
                    if(value != 0){
                        res = uvc_set_ae_mode(devh_,2);
                        if(res < 0) {
                            res = uvc_set_ae_mode(devh_,8);
                        }
                    }
                    else {
                        res = uvc_set_ae_mode(devh_,0);
                        if(res < 0) {
                            res = uvc_set_ae_mode(devh_,4);
                            if(res < 0) {
                                res = uvc_set_ae_mode(devh_,1);
                            }
                        }
                    }
                    break;
                case opt_auto_wb:
                    res = uvc_set_white_balance_temperature_auto(devh_,value ? 1 : 0);
                    if(res < 0)
                        res = uvc_set_white_balance_component_auto(devh_,value ? 1:0);
                    break;
                case opt_exp:
                    res = uvc_set_exposure_abs(devh_,value*10);
                    break;
                case opt_wb:
                    res = uvc_set_white_balance_temperature(devh_,value);
                    break;
                case opt_gain:
                    res = uvc_set_gain(devh_,value);
                    break;
                case opt_gamma:
                    res = uvc_set_gamma(devh_,value*100); 
                    break;
                case opt_brightness:
                    res = uvc_set_brightness(devh_,value);
                    break;
                case opt_contrast:
                    res = uvc_set_contrast(devh_,value);
                    break;
                default:
                    throw UVCError("Option not supported" + cam_option_id_to_name(opt_id));
                }
                if(res < 0) {
                    throw UVCError("Failed to set option" + cam_option_id_to_name(opt_id),res);
                }
            }
            catch(std::exception const &err) {
                e = CamErrorCode(err);
            }
        }

     
    private:

        void read_formats()
        {
            if(!formats_.empty())
                return;

            const uvc_format_desc_t *format_desc = uvc_get_format_descs(devh_);
            for(;format_desc;format_desc=format_desc->next) {
                CamStreamType type;
                if(format_desc->bDescriptorSubtype == UVC_VS_FORMAT_MJPEG)
                    type = stream_mjpeg;
                else if(format_desc->bDescriptorSubtype == UVC_VS_FORMAT_UNCOMPRESSED)
                    type = stream_yuv2;
                else
                    continue;
                for(const uvc_frame_desc_t *p = format_desc->frame_descs;p;p=p->next) {
                    CamStreamFormat fmt;
                    fmt.format = type;
                    fmt.width = p->wWidth;
                    fmt.height = p->wHeight;
                    fmt.framerate= 10000000 / p->dwDefaultFrameInterval;
                    formats_.push_back(fmt);
                }
            }
        }

        std::string name_;
        uvc_device_handle_t *devh_ = nullptr;
        uvc_stream_ctrl_t ctrl_;
        bool stream_active_ = true;
        std::vector<CamStreamFormat> formats_;
        int frame_counter_ = 0;
        // protected by mutex
        std::mutex lock_;
        frame_callback_type callback_; 

    };
    
    extern "C" void ols_uvccamera_callback(uvc_frame_t *frame,void *obj)
    {
        UVCCamera *cam = reinterpret_cast<UVCCamera *>(obj);
        cam->callback(frame);
    }

    class UVCCameraDriver : public CameraDriver {
    public:
        UVCCameraDriver() 
        {
            uvc_error_t res = uvc_init(&ctx_,nullptr);
            if(res < 0)
                throw UVCError("Failed to init libuvc",res);
        }

        virtual std::vector<std::string> list_cameras(CamErrorCode &e) 
        {
            try {
                load_device_list();
            }
            catch(std::exception const &err) {
                e=CamErrorCode(err);
            }
            return names_;
        }
        ~UVCCameraDriver()
        {
            uvc_exit(ctx_);
            if(device_list_)
                uvc_free_device_list(device_list_,1);
        }

        std::unique_ptr<Camera> open_camera(int id,CamErrorCode &e)
        {
            try {
                load_device_list();
                if(id < 0 || size_t(id) >= name_to_device_.size()) {
                    throw UVCError("No device with index #" + std::to_string(id));
                }
                std::unique_ptr<Camera> camera(new UVCCamera(device_list_[name_to_device_[id]],names_[id]));
                return camera;
            }
            catch(std::exception const &err) {
                e=CamErrorCode(err);
                return std::unique_ptr<Camera>();
            }
        }

    private:
        void load_device_list()
        {
            if(device_list_)
                return;
            uvc_error_t res;
            res = uvc_get_device_list(ctx_,&device_list_);
            if(res < 0)
                throw UVCError("Failed to list devices",res);
            for(int i=0;device_list_[i];i++) {
                uvc_device_descriptor_t *dsc = nullptr;
                if(uvc_get_device_descriptor(device_list_[i],&dsc) != UVC_SUCCESS)
                    continue;
                std::string name;
                if(dsc->product) {
                    name += dsc->product;
                    name += "/";
                    name += std::to_string(i);
                }
                else {
                    name = "camera_" + std::to_string(i);
                }
                uvc_free_device_descriptor(dsc);
                names_.push_back(name);    
                name_to_device_.push_back(i);
            }
        }

        uvc_context_t *ctx_;
        uvc_device_t **device_list_;
        std::vector<std::string> names_;
        std::vector<int> name_to_device_;
    };
#ifdef ANDROID_SUPPORT
    class AndroidUVCCameraDriver  : public CameraDriver{
    public:
        static bool usb_option_set;
        AndroidUVCCameraDriver(int fd) 
        {
            if(!usb_option_set) {
                int r = libusb_set_option(NULL,LIBUSB_OPTION_NO_DEVICE_DISCOVERY, NULL);
                if(r < 0)
                    throw UVCError("Failed to set no discovery option");
                usb_option_set=true;
            }
            uvc_error_t res = uvc_init(&ctx_,nullptr);
            if(res < 0)
                throw UVCError("Failed to init libuvc",res);
            fd_ = fd;
        }

        virtual std::vector<std::string> list_cameras() 
        {
            return std::vector<std::string>({"camera_0"});
        }
        AndroidUVCCameraDriver()
        {
            uvc_exit(ctx_);
        }

        std::unique_ptr<Camera> open_camera(int id,CamErrorCode &e)
        {
            try {
                if(id != 0)
                    throw UVCError("Invalid device number");
                uvc_device_handle_t *devh = nullptr;
                uvc_error_t res = uvc_wrap(fd_,ctx_,&devh);
                if(res < 0)
                    throw UVCError("Failed to wrap fd",res);
                std::unique_ptr<Camera> camera(new UVCCamera(devh,"camera_0"));
                return camera;
            }
            catch(std::exception const &err) {
                e=CamErrorCode(err);
                return std::unique_ptr<Camera>();
            }
        }

    private:

        int fd_;
        uvc_context_t *ctx_;
    };

    bool AndroidUVCCameraDriver::usb_option_set;

#endif  //ANDROID_SUPPORT

    std::unique_ptr<CameraDriver> get_uvc_driver(int fd = -1)
    {
        std::unique_ptr<CameraDriver> driver;
        #ifdef ANDROID_SUPPORT
        if(fd >= 0) {
            driver.reset(new AndroidUVCCameraDriver(fd));
            return driver;
        }
        #else
        if(fd >= 0)
            throw UVCError("No android support enabled to open camera with file descriptor");
        #endif
        driver.reset(new UVCCameraDriver());
        return driver;
    }
};


extern "C" ols::CameraDriver *ols_get_uvc_driver(int fd)
{
    try {
        auto p = ols::get_uvc_driver(fd);
        return p.release();
    }
    catch(std::exception const &e) {
        fprintf(stderr,"Failed to load driver %s",e.what());
        return nullptr;
    }
}
