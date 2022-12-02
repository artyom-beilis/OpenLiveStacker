#include "camera.h"

#ifdef ANDROID_SUPPORT
#include "libusb-1.0/libusb.h"
#endif

#include "libuvc/libuvc.h"

#include <mutex>
#include <string.h>
#include <sys/time.h>

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
            if(strh_)
                uvc_stream_stop(strh_);
            uvc_close(devh_);
        }
        virtual std::string name()
        {
            return name_;
        }
        virtual std::vector<CamStreamFormat> formats() 
        {
            read_formats();
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
        virtual void start_stream(CamStreamFormat fmt,frame_callback_type cb) 
        {
            if(strh_)
                stop_stream();

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
        }
        virtual void stop_stream() 
        {
            if(!strh_)
                return;
            {
                std::unique_lock<std::mutex> guard(lock_);
                callback_ = nullptr;
            }
            uvc_error_t res = uvc_stream_stop(strh_);
            strh_ = nullptr;
            if(res < 0)
                throw UVCError("Failed to stop stream",res);
        }
        virtual std::vector<CamOptionId> supported_options() 
        {
            return std::vector<CamOptionId>();
        }
        virtual CamParam get_parameter(CamOptionId /*id*/)
        {
            throw UVCError("Option not supported");
        }
        virtual void set_parameter(CamOptionId /*id*/,double /*value*/)
        {
            throw UVCError("Option not supported");
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
        uvc_stream_handle_t *strh_ = nullptr;
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

        virtual std::vector<std::string> list_cameras() 
        {
            load_device_list();
            return names_;
        }
        ~UVCCameraDriver()
        {
            uvc_exit(ctx_);
            if(device_list_)
                uvc_free_device_list(device_list_,1);
        }

        std::unique_ptr<Camera> open_camera(int id)
        {
            load_device_list();
            if(id < 0 || size_t(id) >= name_to_device_.size()) {
                throw UVCError("No device with index #" + std::to_string(id));
            }
            std::unique_ptr<Camera> camera(new UVCCamera(device_list_[name_to_device_[id]],names_[id]));
            return camera;
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
    class AndroidUVCCameraDriver {
    public:
        static bool usb_option_set;
        AndroidUVCCameraDriver(int fd) 
        {
            if(!usb_option_set) {
                int r = libusb_set_option(NULL,LIBUSB_OPTION_NO_DEVICE_DISCOVERY, NULL);
                if(r < 0)
i                   throw UVCError("Failed to set no discovery option");
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

        std::unique_ptr<Camera> open_camera(int id)
        {
            if(id != 0)
                throw UVCError("Invalid device number");
            uvc_device_handle_t devh = nullptr;
            uvc_error_t res = uvc_wrap(fd_,ctx_,&devh_);
            std::unique_ptr<Camera> camera(new UVCCamera(devh,"camera_0"));
            return camera;
        }

    private:

        int fd_;
        uvc_context_t ctx_;
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
