#include "camera.h"
#include <media/NdkImageReader.h>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraManager.h>
#include <mutex>
#include <sstream>
#include <iostream>
#include <atomic>
#include <map>
#include <cctype>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <thread>
#include <algorithm>
#include <queue>
#include "sw_bin.h"
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/imgproc.hpp>

//#ifdef WITH_LIBRAW
//#include "libraw_wrapper.h"
//#endif

namespace ols {

    static FILE *error_stream = NULL;
    #define LOG(...) do { if( ::ols::error_stream) { fprintf( ::ols::error_stream,__VA_ARGS__); fflush(::ols::error_stream); }} while(0)

    static char const *camera_status_message(camera_status_t status)
    {
        char const *e = "";
        switch(status) {
        case ACAMERA_ERROR_UNKNOWN: e = "ACAMERA_ERROR_UNKNOWN"; break;
        case ACAMERA_ERROR_INVALID_PARAMETER: e = "ACAMERA_ERROR_INVALID_PARAMETER"; break;
        case ACAMERA_ERROR_CAMERA_DISCONNECTED: e = "ACAMERA_ERROR_CAMERA_DISCONNECTED"; break;
        case ACAMERA_ERROR_NOT_ENOUGH_MEMORY: e = "ACAMERA_ERROR_NOT_ENOUGH_MEMORY"; break;
        case ACAMERA_ERROR_METADATA_NOT_FOUND: e = "ACAMERA_ERROR_METADATA_NOT_FOUND"; break;
        case ACAMERA_ERROR_CAMERA_DEVICE        : e = "ACAMERA_ERROR_CAMERA_DEVICE        "; break;
        case ACAMERA_ERROR_CAMERA_SERVICE       : e = "ACAMERA_ERROR_CAMERA_SERVICE       "; break;
        case ACAMERA_ERROR_SESSION_CLOSED       : e = "ACAMERA_ERROR_SESSION_CLOSED       "; break;
        case ACAMERA_ERROR_INVALID_OPERATION    : e = "ACAMERA_ERROR_INVALID_OPERATION    "; break;
        case ACAMERA_ERROR_STREAM_CONFIGURE_FAIL: e = "ACAMERA_ERROR_STREAM_CONFIGURE_FAIL"; break;
        case ACAMERA_ERROR_CAMERA_IN_USE        : e = "ACAMERA_ERROR_CAMERA_IN_USE        "; break;
        case ACAMERA_ERROR_MAX_CAMERA_IN_USE    : e = "ACAMERA_ERROR_MAX_CAMERA_IN_USE    "; break;
        case ACAMERA_ERROR_CAMERA_DISABLED      : e = "ACAMERA_ERROR_CAMERA_DISABLED      "; break;
        case ACAMERA_ERROR_PERMISSION_DENIED    : e = "ACAMERA_ERROR_PERMISSION_DENIED    "; break;
        case ACAMERA_ERROR_UNSUPPORTED_OPERATION: e = "ACAMERA_ERROR_UNSUPPORTED_OPERATION"; break;
        default:
            e="Unknown error";
        }
        return e;
    }
    static void check(char const *op,camera_status_t status)
    {
        if(status == ACAMERA_OK)
            return;
        throw std::runtime_error("Operation " + std::string(op) + " failed, code:" 
            + std::to_string(int(status)) + ":" + std::string(camera_status_message(status)));
    }
    static void log_status(char const *op,camera_status_t status)
    {
        if(status == ACAMERA_OK)
            return;
        LOG("%s: %s:%d\n",op,camera_status_message(status),status);
    }

        
 
    extern "C" {
        static void ac_on_discnnected(void *,ACameraDevice *)
        {
            LOG("Camera disconnected\n");
        }
        static void ac_on_error(void *,ACameraDevice *,int error)
        {
            LOG("Camera error %d\n",error);
        }
        static void ac_image_callback(void* context, AImageReader* reader);
        static void ac_cap_ready(void *,ACameraCaptureSession *)
        {
            LOG("Capture ready\n");
        }
        static void ac_cap_closed(void *,ACameraCaptureSession *)
        {
            LOG("Capture closed\n");
        }
        static void ac_cap_active(void *,ACameraCaptureSession *)
        {
            LOG("Capture active\n");
        }
    }
  
    class AndroidCamera : public Camera {
    public:
        /// for Android
        AndroidCamera(std::string const &id,std::string const &name,ACameraManager *manager) :
            id_(id), name_(name), manager_(manager)
        {
            ACameraDevice_StateCallbacks callbacks = ACameraDevice_StateCallbacks();
            callbacks.context = this;
            callbacks.onDisconnected = ac_on_discnnected;
            callbacks.onError = ac_on_error;
            camera_status_t res = ACameraManager_openCamera(manager_,id_.c_str(),&callbacks,&dev_);
            check("ACameraManager_openCamera",res);
            res = ACameraManager_getCameraCharacteristics(manager_,id_.c_str(),&meta_);
            if(res != ACAMERA_OK) {
                ACameraDevice_close(dev_);
                dev_ = nullptr;
                check("ACameraManager_getCameraCharacteristics",res);
            }
            res = ACameraDevice_createCaptureRequest(dev_, TEMPLATE_STILL_CAPTURE, &request_);
            if(res != ACAMERA_OK) {
                ACameraMetadata_free(meta_);
                meta_ = nullptr;

                ACameraDevice_close(dev_);
                dev_ = nullptr;
                check("ACameraDevice_createCaptureRequest",res);
            }
            stream_active_ = 0;
        }
        virtual ~AndroidCamera()
        {
            CamErrorCode e;
            stop_stream(e);
            if(e) {
                LOG("Failed to close stream %s\n",e.message().c_str());
            }
            if(request_) {
                ACaptureRequest_free(request_);
                request_ = nullptr;
            }
            if(meta_) {
                ACameraMetadata_free(meta_);
                meta_ = nullptr;
            }
            if(dev_) {
                ACameraDevice_close(dev_);
                dev_ = nullptr;
            }
        }
        /// Camera name
        virtual std::string name(CamErrorCode &) 
        {
            return name_;
        }
        /// Return list of suppored video formats

        static char const *astream_format(int fmt)
        {
            char const *strfmt = "Unidentified";
            switch(fmt) {
                case AIMAGE_FORMAT_RGBA_8888        : strfmt = "RGBA_8888"; break; 
                case AIMAGE_FORMAT_RGBX_8888        : strfmt = "RGBX_8888"; break; 
                case AIMAGE_FORMAT_RGB_888          : strfmt = "RGB_888"; break; 
                case AIMAGE_FORMAT_RGB_565          : strfmt = "RGB_565"; break; 
                case AIMAGE_FORMAT_RGBA_FP16        : strfmt = "RGBA_FP16"; break; 
                case AIMAGE_FORMAT_YUV_420_888      : strfmt = "YUV_420_888"; break; 
                case AIMAGE_FORMAT_JPEG             : strfmt = "JPEG"; break; 
                case AIMAGE_FORMAT_RAW16            : strfmt = "RAW16"; break; 
                case AIMAGE_FORMAT_RAW_PRIVATE      : strfmt = "RAW_PRIVATE"; break; 
                case AIMAGE_FORMAT_RAW10            : strfmt = "RAW10"; break; 
                case AIMAGE_FORMAT_RAW12            : strfmt = "RAW12"; break; 
                case AIMAGE_FORMAT_DEPTH16          : strfmt = "DEPTH16"; break; 
                case AIMAGE_FORMAT_DEPTH_POINT_CLOUD: strfmt = "DEPTH_POINT_CLOUD"; break; 
                case AIMAGE_FORMAT_PRIVATE          : strfmt = "PRIVATE"; break; 
                case AIMAGE_FORMAT_Y8               : strfmt = "Y8"; break; 
                case AIMAGE_FORMAT_HEIC             : strfmt = "HEIC"; break; 
                case AIMAGE_FORMAT_DEPTH_JPEG       : strfmt = "DEPTH_JPEG"; break; 
                case 0x32315659                     : strfmt = "HAL_PIXEL_FORMAT_YV12"; break;
            }
            return strfmt;
        }

        static int to_val(CamStreamFormat f) 
        {
            int index;
            switch(f.format) {
            case stream_raw16:  index = 9; break;
            case stream_mono16: index = 8; break;
            case stream_mono8:  index = 5; break;
            case stream_mjpeg:  index = 3; break;
            default:
                index=0;
            }
            int kp = f.width * f.height / 1000;
            if(kp > 2500) {// 2.5 MP 
                kp = -kp;
            }
            return index * 100000 + kp;

        }
        void sort_formats()
        {
           std::stable_sort(formats_.begin(),formats_.end(),[](CamStreamFormat l,CamStreamFormat r) {
               return to_val(l) > to_val(r);
           });
        }

        virtual std::vector<CamStreamFormat> formats(CamErrorCode &e)
        {
            if(!formats_.empty())
                return formats_;

            try {
                ACameraMetadata_const_entry entry = ACameraMetadata_const_entry();
                camera_status_t res = ACameraMetadata_getConstEntry(meta_,ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,&entry);
                check("ACameraMetadata_getConstEntry ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS",res);
                for(unsigned i=0;i<entry.count;i+=4) {
                    if(entry.data.i32[i + 3]) // input
                        continue; 
                    int w = entry.data.i32[i + 1];
                    int h = entry.data.i32[i + 2];
                    int fmt = entry.data.i32[i];
                    LOG("Got format %s / 0x%x of %dx%d\n",astream_format(fmt),fmt,w,h);
                    if(fmt == AIMAGE_FORMAT_JPEG) {
                        CamStreamFormat fmt;
                        fmt.format = stream_mjpeg; 
                        fmt.width = w;
                        fmt.height = h;
                        formats_.push_back(fmt);
                    }
                    else if(fmt == AIMAGE_FORMAT_YUV_420_888) {
                        CamStreamFormat fmt;
                        fmt.format = stream_mono8;
                        fmt.width = w;
                        fmt.height = h;
                        formats_.push_back(fmt);
                        if(yuv_max_w_ * yuv_max_h_ < w*h) {
                            yuv_max_w_ = w;
                            yuv_max_h_ = h;
                        }
                    }
                    else if(fmt == AIMAGE_FORMAT_RAW16) {
                        if(raw_w_ != 0 || raw_h_ != 0)
                            continue;
                        raw_w_ = w;
                        raw_h_ = h;
                        ACameraMetadata_const_entry maxv = ACameraMetadata_const_entry();
                        camera_status_t res = ACameraMetadata_getConstEntry(meta_,ACAMERA_SENSOR_INFO_WHITE_LEVEL,&maxv);
                        if(res != ACAMERA_OK) {
                            log_status("ACAMERA_SENSOR_INFO_WHITE_LEVEL",res);
                            continue;
                        }
                        raw_max_ = maxv.data.i32[0];
                        ACameraMetadata_const_entry bayer = ACameraMetadata_const_entry();
                        res = ACameraMetadata_getConstEntry(meta_,ACAMERA_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT,&bayer);
                        if(res != ACAMERA_OK) {
                            log_status("ACAMERA_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT",res);
                            continue;
                        }
                        switch(bayer.data.u8[0]) {
                        case ACAMERA_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_RGGB: bayer_ = bayer_rg; break;
                        case ACAMERA_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_GRBG: bayer_ = bayer_gr; break;
                        case ACAMERA_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_GBRG: bayer_ = bayer_gb; break;
                        case ACAMERA_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_BGGR: bayer_ = bayer_bg; break;
                        default:
                            LOG("Unsupported bayer pattern %d\n",bayer.data.u8[0]);
                            continue;
                        }
                        LOG("Adding raw16 for %dx%d bayer=%s maxv=%d\n",w,h,bayer_type_to_str(bayer_).c_str(),raw_max_);
                        static const int num_den[5][2] = { {1,1}, {2,3}, {1,2}, {1,3}, {1,4}};
                        for(int roi=0;roi<=4;roi++) {
                            CamStreamFormat fmt;
                            fmt.format = stream_raw16;
                            int num = num_den[roi][0];
                            int den = num_den[roi][1];
                            int width  =  num * w / den / 2 * 2;
                            int height =  num * h / den / 2 * 2;
                            fmt.width = width;
                            fmt.height = height;
                            fmt.roi_num = num;
                            fmt.roi_den = den;
                            formats_.push_back(fmt);
                            CamStreamFormat fmt2 = fmt;
                            fmt2.width = width / 4 * 2;
                            fmt2.height = height / 4 * 2;
                            fmt2.bin = 2;
                            formats_.push_back(fmt2);
                            CamStreamFormat fmt3 = fmt;
                            fmt3.width = width / 6 * 2;
                            fmt3.height = height / 6 * 2;
                            fmt3.bin = 3;
                            formats_.push_back(fmt3);
                        }
                    }
                }
                for(int bin=2;bin<=3;bin++) {
                    CamStreamFormat fmt;
                    fmt.format = stream_mono16;
                    fmt.width  = yuv_max_w_ / bin;
                    fmt.height = yuv_max_h_ / bin;
                    fmt.bin = bin;
                    formats_.push_back(fmt);
                }
            }
            catch(std::exception const &err)  {
                e = err.what();
                formats_.clear();
            }
            sort_formats();
            return formats_;
        }
/*
        std::pair<cv::Mat,CamBayerType> to_rgb(std::pair<cv::Mat,CamBayerType> const &raw)
        {
            cv::Mat rgb,result;
            cv::Mat bayer = raw.first;
            switch(raw.second) {
            case bayer_rg:  cv::cvtColor(bayer,rgb,cv::COLOR_BayerBG2BGR); break; // COLOR_BayerRGGB2BGR = COLOR_BayerBG2BGR
            case bayer_gr:  cv::cvtColor(bayer,rgb,cv::COLOR_BayerGB2BGR); break; // COLOR_BayerGRBG2BGR = COLOR_BayerGB2BGR
            case bayer_bg:  cv::cvtColor(bayer,rgb,cv::COLOR_BayerRG2BGR); break; // COLOR_BayerBGGR2BGR = COLOR_BayerRG2BGR
            case bayer_gb:  cv::cvtColor(bayer,rgb,cv::COLOR_BayerGR2BGR); break; // COLOR_BayerGBRG2BGR = COLOR_BayerGR2BGR
            default:
                throw std::runtime_error("Invalid bayer patter");
            }
            if(format_.bin == 1)
                return std::make_pair(rgb,bayer_na);
            cv::resize(rgb,result,cv::Size(format_.width,format_.height),0.0,0.0,cv::INTER_AREA);
            return std::make_pair(result,bayer_na);
        }
*/
        void mono_bin2(unsigned char *p,int stride,int rows,int cols,uint16_t *out)
        {
            for(int r=0,r2=0;r<rows;r++,r2+=2,p+= 2 * stride) {
                for(int c=0,c2=0;c<cols;c++,c2+=2) {
                    *out ++ = (uint16_t(p[c2]) + p[c2+1] + p[c2+stride] + p[c2+stride + 1])*64;
                }
            }
        }
        void mono_bin3(unsigned char *p,int stride,int rows,int cols,uint16_t *out)
        {
            int stride2 = stride * 2;
            for(int r=0,r3=0;r<rows;r++,r3+=3,p+= 3 * stride) {
                for(int c=0,c3=0;c<cols;c++,c3+=3) {
                    *out ++ = (uint16_t(p[c3]) +       p[c3+1] +         p[c3+2]
                                      + p[c3+stride] + p[c3+stride + 1] +p[c3+stride +2]
                                      + p[c3+stride2]+ p[c3+stride2+ 1] +p[c3+stride2+2]
                                      )*28;
                }
            }
        }
        void handle_frame(void *data,int len)
        {
            struct timeval tv;
            gettimeofday(&tv,nullptr);
            CamFrame frm;
            frm.unix_timestamp = tv.tv_sec;
            frm.unix_timestamp += tv.tv_usec * 1e-6;
            frm.width = format_.width;
            frm.height = format_.height;
            frm.format = format_.format;
            if(format_.format == stream_mjpeg) {
                frm.data = data;
                frm.data_size = len;
            }
            else if(format_.format == stream_mono8) {
                int size = format_.width * format_.height;
                if(len < size) {
                    LOG("Invalid frame size for mono8 %d, expecting at least %d for %dx%d\n",len,size,format_.width,format_.height);
                    handle_error("Invalid frame size");
                    return;
                }
                frm.data = data;
                frm.data_size = size;
            }
            else if(format_.format == stream_mono16) {
                int size = yuv_max_w_ * yuv_max_h_;
                if(len < size) {
                    LOG("Invalid frame size for mono16 %d, expecting at least %d \n",len,size);
                    handle_error("Invalid frame size");
                    return;
                }
                if(yuv_max_h_ / format_.bin != format_.height || yuv_max_w_ / format_.bin != format_.width) {
                    LOG("Image size mistmatch for mono16 from yuv\n");
                    handle_error("Image size mistmatch for mono16 from yuv");
                    return;
                }
                cv::Mat in(yuv_max_h_,yuv_max_w_,CV_8UC1,data);
                out_.create(format_.height,format_.width,CV_16UC1);
                switch(format_.bin) {
                case 2: mono_bin2(in.data,in.step[0],format_.height,format_.width,(uint16_t*)out_.data); break;
                case 3: mono_bin3(in.data,in.step[0],format_.height,format_.width,(uint16_t*)out_.data); break;
                default:
                    LOG("Image size mistmatch for mono16 from yuv\n");
                    handle_error("Image size mistmatch for mono16 from yuv");
                    return;
                }
                frm.data = out_.data;
                frm.data_size = out_.rows*out_.cols*2;
            }
            else if(format_.format == stream_raw16) {
                int width  = format_.width  * format_.bin;
                int height = format_.height * format_.bin;
                int top    = (raw_h_ - height) / 4 * 2;
                int left   = (raw_w_ -  width) / 4 * 2;
                int size = raw_w_ * raw_h_ * 2;
                if(len < size) {
                    LOG("Invalid frame size for raw16 %d, expecting at least %d\n",len,size);
                    handle_error("Invalid frame size");
                    return;
                }
                cv::Mat in_actual(raw_h_,raw_w_,CV_16UC1,data,raw_w_ * 2);
                cv::Rect roi(left,top,width,height);
                cv::Mat in = in_actual(roi);
                int scale = 65535 / raw_max_;
                if(format_.bin == 2) {
                    apply_bin2(in,out_,scale,0);
                }
                else if(format_.bin == 3) {
                    apply_bin3(in,out_,scale,0);
                }
                else {
                    out_ = in.mul(cv::Scalar::all(scale));
                }
                frm.data = out_.data;
                frm.data_size = out_.rows*out_.cols*2;
                frm.bayer = bayer_;
            }
            else {
                LOG("Internal error invalid format\n");
                handle_error("Invalid format");
                return;
            }

            frm.frame_counter = frame_counter_++;
            {
                std::unique_lock<std::mutex> guard(lock_);
                if(callback_) {
                    callback_(frm);
                }
            }
        }
        void handle_error(const char *msg)
        {
            CamFrame frm = CamFrame();
            frm.format = stream_error;
            frm.data = msg;
            frm.data_size = strlen(msg);
            std::unique_lock<std::mutex> guard(lock_);
            if (callback_)
            {
                callback_(frm);
            }
        }

        void image_callback(AImageReader *reader)
        {
            AImage *image = nullptr;
            auto status = AImageReader_acquireNextImage(reader, &image);
            if(status != AMEDIA_OK) {
                LOG("Failed to acquite image media_status code=%d\n",int(status));
                return;
            }
            try {
                uint8_t *data = nullptr;
                int len = 0;
                if(AImage_getPlaneData(image, 0, &data, &len) != AMEDIA_OK) {
                    LOG("Failed to get plane data media_status code=%d\n",int(status));
                }
                else {
                    handle_frame(data,len);
                }
                
            }
            catch(std::exception const &e) {
                LOG("Error %s\n",e.what());
            }
            AImage_delete(image);
        }


        void configure_capture()
        {
            int fmt,w = format_.width,h = format_.height;
            switch(format_.format){
            case stream_mjpeg: 
                fmt = AIMAGE_FORMAT_JPEG;        
                break;
            case stream_mono8:
                fmt = AIMAGE_FORMAT_YUV_420_888; 
                break;
            case stream_mono16:
                fmt = AIMAGE_FORMAT_YUV_420_888; 
                w = yuv_max_w_;
                h = yuv_max_h_;
                break;
            case stream_raw16: 
                fmt = AIMAGE_FORMAT_RAW16;       
                w = raw_w_;
                h = raw_h_;
                break;
            default:
                throw std::runtime_error("Invalid frame format requested");
            }

            media_status_t status = AImageReader_new(w,h,fmt,4,&reader_);
            if(status != 0) {
                throw std::runtime_error("AImageReader_new returned error code: " + std::to_string(int(status)));
            }

            AImageReader_ImageListener listener = AImageReader_ImageListener();
            listener.context = this;
            listener.onImageAvailable = ac_image_callback;
            AImageReader_setImageListener(reader_,&listener);
            AImageReader_getWindow(reader_,&window_);

            camera_status_t res;

            res = ACaptureSessionOutputContainer_create(&output_container_);
            check("ACaptureSessionOutputContainer_create",res);
            ANativeWindow_acquire(window_);

            res = ACaptureSessionOutput_create(window_,&output_);
            check("ACaptureSessionOutput_create",res);

            res = ACaptureSessionOutputContainer_add(output_container_,output_);
            check("ACaptureSessionOutputContainer_add",res);

            res = ACameraOutputTarget_create(window_,&output_target_);
            check("ACameraOutputTarget_create",res);

            res = ACaptureRequest_addTarget(request_,output_target_);
            check("ACaptureRequest_addTarget",res);
            
            start_catpure();
        }

        void start_catpure()
        {
            camera_status_t res;
            ACameraCaptureSession_stateCallbacks capture_callbacks = ACameraCaptureSession_stateCallbacks();
            capture_callbacks.onClosed = ac_cap_closed;
            capture_callbacks.onReady  = ac_cap_ready;
            capture_callbacks.onActive = ac_cap_active;

            res = ACameraDevice_createCaptureSession(dev_,output_container_,&capture_callbacks,&capture_session_);
            check("ACameraDevice_createCaptureSession",res);

            res = ACameraCaptureSession_setRepeatingRequest(capture_session_,nullptr,1,&request_,nullptr);
            check("ACameraCaptureSession_setRepeatingRequest",res);
        }

        void stop_capture()
        {
            if(capture_session_) {
                ACameraCaptureSession_stopRepeating(capture_session_);
                ACameraCaptureSession_close(capture_session_);
                capture_session_ = nullptr;
            }
        }

        void cleanup_capture()
        {
            stop_capture();
            if(output_target_) {
                ACaptureRequest_removeTarget(request_,output_target_);
                ACameraOutputTarget_free(output_target_);
                output_target_ = nullptr;
            }
            if(output_) {
                ACaptureSessionOutputContainer_remove(output_container_,output_);
                ACaptureSessionOutput_free(output_);
                output_ = nullptr;
            }
            if(window_) {
                ANativeWindow_release(window_);
                window_ = nullptr;
            }
            if(output_container_) {
                ACaptureSessionOutputContainer_free(output_container_);
                output_container_ = nullptr;
            }
            if(reader_) {
                AImageReader_delete(reader_);
                reader_ = nullptr;
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
            format_ = format;
            {
                std::unique_lock<std::mutex> guard(lock_);
                callback_ = callback;
            }
            stream_active_ = 1;

            try {
                configure_capture();
            }
            catch(std::exception const &er) {
                cleanup_capture();
                e=er.what();
                {
                    std::unique_lock<std::mutex> guard(lock_);
                    stream_active_ = 0;
                    callback_ = nullptr;
                }
            }
        }


        /// stop the stream - once function ends callback will not be called any more
        virtual void stop_stream(CamErrorCode &)
        {
            if(stream_active_ == 0)
                return;
            {
                std::unique_lock<std::mutex> guard(lock_);
                callback_ = nullptr;
            }

            cleanup_capture();
            
            /// stop capture there
            stream_active_=0;
        }
        
        void make_modes(std::vector<std::string> &supported_names,std::vector<uint8_t> &codes,ACameraMetadata_const_entry &entry,std::vector<std::string> const &names)
        {
            for(unsigned i=0;i<entry.count;i++) {
                unsigned v = entry.data.u8[i];
                if(v>=names.size())
                    continue;
                supported_names.push_back(names[v]);
                codes.push_back(v);
            }
        }

        /// list of camera controls that the camera supports
        virtual std::vector<CamOptionId> supported_options(CamErrorCode &/*ec*/)
        {
            std::vector<CamOptionId> opts;

            ACameraMetadata_const_entry entry;
            camera_status_t res = ACAMERA_OK;

            entry = ACameraMetadata_const_entry();
            res = ACaptureRequest_getConstEntry(request_,ACAMERA_CONTROL_AE_MODE,&entry);
            log_status("AE_MODE",res);
            if(res == ACAMERA_OK) {
                opts.push_back(opt_auto_exp);
            }
            entry = ACameraMetadata_const_entry();
            res = ACameraMetadata_getConstEntry(meta_,ACAMERA_SENSOR_INFO_EXPOSURE_TIME_RANGE, &entry);
            log_status("Exposre time",res);
            if(res == ACAMERA_OK) {
                opts.push_back(opt_exp);
            }
            
            entry = ACameraMetadata_const_entry();
            res = ACameraMetadata_getConstEntry(meta_,ACAMERA_SENSOR_INFO_SENSITIVITY_RANGE, &entry);
            log_status("ISO",res);
            if(res == ACAMERA_OK) {
                opts.push_back(opt_iso);
            }


            entry = ACameraMetadata_const_entry();
            res = ACameraMetadata_getConstEntry(meta_,ACAMERA_CONTROL_ZOOM_RATIO_RANGE, &entry);
            log_status("ZOOM",res);
            if(res == ACAMERA_OK) {
                opts.push_back(opt_zoom);
                max_crop_region_= -1.0f;
            }
            else {
                entry = ACameraMetadata_const_entry();
                res = ACameraMetadata_getConstEntry(meta_,ACAMERA_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM, &entry);
                log_status("Max digital ZOOM",res);
                if(res == ACAMERA_OK) {
                    opts.push_back(opt_zoom);
                    max_crop_region_ = entry.data.f[0];
                }
            }
            

            entry = ACameraMetadata_const_entry();
            res = ACaptureRequest_getConstEntry(request_,ACAMERA_CONTROL_AWB_MODE ,&entry);
            log_status("AWB_MODE",res);
            if(res == ACAMERA_OK) {
                opts.push_back(opt_wb);
            }

            entry = ACameraMetadata_const_entry();
            res = ACameraMetadata_getConstEntry(meta_,ACAMERA_LENS_INFO_MINIMUM_FOCUS_DISTANCE, &entry);
            log_status("ACAMERA_LENS_INFO_MINIMUM_FOCUS_DISTANCE",res);
            if(res == ACAMERA_OK) {
                min_fd_ = entry.data.f[0];
            }
            entry = ACameraMetadata_const_entry();
            res = ACaptureRequest_getConstEntry(request_,ACAMERA_CONTROL_AF_MODE,&entry);
            log_status("ACAMERA_CONTROL_AF_MODE",res);
            if(res == ACAMERA_OK && min_fd_ > 0) {
                opts.push_back(opt_auto_focus);
                opts.push_back(opt_focus);
            }

            return opts;
        }
        void get_array_size(ACameraMetadata_const_entry &entry)
        {
            camera_status_t res = ACAMERA_OK;
            entry = ACameraMetadata_const_entry();
            res = ACameraMetadata_getConstEntry(meta_,ACAMERA_DISTORTION_CORRECTION_MODE, &entry);
            LOG("get ACAMERA_DISTORTION_CORRECTION_MODE=%d\n",res);
            bool use_pre = (res == ACAMERA_OK && entry.data.u8[0] != ACAMERA_DISTORTION_CORRECTION_MODE_OFF);
            entry = ACameraMetadata_const_entry();
            res = ACameraMetadata_getConstEntry(meta_,
                    (use_pre ? ACAMERA_SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE : ACAMERA_SENSOR_INFO_ACTIVE_ARRAY_SIZE),
                    &entry);
            check("ACTIVE_ARRAY_SIZE",res);
        }
        /// get camera control
        virtual CamParam get_parameter(CamOptionId id,bool current_only,CamErrorCode &e) 
        {
            try {
                CamParam r;
                r.option = id;
                ACameraMetadata_const_entry entry = ACameraMetadata_const_entry();
                camera_status_t res = ACAMERA_OK;
                switch(id) {
                case opt_auto_exp:
                    res = ACaptureRequest_getConstEntry(request_,ACAMERA_CONTROL_AE_MODE,&entry);
                    check("ACAMERA_CONTROL_AE_MODE",res);
                    r.type = type_bool;
                    r.min_val = 0;
                    r.max_val = 1;
                    r.step_size = 1;
                    r.cur_val = entry.data.u8[0] != ACAMERA_CONTROL_AE_MODE_OFF ? 1 : 0;
                    r.def_val = r.cur_val; // no way to query if auto is default
                    return r;
                case opt_exp:
                    res = ACaptureRequest_getConstEntry(request_,ACAMERA_SENSOR_EXPOSURE_TIME,&entry);
                    check("ACAMERA_CONTROL_AE_MODE",res);
                    r.cur_val = entry.data.i64[0] * 1e-6;
                    r.type = type_msec;
                    if(!current_only) {
                        entry = ACameraMetadata_const_entry();
                        res = ACameraMetadata_getConstEntry(meta_,ACAMERA_SENSOR_INFO_EXPOSURE_TIME_RANGE, &entry);
                        check("ACAMERA_SENSOR_INFO_EXPOSURE_TIME_RANGE",res);
                        r.min_val = entry.data.i64[0] * 1e-6;
                        r.max_val = entry.data.i64[1] * 1e-6;
                        r.step_size = 1e-3;
                        r.def_val = r.cur_val; // no way to query if auto is default
                    }
                    return r;
                case opt_zoom:
                    if(max_crop_region_ > 0) {
                        get_array_size(entry);
                        int left   = entry.data.i32[0];
                        int top    = entry.data.i32[1];
                        int width  = entry.data.i32[2];
                        int height = entry.data.i32[3];

                        LOG("Active array size=(%d,%d,%d,%d)\n",left,top,width,height);

                        entry = ACameraMetadata_const_entry();
                        res = ACaptureRequest_getConstEntry(request_,ACAMERA_SCALER_CROP_REGION,&entry);
                        check("ACAMERA_SCALER_CROP_REGION",res);
                        LOG("Current crop: (%d,%d,%d,%d)\n",entry.data.i32[0],entry.data.i32[1],entry.data.i32[2],entry.data.i32[3]);
                        int cur_width = entry.data.i32[2];
                        double scale = double(width) / cur_width;
                        r.cur_val = scale;
                        r.min_val = 1;
                        r.def_val = 1;
                        r.max_val = max_crop_region_;
                    }
                    else {
                        res = ACaptureRequest_getConstEntry(request_,ACAMERA_CONTROL_ZOOM_RATIO,&entry);
                        check("ACAMERA_CONTROL_ZOOM_RATIO",res);
                        r.cur_val = entry.data.f[0];
                        r.type = type_number;
                        if(!current_only) {
                            entry = ACameraMetadata_const_entry();
                            res = ACameraMetadata_getConstEntry(meta_,ACAMERA_CONTROL_ZOOM_RATIO_RANGE, &entry);
                            check("ACAMERA_CONTROL_ZOOM_RATIO_RANGE",res);
                            r.min_val = entry.data.f[0];
                            r.max_val = entry.data.f[1];
                            r.step_size = 0.1;
                            r.def_val = r.cur_val; // no way to query if auto is default
                        }
                    }
                    return r;
                case opt_iso:
                    res = ACaptureRequest_getConstEntry(request_,ACAMERA_SENSOR_SENSITIVITY,&entry);
                    check("ACAMERA_SENSOR_SENSITIVITY",res);
                    r.cur_val = entry.data.i32[0];
                    r.type = type_number;
                    if(!current_only) {
                        entry = ACameraMetadata_const_entry();
                        res = ACameraMetadata_getConstEntry(meta_,ACAMERA_SENSOR_INFO_SENSITIVITY_RANGE, &entry);
                        check("ACAMERA_SENSOR_INFO_SENSITIVITY_RANGE1",res);
                        r.min_val = entry.data.i32[0];
                        r.max_val = entry.data.i32[1];
                        r.step_size = 1;
                        r.def_val = r.cur_val; // no way to query if auto is default
                    }
                    return r;
                case opt_wb:
                    {
                        res = ACaptureRequest_getConstEntry(request_,ACAMERA_CONTROL_AWB_MODE ,&entry);
                        check("ACAMERA_CONTROL_AWB_MODE",res);
                        r.type = type_selection;
                        unsigned mode = entry.data.u8[0];
                        auto modes = awb_modes();
                        mode = std::min(mode,unsigned(modes.size()-1));
                        r.cur_val = mode;
                        r.def_val = r.cur_val;
                        r.min_val = 0;
                        r.max_val = modes.size();
                        r.names = modes;
                        return r;
                    }
                case opt_auto_focus:
                    res = ACaptureRequest_getConstEntry(request_,ACAMERA_CONTROL_AF_MODE,&entry);
                    check("ACAMERA_CONTROL_AF_MODE",res);
                    r.type = type_bool;
                    r.min_val = 0;
                    r.max_val = 1;
                    r.step_size = 1;
                    r.cur_val = entry.data.u8[0] != ACAMERA_CONTROL_AF_MODE_OFF ? 1 : 0;
                    r.def_val = r.cur_val; // no way to query if auto is default
                    return r;
                case opt_focus:
                    res = ACaptureRequest_getConstEntry(request_,ACAMERA_LENS_FOCUS_DISTANCE,&entry);
                    check("ACAMERA_LENS_FOCUS_DISTANCE",res);
                    r.cur_val = entry.data.f[0];
                    r.type = type_number;
                    r.def_val = r.cur_val; // no way to query if auto is default
                    r.min_val = 0;
                    r.max_val = min_fd_;
                    r.step_size = 0.001;
                    return r;
                default:
                    throw std::runtime_error("Unsupported parameter");
                }
            }
            catch(std::exception const &er) {
                e = er.what();
            }
            return CamParam();
        }
        static std::vector<std::string> awb_modes()
        {
            return std::vector<std::string>{"off","auto","incandescent","fluorescent","warm-fluorescent","daylight","cloudy-daylight","twilight","shade"};
        }
        /// set camera control
        virtual void set_parameter(CamOptionId id,double value,CamErrorCode &e)
        {
            try {
                camera_status_t res = ACAMERA_OK;
                switch(id) {
                case opt_auto_exp:
                    {
                        uint8_t v=value!= 0 ? ACAMERA_CONTROL_AE_MODE_ON : ACAMERA_CONTROL_AE_MODE_OFF;
                        res = ACaptureRequest_setEntry_u8(request_,ACAMERA_CONTROL_AE_MODE,1,&v);
                        check("set ACAMERA_CONTROL_AE_MODE",res);
                    }
                    break;
                case opt_exp:
                    {
                        int64_t v = value * 1.0e6;
                        res = ACaptureRequest_setEntry_i64(request_,ACAMERA_SENSOR_EXPOSURE_TIME,1,&v);
                        check("set ACAMERA_SENSOR_EXPOSURE_TIME",res);
                    }
                    break;
                case opt_iso:
                    {
                        int32_t v = value;
                        res = ACaptureRequest_setEntry_i32(request_,ACAMERA_SENSOR_SENSITIVITY,1,&v);
                        check("set ACAMERA_SENSOR_SENSITIVITY",res);
                    }
                    break;
                case opt_wb:
                    {
                        uint8_t v = value;
                        res = ACaptureRequest_setEntry_u8(request_,ACAMERA_CONTROL_AWB_MODE,1,&v);
                        check("set ACAMERA_CONTROL_AWB_MODE",res);
                    }
                    break;
                case opt_auto_focus:
                    {
                        uint8_t v = value ? ACAMERA_CONTROL_AF_MODE_CONTINUOUS_PICTURE : ACAMERA_CONTROL_AF_MODE_OFF ;
                        res = ACaptureRequest_setEntry_u8(request_,ACAMERA_CONTROL_AF_MODE,1,&v);
                        check("set ACAMERA_CONTROL_AF_MODE",res);
                    }
                    break;
                case opt_focus:
                    {
                        float v = value;
                        res = ACaptureRequest_setEntry_float(request_,ACAMERA_LENS_FOCUS_DISTANCE,1,&v);
                        check("set ACAMERA_LENS_FOCUS_DISTANCE",res);
                    }
                    break;
                case opt_zoom:
                    if(max_crop_region_ > 0) {
                        ACameraMetadata_const_entry entry;
                        get_array_size(entry);
                        double roi = 1/std::min(value,(double)max_crop_region_);
                        int left   = entry.data.i32[0];
                        int top    = entry.data.i32[1];
                        int width  = entry.data.i32[2];
                        int height = entry.data.i32[3];

                        LOG("Set active array size=(%d,%d,%d,%d)\n",left,top,width,height);

                        entry = ACameraMetadata_const_entry();
                        res = ACaptureRequest_getConstEntry(request_,ACAMERA_SCALER_CROP_REGION,&entry);
                        check("ACAMERA_SCALER_CROP_REGION",res);
                        LOG("Current crop: (%d,%d,%d,%d)\n",entry.data.i32[0],entry.data.i32[1],entry.data.i32[2],entry.data.i32[3]);

                        int new_width = width * roi;
                        int new_height = height * roi;
                        int new_left = (width - new_width) / 2;
                        int new_top  = (height - new_height) / 2;
                        int32_t new_config[4] =  { new_left, new_top, new_width, new_height };
                        LOG("New crop: (%d,%d,%d,%d)\n",new_config[0],new_config[1],new_config[2],new_config[3]);
                        res = ACaptureRequest_setEntry_i32(request_,ACAMERA_SCALER_CROP_REGION,4,new_config);
                        check("set ACAMERA_SCALER_CROP_REGION",res);
                    }
                    else {
                        float v = value;
                        res = ACaptureRequest_setEntry_float(request_,ACAMERA_CONTROL_ZOOM_RATIO,1,&v);
                        check("set ACAMERA_CONTROL_ZOOM_RATIO",res);
                    }
                    break;
                default:
                    throw std::runtime_error("Unsupported");
                }
                if(stream_active_) {
                    stop_capture();
                    start_catpure();
                }
            }
            catch(std::exception const &er) {
                e = er.what();
            }
        }
    private:

        std::string id_;
        std::string name_;
        ACameraManager *manager_; // set in ctor, owned externally
        /// managed in ctor/dtor
        ACameraDevice *dev_ = nullptr;
        ACameraMetadata *meta_ = nullptr;
        ACaptureRequest *request_ = nullptr;

        /// managed by stream start/stop
        ANativeWindow *window_ = nullptr;
        AImageReader *reader_ = nullptr;
        ACameraOutputTarget *output_target_ = nullptr;
        ACaptureSessionOutput *output_ = nullptr;
        ACaptureSessionOutputContainer *output_container_  = nullptr;
        ACameraCaptureSession *capture_session_ = nullptr;

        float min_fd_ = 0.0f;
        float max_crop_region_ = -1.0f;
        CamBayerType bayer_ = bayer_na;
        int raw_w_=0,raw_h_ = 0,raw_max_ = 0;
        int yuv_max_w_ = 0,yuv_max_h_ = 0;
        cv::Mat out_;


        std::atomic<int> stream_active_;
        std::vector<CamStreamFormat> formats_;
        CamStreamFormat format_;
        int frame_counter_ = 0;
        // protected by mutex
        std::mutex lock_;
        frame_callback_type callback_; 
    };
    extern "C" { 
        static void ac_image_callback(void* context, AImageReader* reader)
        {
            static_cast<AndroidCamera*>(context)->image_callback(reader);
        }
    }

    class AndroidDriver : public CameraDriver {
    public:
        AndroidDriver()
        {
            manager_ = ACameraManager_create();
            if(!manager_) {
                throw std::runtime_error("ACameraManager_create failed");
            }
            
        }
        ~AndroidDriver()
        {
            if(manager_)
                ACameraManager_delete(manager_);
        }
        virtual std::vector<std::string> list_cameras(CamErrorCode &e) 
        {
            if(!names_list_.empty()) {
                return names_list_;
            }
            ACameraIdList *id_list = nullptr;
            ACameraMetadata *meta = nullptr;
            try {
                camera_status_t res = ACameraManager_getCameraIdList(manager_, &id_list);
                check("ACameraManager_getCameraIdList",res);
                for(int i=0;i<id_list->numCameras;i++) {
                    camera_status_t res = ACameraManager_getCameraCharacteristics(manager_,id_list->cameraIds[i],&meta);
                    check("ACameraManager_getCameraCharacteristics",res);
                    std::string name = id_list->cameraIds[i];
                    ACameraMetadata_const_entry info = ACameraMetadata_const_entry();
                    if(ACameraMetadata_getConstEntry(meta,ACAMERA_LENS_FACING,&info) == ACAMERA_OK) {
                        auto facing = static_cast<acamera_metadata_enum_android_lens_facing_t>(info.data.u8[0]);
                        if(facing == ACAMERA_LENS_FACING_BACK)
                            name += ":back";
                        else if(facing == ACAMERA_LENS_FACING_FRONT)
                            name += ":front";
                    }
                    camera_list_.push_back(id_list->cameraIds[i]);
                    names_list_.push_back(name);
                    ACameraMetadata_free(meta);
                    meta = nullptr;
                }
            }
            catch(std::exception const &er) {
                e = er.what();
                camera_list_.clear();
                names_list_.clear();
            }
            if(meta)
                ACameraMetadata_free(meta);
            if(id_list)
                ACameraManager_deleteCameraIdList(id_list);

            return names_list_;
        }
        virtual std::unique_ptr<Camera> open_camera(int id,CamErrorCode &e) 
        {
            std::unique_ptr<Camera> cam;
            if(size_t(id) >= camera_list_.size()) {
                e = "No such camera " + std::to_string(id);
                return cam;
            }
            try {
                cam.reset(new AndroidCamera(camera_list_[id],names_list_[id],manager_));
            }
            catch(std::exception const &ex) {
                e = ex.what();
            }
            return cam;
        }
    private:
        ACameraManager *manager_ = nullptr;
        std::vector<std::string> camera_list_;        
        std::vector<std::string> names_list_;
    };
}

extern "C" {

    void ols_set_android_driver_log(char const *log_path,int /*debug*/)
    {
        ols::error_stream = fopen(log_path,"w");
    }

    ols::CameraDriver *ols_get_android_driver(int /*cam_id*/,ols::CamErrorCode *e)
    {
        try {
            return new ols::AndroidDriver();
        }
        catch(std::exception const &er) {
            *e = er.what();
            return nullptr;
        }
    }
}
