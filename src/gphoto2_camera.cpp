#include "camera.h"
#include <gphoto2/gphoto2.h>
#include <gphoto2/gphoto2-port.h>
#include <mutex>
#include <sstream>
#include <iostream>
#include <atomic>
#include <map>
#include <cctype>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <algorithm>
#include <queue>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#ifdef WITH_LIBRAW
#include "libraw_wrapper.h"
#endif

namespace ols {

    static FILE *error_stream = NULL;
    extern "C" {
        static void ctx_error_func(GPContext *, char const *msg, void *)
        {
            FILE *f = !error_stream ? stderr : error_stream;
            fprintf(f,"GP2 Error:%s\n",msg);
            fflush(f);
        }
        static void ctx_status_func(GPContext *, char const *msg, void *)
        {
            FILE *f = !error_stream ? stderr : error_stream;
            fprintf(f,"GP2 Status:%s\n",msg);
            fflush(f);
        }
        static void errordumper(GPLogLevel /*level*/, const char *domain, const char *str,
                 void * /*data*/) 
        {
            FILE *f = !error_stream ? stderr : error_stream;
            fprintf(f,"GP2 LOG:%s:%s\n",domain,str);
            fflush(f);
        }
    }

    #define LOG(...) do { if( ::ols::error_stream) { fprintf( ::ols::error_stream,__VA_ARGS__); fflush(::ols::error_stream); }} while(0)

        
    static constexpr int MAX_FILES = 3;

    static bool check(int err,char const *msg,CamErrorCode &e)
    {
        if(err != GP_OK) {
            e = CamErrorCode(std::string(msg) + ":" + gp_result_as_string(err));
            return false;
        }
        return true;
    }

    class GPError : public CamError {
    public:
        GPError(std::string const &msg) : CamError(msg)
        {
        }
        GPError(std::string const &msg,int code) :
            CamError(std::string(msg) + ":" + gp_result_as_string(code))
        {

        }
    };

    static void check(int status,char const *msg)
    {
        if(status != GP_OK) {
            throw GPError(msg,status);
        }
    }

    class GPWidget {
    public:
        GPWidget( ::Camera *c,GPContext *ct,std::string const &key) :
            cam_(c),ctx_(ct),key_(key),widget_(nullptr)
        {
            
        }

        void set(double value)
        {
            load();
            int res;
            CameraWidgetType type;
            res = gp_widget_get_type(widget_,&type);
            check(res,"get_widget_type");
            switch(type) {
            case GP_WIDGET_TOGGLE:
                {
                    int val = value;
                    res = gp_widget_set_value(widget_,&val);
                    check(res,"set value");
                }
                break;
            case GP_WIDGET_RADIO:
            case GP_WIDGET_MENU:
                {
                    char const *val = nullptr;
                    res = gp_widget_get_choice(widget_,int(value),&val);
                    check(res,"widget_get_choice");
                    res = gp_widget_set_value(widget_,val);
                    check(res,"widget set value string");
                }
                break;
            default:
                throw GPError("Unsupported widget type");
            }
            res = gp_camera_set_single_config(cam_,key_.c_str(),widget_,ctx_);
            check(res,"set signle config");
        }

        CamParam get(CamOptionId id)
        {
            load();
            CamParam r;
            r.option = id;
            CameraWidgetType type;
            int res;
            res = gp_widget_get_type(widget_,&type);
            check(res,"get_widget_type");
            switch(type) {
            case GP_WIDGET_TOGGLE:
                {
                    int ival = -1;
                    res = gp_widget_get_value(widget_,&ival);
                    check(res,"widget_get_value");
                    r.type = type_bool;
                    r.step_size = 1;
                    r.min_val = 0;
                    r.max_val = 1;
                    r.def_val = r.cur_val = ival == 1; 
                }
                break;
            case GP_WIDGET_RADIO:
            case GP_WIDGET_MENU:
                {
                    int N = gp_widget_count_choices(widget_);
                    char const *val = nullptr;
                    for(int i=0;i<N;i++) {
                        res = gp_widget_get_choice(widget_,i,&val);
                        check(res,"widget_get_choice");
                        r.names.push_back(val);
                    }
                    gp_widget_get_value(widget_,&val);
                    check(res,"widget_get_value");
                    
                    r.type = type_selection;
                    r.min_val = 0;
                    r.max_val = N-1;
                    r.cur_val = std::find(r.names.begin(),r.names.end(),std::string(val)) - r.names.begin();
                    r.def_val = r.cur_val;
                }
                break;
            default:
                throw GPError("Unsupported widget type " + key_);
            }
            return r;
        }

        bool try_load()
        {
            if(widget_)
                return true;
            int res = gp_camera_get_single_config(cam_,key_.c_str(),&widget_,ctx_);
            if(res < GP_OK)
                return false;
            return true;
        }

        void load()
        {
            if(widget_)
                return;
            int res = gp_camera_get_single_config(cam_,key_.c_str(),&widget_,ctx_);
            check(res,"get config");
        }
        ~GPWidget()
        {
            if(widget_)
                gp_widget_free(widget_);
        }
        
    private:
        ::Camera *cam_;
        GPContext *ctx_;
        std::string key_;
        CameraWidget *widget_;
    };

    class GPFile {
    public:
        GPFile(CameraFilePath const &path,::Camera *camera, GPContext *context) : 
            file_(nullptr),
            path_(path),
            cam_(camera),
            ctx_(context)
        {
        }
        ~GPFile()
        {
            free();
        }
        void free()
        {
            if(file_)
                gp_file_free(file_);
            file_ = nullptr;
            data_ = nullptr;
            size_ = 0;
        }
        std::string name()
        {
            return path_.folder + std::string("/") + path_.name;
        }
        void del()
        {
            LOG("Deleting file %s/%s\n",path_.folder,path_.name);
            int status = gp_camera_file_delete(cam_,path_.folder,path_.name,ctx_);
            check(status,"delete");
        }
        bool loaded()
        {
            return data_ != nullptr;
        }
        void get()
        {
            int status = 0;
            free();
            status = gp_file_new(&file_);
            check(status,"new file");
            status = gp_camera_file_get(cam_,path_.folder,path_.name,GP_FILE_TYPE_NORMAL,file_,ctx_);
            check(status,"get file");
            status = gp_file_get_data_and_size(file_,&data_,&size_);
            check(status,"get data");
        }

        std::pair<cv::Mat,CamBayerType> to_mat(int w=-1,int h=-1,int bin=1)
        {
            std::pair<cv::Mat,CamBayerType> r;
            if(!loaded()) {
                get();
            }
            if(is_jpeg()) {
                cv::Mat buffer(1,size_,CV_8UC1,const_cast<char*>(data_));
                r.first = cv::imdecode(buffer,cv::IMREAD_UNCHANGED);
                r.second = bayer_na;
                return r;
            }
            else {
#ifdef WITH_LIBRAW                
                return load_libraw(const_cast<char*>(data_),size_,h,w,bin);
#else
                throw GPError("OpenLiveStacker was build without libraw support");                
#endif                
            }
        }

        char const *data() {
            return data_;
        }
        size_t size() {
            return size_;
        }
        bool is_jpeg()
        {
            char *cext = strrchr(path_.name,'.');
            if(!cext)
                return false;
            std::string ext(cext+1);
            for(size_t i=0;i<ext.size();i++)
                ext[i] = std::tolower(ext[i]);
            return ext == "jpeg" || ext == "jpg" || ext == "jpe" || ext == "jif" || ext=="jfif" || ext=="jfi";
        }
    private:
        CameraFile *file_;
        CameraFilePath path_;
        ::Camera *cam_;
        GPContext *ctx_;
        char const *data_ = nullptr;
        unsigned long size_ = 0;
    };
    
    class GP2Camera : public Camera {
    public:
        /// for Android
        GP2Camera(::Camera *c,GPContext *ctx):
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
        }
        /// Camera name
        virtual std::string name(CamErrorCode &) 
        {
            return "Camera";
        }
        /// Return list of suppored video formats

        void trigger()
        {
            std::unique_lock<std::recursive_mutex> guard(clock_);
            int status = gp_camera_trigger_capture(cam_,ctx_);
            check(status,"trigger");
        }

        int wait_event(CameraFilePath files[MAX_FILES],bool wait_multiple = false)
        {
            int fno = 0;
            while(fno <= 0 || wait_multiple) {
                if (stop_) {
                    return fno;
                }
                CameraEventType ev = GP_EVENT_UNKNOWN;
                void *ptr = NULL;
                int status;
                {
                    std::unique_lock<std::recursive_mutex> guard(clock_);
                    // keep timeout short such that other calls can go through
                    status = gp_camera_wait_for_event(cam_,500,&ev,&ptr,ctx_);
                }
                check(status,"Wait event");
                switch(ev) {
                case GP_EVENT_UNKNOWN:
                    LOG("EV:Unknown\n");
                    break;
                case GP_EVENT_TIMEOUT:
                    LOG("EV:Timeout\n");
                    if(fno > 0) {
                        return fno;
                    }
                    break;
                case GP_EVENT_FILE_ADDED:
                    if(fno < MAX_FILES) { 
                        CameraFilePath *fp = static_cast<CameraFilePath*>(ptr);
                        files[fno++] = *fp;
                        LOG("Got file %s/%s\n",fp->folder,fp->name);
                    }
                    break;
                case GP_EVENT_CAPTURE_COMPLETE:
                    LOG("EV:Capture Done\n");
                    if (fno > 0) {
                        return fno;
                    }
                    break;
                default:
                    LOG("EV:Other %d\n",int(ev));
                    break;
                }
                free(ptr);
            }
            return fno;
        }

        virtual std::vector<CamStreamFormat> formats(CamErrorCode &e)
        {
            if(!formats_.empty())
                return formats_;
            try {
                trigger();
                CameraFilePath files[MAX_FILES];
                int N = wait_event(files,true);
                if(N == 0) {
                    e="Failed to capture images";
                    return formats_;
                }
                /// first go over images even if many to make sure we clean them up
                for(int i=0;i<N;i++) {
                    std::unique_lock<std::recursive_mutex> guard(clock_);
                    GPFile file(files[i],cam_,ctx_);
                    file.get();
                    file.del();
                    if(file.is_jpeg()) {
                        CamStreamFormat fmt;
                        fmt.format = stream_mjpeg;
                        std::pair<cv::Mat,CamBayerType> res = file.to_mat();
                        fmt.width = res.first.cols;
                        fmt.height = res.first.rows;
                        formats_.push_back(fmt);
                    }
                    else {
                        #ifdef WITH_LIBRAW
                        std::pair<cv::Mat,CamBayerType> res = file.to_mat();
                        for(int is_rgb=0;is_rgb <=1;is_rgb++) {
                            for(int bin=1;bin<=3;bin++) {
                                for(int scale = 0;scale < 8;scale++) {
                                    int num,den;
                                    switch(scale) {
                                    case  0: num=1; den=1; break;
                                    case  1: num=2; den=3; break;
                                    default: num=1; den=scale;
                                    }
                                    int w = num * res.first.cols / den / (2*bin) * 2; // make sure round of 2
                                    int h = num * res.first.rows / den / (2*bin) * 2; // make sure round of 2

                                    if(std::min(w,h) < 480)
                                        break;
                                    
                                    CamStreamFormat fmt;
                                    fmt.format = is_rgb ? stream_rgb48 : stream_raw16;
                                    fmt.width = w;
                                    fmt.height = h;
                                    fmt.bin = bin;
                                    fmt.roi_num = num;
                                    fmt.roi_den = den;
                                    formats_.push_back(fmt);
                                }
                            }
                        }
                        #else
                        continue;
                        #endif
                    }
                }
                if(N > 1) {
                    e = "OLS does not supports multiple image formats for a single image. Please choose either Raw of JPG, but not both.";
                    formats_.clear();
                    return formats_;
                }
            }
            catch(std::exception const &err)  {
                e = err.what();
            }
            return formats_;
        }

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

        void handle_frame(std::pair<cv::Mat,CamBayerType> const &in)
        {
            struct timeval tv;
            gettimeofday(&tv,nullptr);
            CamFrame frm;
            frm.unix_timestamp = tv.tv_sec;
            frm.unix_timestamp += tv.tv_usec * 1e-6;
            frm.width = format_.width;
            frm.height = format_.height;
            frm.data = in.first.data;
            frm.format = format_.format;
            frm.data_size = in.first.total() * in.first.elemSize();
            frm.frame_counter = frame_counter_++;
            frm.bayer = in.second;
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
            stop_=false;
            thread_ = std::move(std::thread([=]() {
                capture_thread();
            }));
        }
        void handle_cmd_queue()
        {
            std::unique_lock<std::recursive_mutex> guard(clock_);
            while(!cmd_queue_.empty()){
                auto cmd = cmd_queue_.front();
                cmd_queue_.pop();
                CamErrorCode e;
                set_parameter_internal(cmd.first,cmd.second,e);
                if(e) {
                    printf("%s\n",e.message().c_str());
                    handle_error(e.message().c_str());
                }
            }
            
        }
        void capture_thread()
        {
            try {
                trigger();
                while(true) {
                    CameraFilePath files[MAX_FILES];
                    int N = wait_event(files);
                    if(stop_) {
                        break;
                    }
                    handle_cmd_queue();

                    bool delay_enabled = capture_delay_ > 0;
                    auto time_before_downloading_files = std::chrono::steady_clock::now();
                    if (!delay_enabled) {
                        trigger();
                    }

                    for(int i=0;i<N;i++) {
                        std::unique_lock<std::recursive_mutex> guard(clock_);
                        GPFile f(files[i],cam_,ctx_);
                        if(format_.format == stream_mjpeg) {
                            if(f.is_jpeg()) {
                                f.get();
                                // pass data as is
                                cv::Mat buffer(1,f.size(),CV_8UC1,const_cast<char*>(f.data()));
                                handle_frame(std::make_pair(buffer,bayer_na));
                            }
                        }
                        else {
                            if(!f.is_jpeg()) {
                                if(format_.format == stream_raw16) {
                                    handle_frame(f.to_mat(format_.width,format_.height,format_.bin));
                                }
                                else if(format_.format == stream_rgb48) {
                                    handle_frame(to_rgb(f.to_mat(format_.width * format_.bin, format_.height * format_.bin))); 
                                }
                            }
                        }
                        if(!keep_images_) {
                            f.del();
                        }
                    }

                    if (delay_enabled) {
                        std::chrono::duration time_elapsed = std::chrono::steady_clock::now() - time_before_downloading_files;
                        std::chrono::duration time_remaining = std::chrono::seconds(capture_delay_) - time_elapsed;
                        std::chrono::duration half_second = std::chrono::milliseconds(500);
                        while (time_remaining.count() > 0) {
                            std::chrono::duration time_to_sleep = time_remaining < half_second ? time_remaining : half_second;
                            std::this_thread::sleep_for(time_to_sleep);
                            if (stop_) {
                                return;
                            }
                            time_remaining -= time_to_sleep;
                        }
                        trigger();
                    }
                }
            }
            catch(std::exception const &e) {
                handle_error(e.what());
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
                stop_ = true;
            }

            /// stop capture there
            stream_active_=0;
            thread_.join();
            stop_ = false;
        }

        /// list of camera controls that the camera supports
        virtual std::vector<CamOptionId> supported_options(CamErrorCode &)
        {
            std::unique_lock<std::recursive_mutex> guard(clock_);
            std::vector<CamOptionId> opts;
            {
                GPWidget w(cam_,ctx_,"iso");
                if(w.try_load())
                    opts.push_back(opt_iso);
            }
            {
                GPWidget w(cam_,ctx_,"shutterspeed");
                if(w.try_load())
                    opts.push_back(opt_shutter);
            }
            {
                GPWidget w(cam_,ctx_,"viewfinder");
                if(w.try_load())
                    opts.push_back(opt_viewfinder);
            }
            {
                GPWidget w(cam_,ctx_,"capturetarget");
                if(w.try_load())
                    opts.push_back(opt_capturetarget);
            }
            opts.push_back(opt_keep_images);
            opts.push_back(opt_capture_delay);
            return opts;
        }
        std::string opt_to_name(CamOptionId id)
        {
            std::string name;
            switch(id) {
            case opt_iso:
                name = "iso";
                break;
            case opt_shutter:
                name = "shutterspeed";
                break;
            case opt_viewfinder:
                name = "viewfinder";
                break;
            case opt_capturetarget:
                name = "capturetarget";
                break;
            default:
                throw GPError("Unsupported camera option");
            }
            return name;
        }
        /// override to prevent loosing mutex
        virtual std::vector<CamParam> get_all_parameters(bool current_only,CamErrorCode &e)
        {
            std::unique_lock<std::recursive_mutex> guard(clock_);
            return Camera::get_all_parameters(current_only,e);
        }
        /// get camera control
        virtual CamParam get_parameter(CamOptionId id,bool /*current_only*/,CamErrorCode &e) 
        {
            try {
                switch(id) {
                case opt_keep_images:
                    {
                        CamParam r;
                        r.option = id;
                        r.type = type_bool;
                        r.min_val = 0;
                        r.max_val = 1;
                        r.step_size = 1;
                        r.cur_val = keep_images_;
                        r.def_val = 0;
                        return r;
                    }
                    break;
                case opt_capture_delay:
                    {
                        CamParam r;
                        r.option = id;
                        r.type = type_number;
                        r.min_val = 0;
                        r.max_val = 30;
                        r.step_size = 1;
                        r.cur_val = capture_delay_;
                        r.def_val = 0;
                        return r;
                    }
                default:
                    {
                        std::string name = opt_to_name(id);
                        std::unique_lock<std::recursive_mutex> guard(clock_);
                        GPWidget w(cam_,ctx_,name);
                        return w.get(id);
                    }
                }
            }
            catch(std::exception const &er) {
                e = er.what();
            }
            return CamParam();
        }
        /// set camera control
        virtual void set_parameter(CamOptionId id,double value,CamErrorCode &e)
        {
            if(stream_active_) {
                std::unique_lock<std::recursive_mutex> guard(clock_);
                cmd_queue_.push(std::make_pair(id,value));
            }
            else {
                std::unique_lock<std::recursive_mutex> guard(clock_);
                set_parameter_internal(id,value,e);
            }
        }
        void set_parameter_internal(CamOptionId id,double value,CamErrorCode &e)
        {
            try {
                switch(id) {
                case opt_keep_images:
                    {
                        keep_images_ = !!value;
                    }
                    break;
                case opt_capture_delay:
                    {
                        capture_delay_ = value;
                    }
                    break;
                default:
                    {
                        std::string name = opt_to_name(id);
                        GPWidget w(cam_,ctx_,name);
                        w.set(value);
                    }
                }
            }
            catch(std::exception const &er) {
                e = er.what();
            }
        }
    private:
        std::queue<std::pair<CamOptionId,double>> cmd_queue_;
        ::Camera *cam_;
        GPContext *ctx_;
        std::atomic<int> stream_active_;
        std::vector<CamStreamFormat> formats_;
        CamStreamFormat format_;
        int frame_counter_ = 0;
        int keep_images_ = 0;
        int capture_delay_ = 0;
        // protected by mutex
        std::mutex lock_;
        std::recursive_mutex clock_;
        frame_callback_type callback_; 
        std::thread thread_;
        bool stop_ = false;

    };
 

    class GP2CameraDriver : public CameraDriver {
    public:
        virtual std::vector<std::string> list_cameras(CamErrorCode &e) 
        {
            if(!camera_list_.empty()) {
                return camera_list_;
            }
            gp_camera_new(&camera_);
            int ret = gp_camera_init(camera_,ctx_);
            if(!check(ret,"init",e)) {
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
            LOG("context created\n");
        }
        ~GP2CameraDriver()
        {
            if(camera_)
                gp_camera_exit(camera_,ctx_);
        }
    private:
        GPContext *ctx_ = nullptr;
        ::Camera *camera_ = nullptr;
        bool has_camera_ = false;
        std::vector<std::string> camera_list_;        
    };
}

extern "C" {

    void ols_set_gphoto2_driver_log(char const *log_path,int debug)
    {
        ols::error_stream = fopen(log_path,"w");
        gp_log_add_func(debug ? GP_LOG_DEBUG : GP_LOG_ERROR , ols::errordumper, NULL);
    }

#ifdef ANDROID_SUPPORT
    int ols_set_gphoto2_driver_config(char const *libdir)
    {
        setenv("CAMLIBS",libdir,1);
        setenv("IOLIBS", libdir,1);
        setenv("CAMLIBS_PREFIX","libgphoto2_camlib_",1);
        setenv("IOLIBS_PREFIX","libgphoto2_port_iolib_",1);
        LOG("Gphoto libdir %s\n",libdir);
        return 0;
    }
   
    ols::CameraDriver *ols_get_gphoto2_driver(int fd,ols::CamErrorCode *e)
    {
        if(fd >= 0) {
            LOG("Using file descriptor for USB device %d\n",fd);
            int status = gp_port_usb_set_sys_device(fd);
            if(!ols::check(status,"gp_port_usb_set_sys_device",*e))
                return nullptr;
        }
        return new ols::GP2CameraDriver();
    }
#else    
    ols::CameraDriver *ols_get_gphoto2_driver(int /*cam_id*/,ols::CamErrorCode * /*e*/)
    {
        return new ols::GP2CameraDriver();
    }
#endif    
}
