#include "camera.h"
#include <gphoto2/gphoto2.h>
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
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#ifdef WITH_LIBRAW
#include <libraw/libraw.h>
#endif

namespace ols {
        
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

        std::pair<cv::Mat,CamBayerType> to_mat()
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
                return load_libraw();
#else
                throw GPError("OpenLiveStacker was build without libraw support");                
#endif                
            }
        }

#ifdef WITH_LIBRAW
        std::string index2color(LibRaw &raw)
        {
            std::string res;
            int pat_index[4] = {raw.COLOR(0,0),raw.COLOR(0,1),raw.COLOR(1,0),raw.COLOR(1,1)};
            for(int i=0;i<4;i++) {
                char const *v="RGBG";
                if(pat_index[i] < 0 || pat_index[i]>3)
                    res += '?';
                else
                    res += v[pat_index[i]];
            }
            return res;
        }
        
        std::pair<cv::Mat,CamBayerType> load_libraw()
        {
            cv::Mat img;
            LibRaw raw;
            int code=0;
            if(0){
                static int counter = 0;
                char name[256];
                snprintf(name,sizeof(name),"/tmp/%05d.raw",counter);
                counter++;
                std::ofstream f(name);
                f.write(const_cast<char *>(data_),size_);
                f.close();
            }

            if((code = raw.open_buffer(const_cast<char *>(data_),size_))!=LIBRAW_SUCCESS
                || (code=raw.unpack())!=LIBRAW_SUCCESS)
            {
                throw GPError("Failed to read RAW file " + name() + ": " + libraw_strerror(code));
            }
            if(!raw.imgdata.rawdata.raw_image) {
                throw GPError("Is not 16 bit bayer:" + name());
            }
            cv::Mat raw_image(raw.imgdata.sizes.raw_height,raw.imgdata.sizes.raw_width,CV_16UC1,raw.imgdata.rawdata.raw_image,raw.imgdata.sizes.raw_pitch);
            int scale = 65535 / raw.imgdata.color.maximum;
            std::string bayer_name = index2color(raw);
            CamBayerType bayer = bayer_type_from_str(bayer_name);
            if(scale != 1)
                img = raw_image.mul(cv::Scalar::all(scale));
            else
                img = raw_image.clone();

            return std::make_pair(img,bayer);
        }
#endif
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

        int wait_event(CameraFilePath files[MAX_FILES])
        {
            int fno = 0;
            for(;;) {
                CameraEventType ev = GP_EVENT_UNKNOWN;
                void *ptr = NULL;
                int status;
                {
                    std::unique_lock<std::recursive_mutex> guard(clock_);
                    status = gp_camera_wait_for_event(cam_,5000,&ev,&ptr,ctx_);
                }
                check(status,"Wait event");
                switch(ev) {
                case GP_EVENT_UNKNOWN:
                    printf("No event\n");
                    continue;
                case GP_EVENT_TIMEOUT:
                    printf("timeout\n");
                    continue;
                case GP_EVENT_FILE_ADDED:
                    if(fno < MAX_FILES) { 
                        CameraFilePath *fp = static_cast<CameraFilePath*>(ptr);
                        files[fno++] = *fp;
                        printf("Got %s/%s\n",fp->folder,fp->name);
                    }
                    break;
                case GP_EVENT_CAPTURE_COMPLETE:
                    printf("Capture done\n");
                    return fno;
                default:
                    printf("other event\n");
                    break;
                }
                free(ptr);
            }
        }

        virtual std::vector<CamStreamFormat> formats(CamErrorCode &e)
        {
            if(!formats_.empty())
                return formats_;
            try {
                trigger();
                CameraFilePath files[MAX_FILES];
                int N = wait_event(files);
                if(N == 0) {
                    e="Failed to capture images";
                    return formats_;
                }
                for(int i=0;i<N;i++) {
                    std::unique_lock<std::recursive_mutex> guard(clock_);
                    GPFile file(files[i],cam_,ctx_);
                    file.get();
                    file.del();
                    CamStreamFormat fmt;
                    if(file.is_jpeg())
                        fmt.format = stream_mjpeg;
                    else {
                        #ifdef WITH_LIBRAW
                        fmt.format = stream_raw16;
                        #else
                        continue;
                        #endif
                    }
                    std::pair<cv::Mat,CamBayerType> res = file.to_mat();
                    fmt.width = res.first.cols;
                    fmt.height = res.first.rows;
                    formats_.push_back(fmt);
                }
            }
            catch(std::exception const &err)  {
                e = err.what();
            }
            return formats_;
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
                    trigger();
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
                                handle_frame(f.to_mat());
                            }
                        }
                        f.del();
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
            default:
                throw GPError("Unsupported camera option");
            }
            return name;
        }
        /// get camera control
        virtual CamParam get_parameter(CamOptionId id,bool /*current_only*/,CamErrorCode &e) 
        {
            try {
                std::string name = opt_to_name(id);
                std::unique_lock<std::recursive_mutex> guard(clock_);
                GPWidget w(cam_,ctx_,name);
                return w.get(id);
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
                std::string name = opt_to_name(id);
                GPWidget w(cam_,ctx_,name);
                w.set(value);
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
        // protected by mutex
        std::mutex lock_;
        std::recursive_mutex clock_;
        frame_callback_type callback_; 
        std::thread thread_;
        bool stop_ = false;

    };
 

    extern "C" {
        static void ctx_error_func(GPContext *, char const *msg, void *)
        {
            fprintf(stderr,"GP2 Error:%s\n",msg);
            fflush(stderr);
        }
        static void ctx_status_func(GPContext *, char const *msg, void *)
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
    ols::CameraDriver *ols_get_gphoto2_driver(int /*cam_id*/,ols::CamErrorCode * /*e*/)
    {
        //std::unique_ptr<ols::CameraDriver> p;
        //if(cam_id != -1)
        //    p.reset(new ols::SingleGP2CameraDriver(cam_id,e));
        //else
        //p.reset(new ols::GP2CameraDriver());
        //return p.release();
        return new ols::GP2CameraDriver();
    }
}
