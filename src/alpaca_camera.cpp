#include "camera.h"
#include "alpaca_client.h"
#include <mutex>
#include <sstream>
#include <iostream>
#include <atomic>
#include <map>
#include <string.h>
#include "os_util.h"
#include <thread>
#include <algorithm>
#include <opencv2/core.hpp>


namespace ols {
    static FILE *error_stream = NULL;
    #define LOG(...) do { if( ::ols::error_stream) { \
        struct timeval tv; gettimeofday(&tv,nullptr); \
        int now = tv.tv_sec  % (24*3600); \
        fprintf( ::ols::error_stream,"%02d:%02d:%02d.%03d  ",now/3600,(now/60)%60,now%60,int(tv.tv_usec / 1000)); \
        fprintf( ::ols::error_stream,__VA_ARGS__); fflush(::ols::error_stream); \
    }} while(0)

    #define LOGE(except) LOG("Exception %s\n",except.what())
    
    typedef std::unique_lock<std::recursive_mutex> guard_type;

    class AlpacaCamera : public Camera {
    public:
        /// for Android
        AlpacaCamera(std::string const &url,int id,std::string const &name) :
            client_(url,"camera"),
            name_(name)
        {
            client_.set_device(id);
            stream_active_ = 0;
            stop_ = 0;
            client_.set_logf(error_stream);
            client_.connect();
        }
        virtual ~AlpacaCamera()
        {
            CamErrorCode e;
            stop_stream(e);
            if(e) {
                LOG("Failed to close stream %s\n",e.message().c_str());
            }
        }
        /// Camera name
        virtual std::string name(CamErrorCode &) 
        {
            return name_;
        }
        /// Return list of suppored video formats
        virtual std::vector<CamStreamFormat> formats(CamErrorCode &e)
        {
            guard_type g(mutex_);

            if(!formats_.empty())
                return formats_;
            try {
                int sensortype = client_.get_value<int>("/sensortype");
                int width = client_.get_value<int>("/cameraxsize");
                int height = client_.get_value<int>("/cameraysize");
                int max_binx = client_.get_value<int>("/maxbinx");
                int max_biny = client_.get_value<int>("/maxbiny");
                int max_bin = std::min(max_binx,max_biny);
                exp_min_ = client_.get_value<double>("/exposuremin");
                exp_max_ = client_.get_value<double>("/exposuremax");
                cur_exp_  = std::max(exp_min_,std::min(exp_max_,0.1));
                can_abort_ = client_.get_value<bool>("/canabortexposure");
                
                CamStreamFormat fmt;
                if(sensortype == 0) {
                    fmt.format = stream_mono16;
                    bayer_ = bayer_na;
                }
                else if(sensortype == 1) {
                    fmt.format = stream_rgb48;
                    bayer_ = bayer_na;
                }
                else if(sensortype == 2) {
                    int bx = client_.get_value<int>("/bayeroffsetx");
                    int by = client_.get_value<int>("/bayeroffsety");
                    LOG("Bayer offsets %d/%d\n",bx,by);
                    if(bx == 0) {
                        if(by == 0)
                            bayer_ = bayer_rg;
                        else
                            bayer_ = bayer_gb;
                    }
                    else {
                        if(by == 0)
                            bayer_ = bayer_gr;
                        else
                            bayer_ = bayer_bg;
                    }
                    fmt.format = stream_raw16;
                }
                else {
                    throw std::runtime_error("Unsupported sensor type " + std::to_string(sensortype));
                }

                for(int bin=1;bin<max_bin;bin++) {
                    int divisor = sensortype == 2 ? 2 : 1;
                    if(bin == 1)
                        divisor = 1;
                    fmt.width = width / (bin * divisor) * divisor;
                    fmt.height = height / (bin * divisor) * divisor;
                    fmt.bin = bin;
                    formats_.push_back(fmt);
                }
            }
            catch(std::exception const &ex) {
                LOGE(ex);
                formats_.clear();
                e = ex;
            }
            return formats_;
        }

        void handle_format_error(CamStreamFormat format,cv::Mat &img)
        {
            std::ostringstream ss;
            ss << "Format mistmatch format=" << format << " img = " << img.rows << "x" << img.cols << " ch=" << (img.elemSize() / img.elemSize1()) << " pix bytes = " << img.elemSize1();
            handle_error(ss.str());
        }
        void handle_error(std::string const &er)
        {
            LOG("Stream Error %s\n",er.c_str());
            std::string msg = er;
            CamFrame frm;
            frm.format = stream_error;
            frm.data = &msg[0];
            frm.data_size = msg.size();
            {
                guard_type g(mutex_);
                if(callback_) {
                    callback_(frm);
                }
            }
        }

        void handle_frame(CamStreamFormat format,cv::Mat &img)
        {
            struct timeval tv;
            gettimeofday(&tv,nullptr);
            CamFrame frm;
            frm.unix_timestamp = tv.tv_sec;
            frm.unix_timestamp += tv.tv_usec * 1e-6;
            bool format_mistmatch = false;
            if(img.rows != format.height || img.cols != format.width)
                format_mistmatch = true;
            switch(format.format) {
            case stream_raw16:
            case stream_mono16:
                if(img.elemSize1() != 2 || img.elemSize() != 2)
                    format_mistmatch = true;
                break;
            case stream_rgb48:
                if(img.elemSize1() != 2 || img.elemSize() != 6)
                    format_mistmatch = true;
                break;
            default:
                format_mistmatch = true;
            }
            if(format_mistmatch) {
                handle_format_error(format,img);
                return;
            }
            frm.width = format.width;
            frm.height = format.height;
            frm.data = img.data;
            frm.data_size =  img.total() * img.elemSize();
            frm.frame_counter = frame_counter_++;
            frm.format = format.format;
            frm.bayer = bayer_;
            LOG("BAYER %s\n",bayer_type_to_str(frm.bayer).c_str());
            {
                guard_type g(mutex_);
                if(callback_) {
                    callback_(frm);
                }
            }
        }

        bool wait_image(double exp,bool check_stop)
        {
            double passed = 0;
            do {
                double sleep_limit = std::min(exp - passed,0.5);
                usleep(int(sleep_limit * 1e6));
                passed += sleep_limit;
            } while(passed < exp && !(check_stop && stop_));

            if(stop_)
                return false;
            while(!(check_stop && stop_)){
                {
                    guard_type g(mutex_);
                    if(client_.get_value<bool>("/imageready"))
                        return true;
                }
                usleep(100000);
            }
            return false;
        }
        /// Start a video stream with provided callback 
        void thread_loop(CamStreamFormat format)
        {
            try {
                cv::Mat buffer;
                while(!stop_) {
                    double exp;
                    bool can_abort;
                    {
                        guard_type g(mutex_);
                        exp = cur_exp_;
                        can_abort = can_abort_;
                        client_.put("/startexposure",{
                            {"Duration",std::to_string(exp)},
                            {"Light","true"}
                        });
                    }
                    if(wait_image(exp,true)) {
                        {
                            guard_type g(mutex_);
                            client_.get_binary_image(buffer);
                        }
                        handle_frame(format,buffer);
                    }
                    else if(can_abort) {
                        LOG("Aboring exposure\n");
                        guard_type g(mutex_);
                        client_.put("/abortexposure",{});
                    }
                }
            }
            catch(std::exception const &e)
            {
                LOGE(e);
                handle_error(e.what());
            }
        }
        
        virtual void start_stream(CamStreamFormat format,frame_callback_type callback,CamErrorCode &e) 
        {
            if(stream_active_ != 0) {
                stop_stream(e);
                if(e)
                    return;
            }

            try {
                guard_type g(mutex_);
                std::string bin = std::to_string(format.bin);

                client_.put("/binx",{{"BinX",bin}});
                client_.put("/biny",{{"BinY",bin}});
                client_.put("/numx",{{"NumX",std::to_string(format.width)}});
                client_.put("/numy",{{"NumY",std::to_string(format.height)}});

                callback_ = callback;
                
                stop_ = 0;
                stream_active_ = 1;
                thread_ = std::thread([=]() {
                    thread_loop(format);
                });
            }
            catch(std::exception const &ex) {
                LOGE(ex);
                e=ex;
            }
            
        }
        /// stop the stream - once function ends callback will not be called any more
        virtual void stop_stream(CamErrorCode &)
        {
            {
                guard_type g(mutex_);
                if(stream_active_ == 0)
                    return;
                callback_ = nullptr;
                stop_ = 1;
            }
            LOG("Stoping stream\n");
            thread_.join();
            stream_active_=0;
            LOG("Stopped\n");
        }

        /// list of camera controls that the camera supports
        virtual std::vector<CamOptionId> supported_options(CamErrorCode &e)
        {
            guard_type g(mutex_);
            if(!params_vec_.empty())
                return params_vec_;
            std::vector<CamOptionId> opts;
            try {
                opts.push_back(opt_exp);
                if(client_.try_get_value("/gains",iso_values_)) {
                    opts.push_back(opt_iso);
                }
                int gmin = 0;
                if(client_.try_get_value("/gainmin",gmin)) {
                    opts.push_back(opt_gain);
                }
                double v = 0;
                if(client_.try_get_value("/ccdtemperature",v)) {
                    opts.push_back(opt_temperature);
                }
                bool flag=false;
                if(client_.try_get_value("/cansetccdtemperature",flag) && flag) {
                    opts.push_back(opt_cooler_target);
                }
                flag = false;
                if(client_.try_get_value("/cooleron",flag)) {
                    opts.push_back(opt_cooler_on);
                }
                v = 0;
                if(client_.try_get_value("/coolerpower",v)) {
                    opts.push_back(opt_cooler_power_perc);
                }
            }
            catch(std::exception const &ex)
            {
                LOGE(ex);
                e=ex;
                opts.clear();
            }
            params_vec_ = opts;
            return params_vec_;
        }
        /// get camera control
        virtual CamParam get_parameter(CamOptionId id,bool current_only,CamErrorCode &e) 
        {
            guard_type g(mutex_);
            CamParam r;
            if(std::find(params_vec_.begin(),params_vec_.end(),id) == params_vec_.end()) {
                e = "Unsupported parametrer " + cam_option_id_to_name(id);
                return r;
            }
            try {
                r.option = id;
                switch(id) {
                case opt_exp:
                    {
                        constexpr double scale = 1e3;
                        r.type = type_msec;
                        r.cur_val = cur_exp_ * scale;
                        if(!current_only) {
                            r.def_val = r.cur_val;
                            r.min_val = client_.get_value<double>("/exposuremin") * scale;
                            r.max_val = client_.get_value<double>("/exposuremax") * scale;
                            r.step_size = client_.get_value<double>("/exposureresolution") * scale;
                        }
                    }
                    break;
                case opt_iso:
                    {
                        r.type = type_selection;
                        r.cur_val = client_.get_value<int>("/gain");
                        if(!current_only) {
                            r.def_val = r.cur_val;
                            r.min_val = 0;
                            r.max_val = iso_values_.size();
                            r.step_size = 1;
                            r.names = iso_values_;
                        }
                    }
                    break;
                case opt_gain:
                    {
                        r.type = type_number;
                        r.cur_val = client_.get_value<int>("/gain");
                        if(!current_only) {
                            r.def_val = r.cur_val;
                            r.min_val = client_.get_value<int>("/gainmin");
                            r.max_val = client_.get_value<int>("/gainmax");
                            r.step_size = 1;
                        }
                    }
                    break;
                case opt_temperature:
                    {
                        r.type = type_celsius;
                        r.cur_val = client_.get_value<double>("/ccdtemperature");
                        r.read_only = true;
                        r.def_val = r.cur_val;
                        r.min_val = -100;
                        r.max_val = 100;
                        r.step_size = 0.1;
                    }
                    break;
                case opt_cooler_target:
                    {
                        r.type = type_celsius;
                        r.cur_val = client_.get_value<double>("/setccdtemperature");
                        r.def_val = r.cur_val;
                        r.min_val = -100;
                        r.max_val = 100;
                        r.step_size = 0.1;
                    }
                    break;
                case opt_cooler_on:
                    {
                        r.type = type_bool;
                        r.cur_val = client_.get_value<bool>("/cooleron");
                        r.def_val = r.cur_val;
                        r.min_val = 0;
                        r.max_val = 1;
                        r.step_size = 1;
                    }
                    break;
                case opt_cooler_power_perc:
                    {
                        r.type = type_percent;
                        r.cur_val = client_.get_value<double>("/coolerpower");
                        r.def_val = r.cur_val;
                        r.min_val = 0;
                        r.max_val = 100;
                        r.read_only = true;
                        r.step_size = 1;
                    }
                    break;
                default:
                    throw std::runtime_error("Not supported");
                }
            }
            catch(std::exception const &ex)
            {
                LOGE(ex);
                e = ex;
            }
            return r;
        }
        /// set camera control
        void set_parameter_actual(CamOptionId id,double value)
        {
            guard_type g(mutex_);
            if(std::find(params_vec_.begin(),params_vec_.end(),id) == params_vec_.end()) {
                throw std::runtime_error("Unlisted parametrer " + cam_option_id_to_name(id));
            }
            switch(id) {
            case opt_iso:
            case opt_gain:
                client_.put("/gain",{{"Gain",std::to_string(int(value))}});
                break;
            case opt_temperature:
                throw std::runtime_error("Temperature is read only");
                break;
            case opt_cooler_target:
                client_.put("/setccdtemperature",{{"SetCCDTemperature",std::to_string(value)}});
                break;
            case opt_cooler_on:
                client_.put("/cooleron",{{"CoolerOn",std::string(value ? "true":"false")}});
                break;
            case opt_cooler_power_perc:
                throw std::runtime_error("Cooler power is read only");
            default:
                throw std::runtime_error("Unsupported option: " + cam_option_id_to_name(id));
            }
            
        }
        virtual void set_parameter(CamOptionId id,double value,CamErrorCode &e)
        {
            try {
                switch(id) {
                case opt_exp:
                    {
                        guard_type g(mutex_);
                        cur_exp_ = value * 1e-3;
                        cur_exp_ = std::max(exp_min_,std::min(exp_max_,cur_exp_));
                    }
                    break;
                default:
                    set_parameter_actual(id,value);
                }
            }
            catch(std::exception const &ex) 
            {
                LOGE(ex);
                e=ex;
            }
        }
    private:
        AlpacaClient client_;
        std::string name_;
        std::atomic<int> stream_active_;
        std::atomic<int> stop_;
        double cur_exp_ = 0.1;
        double exp_min_ = 0.01;
        double exp_max_ = 1;
        bool can_abort_ = false;
        CamBayerType bayer_ = bayer_na;

        int frame_counter_ = 0;
        // protected by mutex
        std::recursive_mutex mutex_;
        frame_callback_type callback_; 
        std::thread thread_;
        std::vector<CamStreamFormat> formats_;
        std::vector<CamOptionId> params_vec_;
        std::vector<std::string> iso_values_;
    };
 


    class AlpacaCameraDriver : public CameraDriver {
    public:
        static std::string connection_string;
        AlpacaCameraDriver() : 
            client_(AlpacaCameraDriver::connection_string,"camera")
        {
            if(error_stream)
                client_.set_logf(error_stream);
        }
        virtual std::vector<std::string> list_cameras(CamErrorCode &e) 
        {
            try {
                names_.clear();
                ids_.clear();
                auto devices = client_.list_devices();
                for(auto d :devices) {
                    names_.push_back(d.first);
                    ids_.push_back(d.second);
                }
            }
            catch(std::exception const &ex) {
                LOGE(ex);
                e = ex;
            }
            return names_;
        }
        virtual std::unique_ptr<Camera> open_camera(int id,CamErrorCode &e) 
        {
            try {
                if(size_t(id) >= names_.size()) {
                    throw std::runtime_error("No such camera " + std::to_string(id));
                }
                std::unique_ptr<Camera> cam(new AlpacaCamera(AlpacaCameraDriver::connection_string,ids_[id],names_[id]));
                return cam;
            }
            catch(std::exception const &ex) {
                LOGE(ex);
                e=ex.what();
                return std::unique_ptr<Camera>();
            }
        }
    private:
        std::vector<int> ids_;
        std::vector<std::string> names_;
        AlpacaClient client_;
    };
    std::string AlpacaCameraDriver::connection_string;
}

extern "C" {
    int ols_set_alpaca_driver_config(char const *s)
    {
        ols::AlpacaCameraDriver::connection_string = s;
        return 0;
    }
    void ols_set_alpaca_driver_log(char const *log_path,int debug)
    {
        if(debug) {
            ols::error_stream = fopen(log_path,"w");
        }
    }
    ols::CameraDriver *ols_get_alpaca_driver(int /*unused*/,ols::CamErrorCode *e)
    {
        try {
            return new ols::AlpacaCameraDriver();
        }
        catch(std::exception const &ex) {
            LOGE(ex);
            if(e)  {
                *e = ex;
            }
            return nullptr;
        }
    }
}


