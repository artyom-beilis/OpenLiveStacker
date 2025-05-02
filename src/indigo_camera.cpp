#include "camera.h"
#include <mutex>
#include <sstream>
#include <iostream>
#include <atomic>
#include <map>
#include <set>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <thread>
#include <algorithm>
#include <fstream>
#include <cmath>
#include "indigo_camera.h"
#include "sync_queue.h"
#include "fitsmat.h"
#include "fits_guard.h"
#include "shift_bit.h"
#define INDIGO_LINUX
#include <indigo/indigo_bus.h>
#include <indigo/indigo_client.h>

namespace ols {

 

    static FILE *error_stream = NULL;
    #define LOG(...) do { if( ::ols::error_stream) { \
        struct timeval tv; gettimeofday(&tv,nullptr); \
        int now = tv.tv_sec  % (24*3600); \
        fprintf( ::ols::error_stream,"%02d:%02d:%02d.%03d  ",now/3600,(now/60)%60,now%60,int(tv.tv_usec / 1000)); \
        fprintf( ::ols::error_stream,__VA_ARGS__); fflush(::ols::error_stream); \
    }} while(0)


    static void log_property(indigo_property *property,char const *message) 
    {
        if(!error_stream)
            return;
        char const *types[] = {"n/a","text","number","switch","light","blob"};
        char const *perms[] = {"n/a","ro","rw","wo"};
        char const *states[] = {"idle","ok","busy","alert"};

        if(message == nullptr)
            message = "";
    
        LOG("%s:%s (%s) %s type %s, items %d %s\n",property->device, property->name, states[property->state], perms[property->perm], types[property->type],property->count,message);
        for(int i=0;i<property->count;i++) {
            indigo_item *item = property->items+i;
            switch(property->type) {
            case INDIGO_TEXT_VECTOR:
                fprintf(error_stream,"  %2d: %s/%s: %s\n",i,item->name,item->label,(item->text.length > INDIGO_VALUE_SIZE ? item->text.long_value : item->text.value ));
                break;
            case INDIGO_NUMBER_VECTOR:
                fprintf(error_stream,"  %2d: %s/%s: val=%f min=%f max=%f step=%f target=%f (%s)\n",i,item->name,item->label,item->number.value,item->number.min,item->number.max,item->number.step,item->number.target,item->number.format);
                break;
            case INDIGO_SWITCH_VECTOR:
                fprintf(error_stream,"  %2d: %s/%s: %s\n",i,item->name,item->label,item->sw.value ? "ON " : "OFF");
                break;
            case INDIGO_LIGHT_VECTOR:
                fprintf(error_stream,"  %2d: %s/%s: %s\n",i,item->name,item->label,states[item->light.value]);
                break;
            case INDIGO_BLOB_VECTOR:
                fprintf(error_stream,"  %2d: %s/%s: format=%s size=%ld url=%s ptr=%p\n",i,item->name,item->label,item->blob.format,item->blob.size,item->blob.url,item->blob.value);
                break;
            }
        }
    }

    typedef std::unique_lock<std::recursive_mutex> guard_type;
    class IndigoCamera : public Camera 
    {
    public:

        IndigoCamera(std::string const &name,bool local_driver) : name_(name), local_driver_(local_driver)
        {
            LOG("Camera %s created\n",name.c_str());
            config_client();
            indigo_attach_client(&client_);
            LOG("Client %s attached\n",name.c_str());
            if(local_driver_) {
                auto worker = std::thread([this]() {
                   this->thread_func();
                });
                async_calls_ = std::move(worker);
            }
        }
        void async_exec(void (IndigoCamera::*f)())
        {
            if(!local_driver_) {
                (this->*f)();
            }
            else {
                async_queue_.push([=]() {
                    (this->*f)(); 
                });
            }
        }
        ~IndigoCamera()
        {
            if(local_driver_) {
                async_queue_.push(std::function<void()>());
                async_calls_.join();
            }
            indigo_detach_client(&client_);
        }

        void thread_func()
        {
            while(true) {
                auto f = async_queue_.pop();
                if(!f)
                    break;
                {
                    guard_type g(lock_);
                    f(); 
                }
            }
        }

        bool wait_ready(CamErrorCode &e)
        {
            for(int i=0;i<100;i++) {
                {
                    guard_type g(lock_);
                    if(connected_)
                        return true;
                }
                indigo_usleep(100000);
            }
            {
                guard_type g(lock_);
                if(connected_)
                    return true;
            }
            e = "Failed to connect to camera - timeout";
            return false;
        }
        /// Camera name
        virtual std::string name(CamErrorCode &)
        {
            return name_;
        }
        /// Return list of suppored video formats
        virtual std::vector<CamStreamFormat> formats(CamErrorCode &e)
        {
            if(!wait_ready(e))
                return std::vector<CamStreamFormat>();
            guard_type g(lock_);
            return formats_;
        }

        /// Start a video stream with provided callback 
        virtual void start_stream(CamStreamFormat format,frame_callback_type callback,CamErrorCode &e) 
        {
            guard_type g(lock_);
            std::string name;
            int bits=8;
            for(unsigned int i=0;i<formats_.size();i++) {
                if(formats_[i].format == format.format 
                   && formats_[i].width == format.width 
                   && formats_[i].height == format.height
                   && formats_[i].bin == format.bin)
                {
                    name = format_names_[i];
                    bits = scale_factors_[i];
                    break;
                }
            }
            stream_ =format;
            if(name.empty()) {
                e="Invalid format" + name;
            }
            indigo_change_switch_property_1(&client_,name_.c_str(), CCD_MODE_PROPERTY_NAME, name.c_str(),true);
            callback_ = callback;
            current_bits_ = bits;
            if(!streaming_) {
                expose();
            }
            streaming_ = true;
        }

        void handle_frame(void *ptr,size_t len)
        {
            if(!callback_)
                return;
            CamFrame frm;
            struct timeval tv;
            gettimeofday(&tv,nullptr);
            frm.unix_timestamp = tv.tv_sec;
            frm.unix_timestamp += tv.tv_usec * 1e-6;
            TempFileGuard g(ptr,len);
            if(!g) {
                LOG("Failed to save fits file %s\n",g.error_message);
                return;
            }
            try {
                auto res = load_fits(g.fname);
                auto img = res.first; 
                frm.bayer  = res.second;
                frm.data = img.data;
                frm.data_size = img.rows * img.cols * img.elemSize();
                frm.height = img.rows;
                frm.width = img.cols;
                frm.format = stream_.format;
                frm.frame_counter = frame_counter_++;

                if(img.elemSize1() == 2 && current_bits_ > 8 && current_bits_ < 16) {
                    shift_to16(current_bits_,(unsigned short *)frm.data,frm.data_size/2);
                }
                callback_(frm);
            }
            catch(std::exception const &e) {
                LOG("Exception in callback %s\n",e.what());
            }
            catch(...) {
                LOG("Unknown exception in callback\n");
            }
        }

        /// stop the stream - once function ends callback will not be called any more
        virtual void stop_stream(CamErrorCode &) 
        {
            guard_type g(lock_);
            if(!streaming_)
                return;
            streaming_ = false;
            callback_ = nullptr;
            //indigo_change_switch_property_1(&client_, name_.c_str(), CCD_ABORT_EXPOSURE_PROPERTY_NAME, CCD_ABORT_EXPOSURE_ITEM_NAME, true);
        }

        /// list of camera controls that the camera supports
        virtual std::vector<CamOptionId> supported_options(CamErrorCode &e)
        {
            std::vector<CamOptionId> opts;
            if(!wait_ready(e)) {
                return opts;
            }
            for(auto const &v:parameters_) {
                opts.push_back(v.first);
            }
            return opts;
        }
        virtual CamParam get_parameter(CamOptionId id,bool /*current_only*/,CamErrorCode &e)
        {
            guard_type g(lock_);
            auto p = parameters_.find(id);
            if(p==parameters_.end()) {
                e = "Unsupported parameters";
                return CamParam();
            }
            return p->second;
        }
        void defer(CamOptionId id,double value)
        {
            deferred_.insert(id);
            LOG("Deferring option %s:%f\n",cam_option_id_to_string_id(id).c_str(),value);
        }
        void set_indigo_parameter(CamOptionId id)
        {
            switch(id) {
            case opt_gain: 
                {
                    char const *item = CCD_GAIN_ITEM_NAME;
                    double v = parameters_[id].cur_val;
                    LOG("Setting gain to %f\n",v);
                    indigo_change_number_property(&client_, name_.c_str(), CCD_GAIN_PROPERTY_NAME, 1, &item, &v);
                }
                break;
            default:
                break;
            }
        }
        /// set camera control
        virtual void set_parameter(CamOptionId id,double value,CamErrorCode &e)
        {
            guard_type g(lock_);
            if(parameters_.find(id) == parameters_.end()) {
                e = "No such option";
                return;
            }
            parameters_[id].cur_val = value;
            switch(id) {
            case opt_exp:
                break;
            case opt_gain:
                if(streaming_)
                    defer(id,value);
                else
                    set_indigo_parameter(id);
                break;
            default:
                e = "Not implemented";
            }
        }


        
    private:
        indigo_result on_client_attach(indigo_client *client)
        {
            LOG("Camera Client attached to %s\n",name_.c_str());
            indigo_property filter = indigo_property();
            strncpy(filter.device,name_.c_str(),INDIGO_NAME_SIZE - 1);
            indigo_enumerate_properties(client, &filter);
            return INDIGO_OK;
        }
        
        static indigo_result client_attach(indigo_client *client) 
        {
            IndigoCamera *self = static_cast<IndigoCamera*>(client->client_context);
            return self->on_client_attach(client);
        }
        static indigo_result client_update_property(indigo_client *client, indigo_device *device, indigo_property *property, const char *message)
        {
            IndigoCamera *self = static_cast<IndigoCamera*>(client->client_context);
            return self->client_new_property_impl(false,client,device,property,message);
        }
        static indigo_result client_new_property(indigo_client *client, indigo_device *device, indigo_property *property, const char *message)
        {
            IndigoCamera *self = static_cast<IndigoCamera*>(client->client_context);
            return self->client_new_property_impl(true,client,device,property,message);
        }

        void set_fits()
        {
            indigo_change_switch_property_1(&client_,name_.c_str(), CCD_IMAGE_FORMAT_PROPERTY_NAME, CCD_IMAGE_FORMAT_FITS_ITEM_NAME,true);
        }
        
        indigo_result client_new_property_impl(bool is_new,indigo_client *client, indigo_device *device, indigo_property *property, const char *msg)
        {
            guard_type g(lock_);
            log_property(property,msg);
            if (!strcmp(property->name, CONNECTION_PROPERTY_NAME)) {
                if (indigo_get_switch(property, CONNECTION_CONNECTED_ITEM_NAME)) { 
                    if(property->state == INDIGO_BUSY_STATE) {
                        busy_detected_ = true;
                        LOG("Connected IDLE to %s\n",name_.c_str());
                    }
                    else if(property->state == INDIGO_OK_STATE) {
                        if(busy_detected_) {
                            connected_ = true;
                            LOG("Connected OK to %s\n",name_.c_str());
                        }
                        else {
                            LOG("Reconnecting to camera %s\n",name_.c_str());
                            indigo_device_disconnect(client, &name_[0]);
                        }
                    }
                } 
                else {
                    LOG("Connecting to %s\n",name_.c_str());
                    indigo_device_connect(client, &name_[0]);
                }
                return INDIGO_OK;
            }
            
            if( config_num_opt(property,opt_gain,type_number,CCD_GAIN_PROPERTY_NAME,CCD_GAIN_ITEM_NAME)
                || config_num_opt(property,opt_temperature,type_celsius,CCD_TEMPERATURE_PROPERTY_NAME,CCD_TEMPERATURE_ITEM_NAME,1.0,true))
            {
                return INDIGO_OK;
            }
            if(is_new && !strcmp(property->name,CCD_STREAMING_PROPERTY_NAME)) {
                CamParam r = CamParam();
                if(config_num_opt(r,property,opt_exp,type_msec,CCD_STREAMING_PROPERTY_NAME,CCD_EXPOSURE_ITEM_NAME,1e3)) {
                    streaming_max_exp_ = r.max_val;
                }
            }
            if(!strcmp(property->name, CCD_EXPOSURE_PROPERTY_NAME)) {
                if(!connected_) {
                    config_num_opt(property,opt_exp,type_msec,CCD_EXPOSURE_PROPERTY_NAME,CCD_EXPOSURE_ITEM_NAME,1e3);
                }
                if(property->state == INDIGO_OK_STATE && streaming_) {
                    async_exec(&IndigoCamera::expose);
                }
                return INDIGO_OK;
            }
            if(is_new && !strcmp(property->name, CCD_IMAGE_FORMAT_PROPERTY_NAME)) {
                async_exec(&IndigoCamera::set_fits);
                return INDIGO_OK;
            }

            if(!strcmp(property->name, CCD_MODE_PROPERTY_NAME)) {
                if(formats_.empty()) {
                    for(int i=0;i<property->count;i++) {
                        add_format(property->items + i);
                    }
                }
                return INDIGO_OK;
            }
            
            if(is_new && !strcmp(property->name, CCD_IMAGE_PROPERTY_NAME)) {
                if (device->version >= INDIGO_VERSION_2_0)
                    indigo_enable_blob(client, property, INDIGO_ENABLE_BLOB_URL);
                else
                    indigo_enable_blob(client, property, INDIGO_ENABLE_BLOB_ALSO);
            }

            if (!strcmp(property->name, CCD_IMAGE_PROPERTY_NAME) && property->state == INDIGO_OK_STATE) {
                if (*property->items[0].blob.url && indigo_populate_http_blob_item(&property->items[0]))
                    LOG("image URL received (%s, %d bytes)...", property->items[0].blob.url, int(property->items[0].blob.size));
                if (property->items[0].blob.value) {
                    handle_frame(property->items[0].blob.value, property->items[0].blob.size);
                    if (*property->items[0].blob.url) {
                        free(property->items[0].blob.value);
                        property->items[0].blob.value = NULL;
                    }
                }
            }

            return INDIGO_OK;
        }
        void expose() 
        {
            CamErrorCode e;
            for(auto id:deferred_) {
                set_indigo_parameter(id);
            }
            deferred_.clear();
            char const *item = CCD_EXPOSURE_ITEM_NAME;
            double value = parameters_[opt_exp].cur_val * 1e-3;
            LOG("Setting exposure to %f\n",value);
            indigo_change_number_property(&client_, name_.c_str(), CCD_EXPOSURE_PROPERTY_NAME, 1, &item, &value);
        }

        void add_format(indigo_item *item)
        {
            CamStreamFormat fmt = CamStreamFormat();
            int bits=-1;
            if(!parse_indigo_format(item->name,item->label,fmt,bits))
                return;
            formats_.push_back(fmt);
            format_names_.push_back(item->name);
            scale_factors_.push_back(bits);
        }

        
        bool config_num_opt(CamParam &r,indigo_property *property,CamOptionId id,CamOptionType type,char const *name1,char const *name2,double scale = 1.0,bool read_only=false)
        {
            r = CamParam();
            indigo_item *item = nullptr;
            // poor api no really need const cast
            if(!strcmp(property->name,name1) && (item = indigo_get_item(property,const_cast<char*>(name2)))!= nullptr)  {
                r.option = id;
                r.read_only = read_only;
                r.type = type;
                r.min_val = item->number.min * scale;
                r.max_val = item->number.max * scale;
                r.cur_val = item->number.value * scale;
                r.def_val = r.cur_val;
                r.step_size = item->number.step * scale;
                return true;
            }
            return false;
        }

        bool config_num_opt(indigo_property *property,CamOptionId id,CamOptionType type,char const *name1,char const *name2,double scale = 1.0,bool read_only=false)
        {
            CamParam r = CamParam();
            if(config_num_opt(r,property,id,type,name1,name2,scale,read_only)) {
                auto p = parameters_.find(r.option);
                if(p == parameters_.end())
                    parameters_[r.option] = r;
                else
                    parameters_[r.option].cur_val = r.cur_val;
                return true;
            }
            return false;
        }


        void config_client() {
            client_ = indigo_client
            {
                "OpenLiveStacker Cam", // name
                false, // remote
                this, // ctx
                INDIGO_OK, // last result
                INDIGO_VERSION_CURRENT, //version
                NULL, // enable blob model
                &IndigoCamera::client_attach,
                &IndigoCamera::client_new_property,
                &IndigoCamera::client_update_property,
                NULL,
                NULL,
                nullptr, // detach IndigoCameraDriver::client_detach
            };
        }

        CamStreamFormat stream_;
        int frame_counter_ = 0;
        std::string name_; 
        bool local_driver_;
        bool is_indi_;
        indigo_client client_;
        bool streaming_ = false;
        frame_callback_type callback_;
        std::map<CamOptionId,CamParam> parameters_;
        std::set<CamOptionId> deferred_;
        double streaming_max_exp_ = -1;
        std::vector<CamStreamFormat> formats_;
        std::vector<std::string> format_names_;
        std::vector<int> scale_factors_;
        int current_bits_;
        std::recursive_mutex lock_;
        bool busy_detected_ = false;
        bool connected_ = false;
        indigo_server_entry *server_ = nullptr;

        sync_queue<std::function<void()>> async_queue_;
        std::thread async_calls_;
    };



    class IndigoCameraDriver : public CameraDriver
    {
    public:

        static std::string connection_string;

        IndigoCameraDriver(CamErrorCode &e)
        {
            indigo_start();
            config_client();
            indigo_attach_client(&client_);
            std::string host,driver;
            int port = -1;
            size_t pos = 0;
            if(connection_string.empty()) {
                host = "localhost";
                port = 7624;
            }
            else if((pos = connection_string.find(':')) != std::string::npos) {
                host = connection_string.substr(0,pos);
                port = atoi(connection_string.c_str() + pos + 1);
            }
            else {
                driver = connection_string;
            }
            LOG("Connection String=[%s] driver=[%s], host=[%s], port=%d\n",connection_string.c_str(),driver.c_str(),host.c_str(),port);
            if(port != -1) {
                if(indigo_connect_server(NULL, host.c_str(),port, &server_) != INDIGO_OK) {
                    e = "Failed to connect to server " + host + ":" + std::to_string(port);
                }
            }
            else {
                indigo_service_ = true;
                if(indigo_load_driver(driver.c_str(), true, &driver_) != INDIGO_OK) {
                    e="Failed to load driver: " + driver;
                }
            }
        }

        ~IndigoCameraDriver()
        {
            if(server_)
                indigo_disconnect_server(server_);
            if(driver_)
                indigo_remove_driver(driver_);
            indigo_detach_client(&client_);
            indigo_stop();
        }

        virtual std::vector<std::string> list_cameras(CamErrorCode &)
        {
            for(int i=0;i<50;i++) {
                {
                    guard_type g(lock_);
                    if(!device_names_.empty())
                        return device_names_;
                }
                indigo_usleep(100000);
            }
            guard_type g(lock_);
            return device_names_;
        }

        virtual std::unique_ptr<Camera> open_camera(int id, CamErrorCode &e)
        {

            guard_type g(lock_);
            if (size_t(id) >= device_names_.size())
            {
                e = "No such camera " + std::to_string(id);
                return std::unique_ptr<Camera>();
            }
            bool local_driver = driver_ != nullptr;
            std::unique_ptr<Camera> c(new IndigoCamera(device_names_[size_t(id)],local_driver));
            return c;
        }
    private:

        static indigo_result client_new_property(indigo_client *client, indigo_device *device, indigo_property *property, const char *message)
        {
            log_property(property,message);
            return client_update_property(client,device,property,message);
        }
        static indigo_result client_update_property(indigo_client *client, indigo_device *, indigo_property *property, const char *message)
        {
            IndigoCameraDriver *self = static_cast<IndigoCameraDriver*>(client->client_context);
            if(strcmp(property->name,"INFO") == 0 && property->type == INDIGO_TEXT_VECTOR) {
                log_property(property,message);
                self->check_info_property(property);
            }
            return INDIGO_OK;
        }
        void check_info_property(indigo_property *property)
        {
            guard_type g(lock_);
            for(int i=0;i<property->count;i++) {
                if(strcmp(property->items[i].name,"VERSION") == 0 && strcmp(property->items[i].label,"INDIGO") == 0) {
                    set_indigo();
                }
                if(strcmp(property->items[i].name,"DEVICE_INTERFACE")==0 && (atoi(property->items[i].text.value) & INDIGO_INTERFACE_CCD)) {
                    add_device(property->device);
                }
            }
        }
        void set_indigo()
        {
            guard_type g(lock_);
            indigo_service_ = true;
        }
        
        static indigo_result client_attach(indigo_client *client) 
        {
            indigo_enumerate_properties(client, &INDIGO_ALL_PROPERTIES);
            return INDIGO_OK;
        }

        void add_device(char const *name)
        {
            for(size_t i=0;i<device_names_.size();i++) {
                if(device_names_[i] == name)
                    return;
            }
            device_names_.push_back(name);
        }

        void config_client() {
            client_ = indigo_client
            {
                "OpenLiveStacker", // name
                false, // remote
                this, // ctx
                INDIGO_OK, // last result
                INDIGO_VERSION_CURRENT, //version
                NULL, // enable blob model
                &IndigoCameraDriver::client_attach,
                &IndigoCameraDriver::client_new_property,
                &IndigoCameraDriver::client_update_property,
                NULL,
                NULL,
                nullptr, // detach IndigoCameraDriver::client_detach
            };
        }

        indigo_client client_;
        std::vector<std::string> device_names_;
        indigo_server_entry *server_ = nullptr;
        indigo_driver_entry *driver_ = nullptr;
        bool connected_ = false;
        bool indigo_service_ = false;
        std::recursive_mutex lock_;
    };

    std::string IndigoCameraDriver::connection_string;


}
extern "C" {
    int ols_set_indigo_driver_config(char const *s)
    {
        ols::IndigoCameraDriver::connection_string = s;
        return 0;
    }
    void ols_set_indigo_driver_log(char const *log_path,int debug)
    {
        if(debug) {
            ols::error_stream = fopen(log_path,"w");
        }
    }
    ols::CameraDriver *ols_get_indigo_driver(int /*unused*/,ols::CamErrorCode *e)
    {
        std::unique_ptr<ols::IndigoCameraDriver> driver(new ols::IndigoCameraDriver(*e));
        if(*e) {
            driver.reset();
        }
        return driver.release();
    }
}
