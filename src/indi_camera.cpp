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

#ifdef INDI_AS_LIBRARY
#include "baseclient.h"
#include "basedevice.h"
#else
#include <libindi/baseclient.h>
#include <libindi/basedevice.h>
#endif
namespace ols {

 

    static FILE *error_stream = NULL;
    #define LOG(...) do { if( ::ols::error_stream) { \
        struct timeval tv; gettimeofday(&tv,nullptr); \
        int now = tv.tv_sec  % (24*3600); \
        fprintf( ::ols::error_stream,"%02d:%02d:%02d.%03d  ",now/3600,(now/60)%60,now%60,int(tv.tv_usec / 1000)); \
        fprintf( ::ols::error_stream,__VA_ARGS__); fflush(::ols::error_stream); \
    }} while(0)


    static void log_property(char const *src,INDI::Property const &p) 
    {
        if(!error_stream)
            return;
        char const *perms[]={"ro","wo","rw"};
        LOG("%3s %s:%s (%s) %s type %s\n",src,p.getDeviceName(), p.getName(), p.getStateAsString(), perms[p.getPermission()], p.getTypeAsString());
        switch(p.getType()) {
        case INDI_NUMBER:
            {
                INDI::PropertyNumber v(p);
                for(size_t i=0;i<v.size();i++) {
                    fprintf(error_stream,"  %2d %s/%s val=%f min=%f max=%f step=%f\n",int(i),v[i].getName(),v[i].getLabel(),v[i].getValue(),v[i].getMin(),v[i].getMax(),v[i].getStep());
                }
            }
            break;
        case INDI_TEXT:
            {
                INDI::PropertyText v(p);
                for(size_t i=0;i<v.size();i++) {
                    fprintf(error_stream,"  %2d %s/%s val=%s\n",int(i),v[i].getName(),v[i].getLabel(),v[i].getText());
                }
            }
            break;
        case INDI_SWITCH:
            {
                INDI::PropertySwitch v(p);
                for(size_t i=0;i<v.size();i++) {
                    fprintf(error_stream,"  %2d %s/%s %s\n",int(i),v[i].getName(),v[i].getLabel(),(v[i].getState() == ISS_ON ? "ON " : "OFF"));
                }
            }
            break;
        case INDI_LIGHT:
            {
                INDI::PropertyLight v(p);
                for(size_t i=0;i<v.size();i++) {
                    fprintf(error_stream,"  %2d %s/%s %s\n",int(i),v[i].getName(),v[i].getLabel(),v[i].getStateAsString());
                }
            }
            break;
        case INDI_BLOB:
            {
                INDI::PropertyBlob v(p);
                for(size_t i=0;i<v.size();i++) {
                    fprintf(error_stream,"  %2d %s/%s ptr=%p size=%d\n",int(i),v[i].getName(),v[i].getLabel(),v[i].getBlob(),v[i].getSize());
                }
            }
            break;
        default:
            ;
        }
    }

    typedef std::unique_lock<std::recursive_mutex> guard_type;
    
    class IndiCamera : public Camera , public INDI::BaseClient
    {
    public:

        IndiCamera(std::string const &name,std::string const &host,int port): 
            name_(name)
        {
            LOG("Camera %s created\n",name.c_str());
            setServer(host.c_str(),port);
            connectServer();
            setBLOBMode(B_ALSO, name_.c_str(), nullptr);
            enableDirectBlobAccess(name_.c_str(), nullptr);
            watchDevice(name_.c_str(),[=](INDI::BaseDevice dp) {
                device_ = dp;
            });
        }
        ~IndiCamera()
        {
        }


        bool wait_ready(CamErrorCode &e)
        {
            for(int i=0;i<50;i++) {
                {
                    guard_type g(lock_);
                    if(connected_ && controls_detected_ && frame_width_ > 0 && bin_ >= 1)
                        return true;
                }
                usleep(100000);
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
        virtual std::string name(CamErrorCode &) override
        {
            return name_;
        }
        /// Return list of suppored video formats
        virtual std::vector<CamStreamFormat> formats(CamErrorCode &e) override
        {
            if(!wait_ready(e))
                return std::vector<CamStreamFormat>();
            guard_type g(lock_);
            if(formats_.empty()) {
                CamStreamType format;
                if(is_rgb_) {
                    if(current_bits_ == 8)
                        format = stream_rgb24;
                    else
                        format = stream_rgb48;
                }
                else if(is_color_) {
                    if(current_bits_ == 8)
                        format = stream_raw8;
                    else
                        format = stream_raw16;
                }
                else {
                    if(current_bits_ == 8)
                        format = stream_mono8;
                    else
                        format = stream_mono16;
                }
                for(int bin=1;bin<=bin_;bin *= 2) {
                    if(frame_width_ % (bin*2) != 0 || frame_height_ % (bin*2) != 0) {
                        LOG("width/height = %d/%d incompatible with bin=%d\n",frame_width_,frame_height_,bin);
                        continue;
                    }
                    CamStreamFormat fmt;
                    fmt.format = format;
                    fmt.bin = bin;
                    fmt.width = frame_width_ / bin;
                    fmt.height = frame_height_ / bin;
                    formats_.push_back(fmt);
                }
            }
            return formats_;
        }

        /// Start a video stream with provided callback 
        virtual void start_stream(CamStreamFormat format,frame_callback_type callback,CamErrorCode &e) override 
        {
            guard_type g(lock_);
            unsigned int i=0;
            for(i=0;i<formats_.size();i++) {
                if(formats_[i].format == format.format 
                   && formats_[i].width == format.width 
                   && formats_[i].height == format.height
                   && formats_[i].bin == format.bin)
                {
                    break;
                }
            }
            if( i == formats_.size()) {
                e = "Invalid format";
                return;
            }
            stream_ =format;
            callback_ = callback;
            if(!streaming_) {
                expose(e);
            }
            streaming_ = true;
        }

        void expose(CamErrorCode &e)
        {
            if(!deferred_.empty()) {
                for(auto id : deferred_) {
                    set_indi_parameter(id);
                }
            }
            deferred_.clear();

            INDI::PropertyNumber ccd_exposure = device_.getProperty("CCD_EXPOSURE");
            INDI::PropertyNumber ccd_binning = device_.getProperty("CCD_BINNING");
            
            if (!ccd_exposure.isValid())
            {
                e="Can't find CCD_EXPOSURE";
                LOG("Error: unable to find CCD_EXPOSURE property...\n");
                return;
            }

            bool update_binning = false;
            double exp = parameters_[opt_exp].cur_val;
            if(ccd_binning.isValid() && ccd_binning.size() == 2) {
                if(ccd_binning[0].getValue() != stream_.bin || ccd_binning[1].getValue() != stream_.bin) {
                    ccd_binning[0].setValue(stream_.bin);
                    ccd_binning[1].setValue(stream_.bin);
                    update_binning = true;
                }
            }

            if(update_binning) {
                LOG("Setting new binning %d\n",stream_.bin);
                sendNewProperty(ccd_binning);
            }
            LOG("Starting new exposure of %f ms\n",exp);
            ccd_exposure[0].setValue(exp * 1e-3);
            sendNewProperty(ccd_exposure);
        }


        void handle_frame(INDI::PropertyBlob prop)
        {
            if(!callback_)
                return;
            if(prop.size() < 1 || prop[0].getBlob() == nullptr || prop[0].getSize() <= 0)
                return;
            void *ptr = prop[0].getBlob();;
            int len = prop[0].getSize();
            CamFrame frm;
            struct timeval tv;
            gettimeofday(&tv,nullptr);
            frm.unix_timestamp = tv.tv_sec;
            frm.unix_timestamp += tv.tv_usec * 1e-6;
            TempFileGuard g(ptr,len);
            if(!g) {
                LOG("Faled to save file %s\n",g.error_message);
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
                
                LOG("Got fits frame %dx%d channels=%d bytes_per_ch=%d\n",img.rows,img.cols,int(img.elemSize()/img.elemSize1()),int(img.elemSize1()));

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
        virtual void stop_stream(CamErrorCode &) override 
        {
            guard_type g(lock_);
            if(!streaming_)
                return;
            streaming_ = false;
            callback_ = nullptr;
        }

        /// list of camera controls that the camera supports
        virtual std::vector<CamOptionId> supported_options(CamErrorCode &e) override
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

        void handle_iso_switch(INDI::PropertySwitch prop)
        {
            // Só adiciona uma vez
            if (parameters_.find(opt_iso) != parameters_.end())
                return;

            CamParam r;
            r.option = opt_iso;
            r.type = type_selection;
            r.names.clear();
            int cur_idx = 0;
            for (size_t i = 0; i < prop.size(); ++i)
            {
                r.names.push_back(prop[i].getLabel());
                if (prop[i].getState() == ISS_ON)
                    cur_idx = int(i);
            }
            r.min_val = 0;
            r.max_val = int(prop.size()) - 1;
            r.step_size = 1;
            r.cur_val = cur_idx;
            r.def_val = cur_idx;
            r.read_only = (prop.getPermission() == IP_RO);
            parameters_[opt_iso] = r;
        }
        

        
        virtual CamParam get_parameter(CamOptionId id,bool /*current_only*/,CamErrorCode &e) override
        {
            guard_type g(lock_);
            auto it = parameters_.find(id);
            if(it == parameters_.end()) {
                e = "Unsupported parameters";
                return CamParam();
            }
            if(id == opt_iso) {
                INDI::PropertySwitch prop = device_.getProperty("CCD_ISO");
                if(prop.isValid() && prop.size() > 0) {
                    CamParam r = it->second;
                    int cur_idx = 0;
                    for(size_t i=0; i<prop.size(); ++i) {
                        if(prop[i].getState() == ISS_ON) {
                            cur_idx = int(i);
                            break;
        }
        }
                    r.cur_val = cur_idx;
                    return r;
                }
            }
            return it->second;
        }
        

        
        void set_indi_parameter(CamOptionId id)
        {
            char const *control_name = nullptr;
            char const *switch_name = nullptr;
            char const *prop_name = ccd_controls_name_;
            double scale = 1.0;
            switch(id) {
            case opt_gain:
                control_name = "Gain";
                break;
            case opt_gamma:
                control_name = "Gamma";
                scale = 100;
                break;
            case opt_wb_r:
                control_name = "WB_R";
                break;
            case opt_wb_b:
                control_name = "WB_B";
                break;
            case opt_contrast:
                control_name = "Contrast";
                break;
            case opt_brightness:
                control_name = "Brigtness";
                break;
            case opt_temperature:
                LOG("Temp is read only\n");
                break;
            case opt_cooler_target:
                prop_name = "CCD_TEMPERATURE";
                control_name = "CCD_TEMPERATURE_VALUE";
                break;
            case opt_cooler_power_perc:
                prop_name = "CCD_COOLER_POWER";
                control_name = "CCD_COOLER_VALUE";
                break;
            case opt_cooler_on:
                {
                    switch_name = (parameters_[id].cur_val != 0) ? "COOLER_ON" : "COOLER_OFF";
                    prop_name = "CCD_COOLER";
                }
                break;
            
            case opt_iso:
                {
                    INDI::PropertySwitch prop = device_.getProperty("CCD_ISO");
                    if (!prop.isValid() || prop.size() < 1)
                    {
                        LOG("Can't find CCD_ISO property\n");
                        return;
                    }
                    int idx = int(parameters_[opt_iso].cur_val);
                    if (idx < 0 || idx >= int(prop.size()))
                    {
                        LOG("ISO idx out of range\n");
                        return;
                    }
                    // Desliga todos, liga só o pretendido
                    for (size_t i = 0; i < prop.size(); ++i)
                        prop[i].setState(i == size_t(idx) ? ISS_ON : ISS_OFF);

                    sendNewProperty(prop);
                    LOG("Set CCD_ISO to %s\n", prop[idx].getLabel());
                    return;
                }
            
            default:
                ;
            }
            if(switch_name && prop_name) {
                INDI::PropertySwitch p=device_.getProperty(prop_name);
                if(!p.isValid()) {
                    LOG("Can't find %s\n",prop_name);
                    return;
                }
                auto w=p.findWidgetByName(switch_name);
                if(!w) {
                    LOG("Can't find %s in %s\n",switch_name,prop_name);
                    return;
                }
                w->setState(ISS_ON);
                sendNewProperty(p);
            }
            if(control_name && prop_name) {
                INDI::PropertyNumber p=device_.getProperty(prop_name);
                if(!p.isValid()) {
                    LOG("Can't find %s\n",prop_name);
                    return;
                }
                auto w=p.findWidgetByName(control_name);
                if(!w) {
                    LOG("Can't find %s in %s\n",control_name,prop_name);
                    return;
                }
                double v = parameters_[id].cur_val * scale;
                LOG("Setting %s to %f\n",control_name,v);
                w->setValue(v);
                sendNewProperty(p);
            }
        }
        /// set camera control
        virtual void set_parameter(CamOptionId id,double value,CamErrorCode &e) override
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
            case opt_iso:
                if(streaming_) {
                    defer(id, value);
                } else {
                    set_indi_parameter(id);
                }
                break;
            default:
                if(streaming_) {
                    defer(id,value);
                }
                else {
                    set_indi_parameter(id);
                }
            }
        }
        

        void defer(CamOptionId id,double value)
        {
            deferred_.insert(id);
            LOG("Deferring option %s:%f\n",cam_option_id_to_string_id(id).c_str(),value);
        }

    protected:
        
        void configure_bool_switch_param(INDI::PropertySwitch prop,char const *true_name,char const *false_name,CamOptionId id)
        {
            auto p_true  = prop.findWidgetByName(true_name);
            auto p_false = prop.findWidgetByName(false_name);
            if(!p_true || !p_false)
                return;
            CamParam r = CamParam();
            r.option = id;
            r.read_only = false;
            r.type = type_bool;
            r.min_val = 0;
            r.max_val = 1;
            r.cur_val = p_true -> getState() == ISS_ON ? 1.0 : 0.0;
            r.def_val = r.cur_val;
            r.step_size = 1;
            parameters_[id] = r;
        }
        bool configure_num_param(INDI::PropertyNumber prop,char const *name,CamOptionId id,CamOptionType type=type_number,double scale=1.0,bool read_only=false)
        {
            auto p = prop.findWidgetByName(name);
            if(!p)
                return false;
            CamParam r = CamParam();
            r.option = id;
            r.read_only = read_only;
            r.type = type;
            r.min_val = p->getMin()* scale;
            r.max_val = p->getMax() * scale;
            r.cur_val = p->getValue() * scale;
            r.def_val = r.cur_val;
            r.step_size = p->getStep() * scale;
            parameters_[id] = r;
            return true;
        }

        void handle_exposure(INDI::PropertyNumber p)
        {
            if(parameters_.find(opt_exp) == parameters_.end()) {
                if(configure_num_param(p,"CCD_EXPOSURE_VALUE",opt_exp,type_msec,1e3)) {
                    // since default could be 0
                    parameters_[opt_exp].cur_val = parameters_[opt_exp].def_val = 100;
                }
            }
            if(p.getState() == IPS_OK && streaming_) {
                CamErrorCode e;
                expose(e);
            }
        }

        void handle_controls(INDI::PropertyNumber p)
        {
            configure_num_param(p,"Gain",opt_gain);
            // it does not really behave well on raw image - ignore one
            // configure_num_param(p,"Gamma",opt_gamma,type_number,1e-2);
            //
            configure_num_param(p,"WB_R",opt_wb_r);
            configure_num_param(p,"WB_B",opt_wb_b);
            configure_num_param(p,"White Balance Temperature",opt_wb,type_kelvin);
            configure_num_param(p,"Contrast",opt_contrast);
            configure_num_param(p,"Brigtness",opt_brightness);
            controls_detected_ = true;
        }

        void handle_connection(INDI::PropertySwitch p)
        {
            auto ptr = p.findWidgetByName("CONNECT");
            if(!ptr) {
                LOG("No CONNECT in connection\n");
                return;
            }
            if(ptr->getState() != ISS_ON) {
                LOG("Connecting to %s\n",name_.c_str());
                connectDevice(name_.c_str());
            }
            else {
                LOG("Connected to camera %s\n",name_.c_str());
                connected_ = true;
            }
        }

        void handle_ccd_frame(INDI::PropertyNumber p)
        {
            if(frame_width_ > 0 && frame_height_ > 0)
                return;
            auto wp = p.findWidgetByName("WIDTH");
            auto hp = p.findWidgetByName("HEIGHT");
            
            if(!wp || !hp || wp->getMax() == 0 || hp->getMax() == 0)
                return;
            frame_width_  = wp->getMax();
            frame_height_ = hp->getMax();
            LOG("Configure CCD frame size = %dx%d\n",frame_width_,frame_height_);
        }

        void handle_ccd_binning(INDI::PropertyNumber p)
        {
            if(bin_ >= 1) {
                return;
            }
            auto vb = p.findWidgetByName("VER_BIN");
            auto hb = p.findWidgetByName("HOR_BIN");
            if(vb->getMin() != 1 || hb->getMin() != 1 || vb->getMax() != vb->getMax()) {
                LOG("Incompatible CCD_BINNING values\n");
                bin_ = 1;
            }
            else {
                bin_ = vb->getMax();
                LOG("Configure CCD bin = %d\n",bin_);
            }
        }
        void handle_capture_format(INDI::PropertySwitch p)
        {
            if(!p.isValid())
                return;
            std::string label;
            for(unsigned i=0;i<p.count();i++) {
                if(p[i].getState() == ISS_ON) {
                    label = p[i].getLabel();
                    for(char &c:label) {
                        c = std::tolower(c);
                    }
                    break;
                }
            }
            if(label.empty() || label.size() >= 256)
                return;
            char name[256] = {};
            int bits = -1;
            int n = sscanf(label.c_str(),"%s %d",name,&bits);
            if(n < 1)
                return;
            LOG("Parsing label %s bits =%d\n",name,bits);
            if(name == std::string("rgb")) {
                is_color_ = true;
                is_rgb_ = true;
                if(bits == -1)
                    current_bits_ = 8;
            }
            else if(name == std::string("mono") && bits != -1) {
                current_bits_ = bits;
                is_color_ = false;
                is_rgb_ = false;
            }
        }
        void handle_property(bool /*is_new*/,INDI::Property prop)
        {
            guard_type g(lock_);
            if(prop.isNameMatch("CONNECTION")) {
                handle_connection(prop);
            }
            else if(prop.isNameMatch("CCD_CAPTURE_FORMAT")) {
                handle_capture_format(prop);
            }
            else if(prop.isNameMatch("CCD_CONTROLS")) {
                ccd_controls_name_ = "CCD_CONTROLS";
                handle_controls(prop);
            }
            else if(prop.isNameMatch("Image Adjustments") && ccd_controls_name_ == nullptr) {
                ccd_controls_name_ = "Image Adjustments";
                handle_controls(prop);
            }
            else if(prop.isNameMatch("CCD_EXPOSURE")) {
                handle_exposure(prop);
            }
            else if(prop.isNameMatch("CCD_ISO") && prop.getType() == INDI_SWITCH) {
                handle_iso_switch(prop);
            }
            else if(prop.isNameMatch("CCD_FRAME")) {
                handle_ccd_frame(prop);
            }
            else if(prop.isNameMatch("CCD_BINNING")) {
                handle_ccd_binning(prop);
            }
            else if(prop.isNameMatch("V4L2_SIZE_DISCRETE")) {
                // No binning for V4L2
                bin_ = 1;
            }
            else if(prop.isNameMatch("CCD_CFA")) {
                is_color_ = true;
            }
            else if(prop.isNameMatch("CCD1")) {
                handle_frame(prop);
            }
            else if(prop.isNameMatch("CCD_TEMPERATURE")) {
                configure_num_param(prop,"CCD_TEMPERATURE_VALUE",opt_temperature,type_celsius,1.0,true);
                if(prop.getPermission() != IP_RO && parameters_.find(opt_temperature) != parameters_.end()) {
                    parameters_[opt_cooler_target] = parameters_[opt_temperature];
                    parameters_[opt_cooler_target].read_only = false;
                    parameters_[opt_cooler_target].option = opt_cooler_target;
                }
            }
            else if(prop.isNameMatch("CCD_COOLER_POWER")) {
                configure_num_param(prop,"CCD_COOLER_VALUE",opt_cooler_power_perc,type_percent);
            }
            else if(prop.isNameMatch("CCD_COOLER")) {
                configure_bool_switch_param(prop,"COOLER_ON","COOLER_OFF",opt_cooler_on);
            }
        }
        virtual void newProperty(INDI::Property prop) override
        {
            log_property("CN",prop);
            handle_property(true,prop);
        }
        virtual void updateProperty(INDI::Property prop) override
        {
            log_property("CU",prop);
            handle_property(false,prop);
        }


        CamStreamFormat stream_;
        int frame_counter_ = 0;
        std::string name_; 
        bool streaming_ = false;
        frame_callback_type callback_;
        std::map<CamOptionId,CamParam> parameters_;
        std::set<CamOptionId> deferred_;
        int bin_ = -1;
        int frame_width_ = -1, frame_height_ = -1;
        bool is_color_ = false;
        bool is_rgb_ = false;
        char const *ccd_controls_name_ = nullptr;
        std::vector<CamStreamFormat> formats_;
        int current_bits_ = 16;
        std::recursive_mutex lock_;
        bool connected_ = false;
        INDI::BaseDevice device_;
        bool controls_detected_ = false;
    };

    class IndiCameraDriver : public CameraDriver, public INDI::BaseClient
    {
    public:


        static std::string connection_string;

        IndiCameraDriver(CamErrorCode &e)
        {
            size_t pos = 0;
            if(connection_string.empty()) {
                host_ = "localhost";
                port_ = 7624;
            }
            else if((pos = connection_string.find(':')) != std::string::npos) {
                host_ = connection_string.substr(0,pos);
                port_ = atoi(connection_string.c_str() + pos + 1);
            }
            else {
                e = "Invalid connection string " + connection_string;
            }
            setServer(host_.c_str(),port_);
            int count = 0;
            bool connected;
            while((connected = connectServer()) == false && count < 10 ) {
                usleep(500000);
                count ++;
            }
            if(!connected) {
                e = "Failed to connect to server " + host_ + ":" + std::to_string(port_);
            }
        }
        virtual void newProperty(INDI::Property prop) override
        {
            log_property("DN",prop);
            if(prop.isNameMatch("DRIVER_INFO")) {
                INDI::PropertyText p(prop);
                auto interface = p.findWidgetByName("DRIVER_INTERFACE");
                if(interface && atoi(interface->getText()) & INDI::BaseDevice::CCD_INTERFACE) {
                    guard_type g(lock_);
                    add_device(prop.getDeviceName());
                }
            }
        }
        virtual void updateProperty(INDI::Property prop) override
        {
            log_property("DU",prop);
        }

        ~IndiCameraDriver()
        {
        }

        virtual std::vector<std::string> list_cameras(CamErrorCode &) override
        {
            for(int i=0;i<50;i++) {
                {
                    guard_type g(lock_);
                    if(!device_names_.empty())
                        return device_names_;
                }
                usleep(100000);
            }
            guard_type g(lock_);
            return device_names_;
        }

        virtual std::unique_ptr<Camera> open_camera(int id, CamErrorCode &e) override
        {
            guard_type g(lock_);
            if (size_t(id) >= device_names_.size())
            {
                e = "No such camera " + std::to_string(id);
                return std::unique_ptr<Camera>();
            }
            LOG("Disconnecting driver\n");
            disconnectServer();
            std::unique_ptr<Camera> p(new IndiCamera(device_names_[id],host_,port_));
            return p;
        }
        void add_device(char const *name)
        {
            for(size_t i=0;i<device_names_.size();i++) {
                if(device_names_[i] == name)
                    return;
            }
            device_names_.push_back(name);
        }


        std::vector<std::string> device_names_;
        std::string host_;
        int port_ = -1;
        std::recursive_mutex lock_;
    };

    std::string IndiCameraDriver::connection_string;


}
extern "C" {
    int ols_set_indi_driver_config(char const *s)
    {
        ols::IndiCameraDriver::connection_string = s;
        return 0;
    }
    void ols_set_indi_driver_log(char const *log_path,int debug)
    {
        if(debug) {
            ols::error_stream = fopen(log_path,"w");
        }
    }
    ols::CameraDriver *ols_get_indi_driver(int /*unused*/,ols::CamErrorCode *e)
    {
        std::unique_ptr<ols::IndiCameraDriver> driver(new ols::IndiCameraDriver(*e));
        if(*e) {
            driver.reset();
        }
        return driver.release();
    }
}
