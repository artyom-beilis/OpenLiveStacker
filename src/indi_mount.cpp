#include "mount.h"
#include "indi_mount.h"

#ifdef INDI_AS_LIBRARY
#include "baseclient.h"
#include "basedevice.h"
#include "indicom.h"
#include "indiserver_lib.h"
#include "libastro.h"
#include <dirent.h>
#include <signal.h>
#else
#include <libindi/baseclient.h>
#include <libindi/basedevice.h>
#include <libindi/libastro.h>
#include <libindi/indicom.h>
#endif

#include <sys/time.h>
#include <iostream>
#include <sstream>
#include <string>
#include <mutex>
#include <thread>
#include <queue>
#include <regex>
#include "util.h"
#include <booster/log.h>

namespace ols {

namespace {
    struct LogFile {
        static FILE *log_file;
        LogFile() {
            if(!log_file) {
                char name[512];
                char prev_name[512];
                #ifdef ANDROID_SUPPORT
                char const *fname = "debug/indi_mount_prop_log.txt";
                char const *prev_fname = "debug/indi_mount_prop_log.1.txt";
                char const *home_dir = getenv("HOME");
                #else
                char const *fname = "indi_mount_prop_log.txt";
                char const *prev_fname = "indi_mount_prop_log.1.txt";
                char const *home_dir = "/tmp";
                #endif    
                snprintf(name,sizeof(name),"%s/%s",home_dir,fname);
                snprintf(prev_name,sizeof(prev_name),"%s/%s",home_dir,prev_fname);
                rename(name,prev_name);
                log_file = fopen(name,"w");
                if(!log_file) {
                    fprintf(stderr,"Failed to open file [%s]\n",name);
                    log_file = stderr;
                }
            }
        }
        ~LogFile()
        { 
            if(log_file) { 
                fflush(log_file);
            }
        }
    };
    FILE *LogFile::log_file;
}

    #define LOGP(...) do { LogFile l; fprintf(l.log_file,__VA_ARGS__); } while(0)

    static void log_property(char const *src,INDI::Property const &p) 
    {
        
        char const *perms[]={"ro","wo","rw"};
        struct timeval tv;
        struct tm t;
        gettimeofday(&tv, nullptr);
        localtime_r(&tv.tv_sec,&t);
        char ts[64];
        strftime(ts,sizeof(ts),"%H:%M:%S",&t);
        LOGP("%3s (%s.%03d) %s:%s (%s) %s type %s\n",src,ts,(int)tv.tv_usec/1000,p.getDeviceName(), p.getName(), p.getStateAsString(), perms[p.getPermission()], p.getTypeAsString());
        switch(p.getType()) {
            case INDI_NUMBER:
                {
                    INDI::PropertyNumber v(p);
                    for(size_t i=0;i<v.size();i++) {
                        LOGP("  %2d %s/%s val=%f min=%f max=%f step=%f\n",int(i),v[i].getName(),v[i].getLabel(),v[i].getValue(),v[i].getMin(),v[i].getMax(),v[i].getStep());
                    }
                }
                break;
            case INDI_TEXT:
                {
                    INDI::PropertyText v(p);
                    for(size_t i=0;i<v.size();i++) {
                        LOGP("  %2d %s/%s val=%s\n",int(i),v[i].getName(),v[i].getLabel(),v[i].getText());
                    }
                }
                break;
            case INDI_SWITCH:
                {
                    INDI::PropertySwitch v(p);
                    for(size_t i=0;i<v.size();i++) {
                        LOGP("  %2d %s/%s %s\n",int(i),v[i].getName(),v[i].getLabel(),(v[i].getState() == ISS_ON ? "ON " : "OFF"));
                    }
                }
                break;
            case INDI_LIGHT:
                {
                    INDI::PropertyLight v(p);
                    for(size_t i=0;i<v.size();i++) {
                        LOGP("  %2d %s/%s %s\n",int(i),v[i].getName(),v[i].getLabel(),v[i].getStateAsString());
                    }
                }
                break;
            case INDI_BLOB:
                {
                    INDI::PropertyBlob v(p);
                    for(size_t i=0;i<v.size();i++) {
                        LOGP("  %2d %s/%s ptr=%p size=%d\n",int(i),v[i].getName(),v[i].getLabel(),v[i].getBlob(),v[i].getSize());
                    }
                }
                break;
            default:
                ;
        }
    }

    class IndiMount : public Mount, public INDI::BaseClient {
    public:
        IndiMount(std::string const &device_name,std::string const &host,int port,bool disable_serial):
            device_name_(device_name),
            disable_serial_(disable_serial)
        {
            setServer(host.c_str(),port);
            if(!connectServer()) {
                throw MountError("Faild to connect to indi server");
            }
        }
        
        static double get_jd()
        {
            double jd = time(nullptr) / 86400.0 + 2440587.5;
            return jd;
        }

        AltAzCoord get_altitude_for_jnow(EqCoord eq)
        {
            INDI::IEquatorialCoordinates eod={eq.RA,eq.DEC};
            INDI::IHorizontalCoordinates altaz;
            double lon = lon_;
            if(lon < 0) {
                lon += 360;
            }
            double jd = get_jd();
            INDI::IGeographicCoordinates geo = { lon, lat_, 0 };
       
            INDI::EquatorialToHorizontal(&eod,&geo,jd,&altaz);
            return AltAzCoord{altaz.altitude,altaz.azimuth};
        }

        void abort()
        {
            MountErrorCode e;
            setPropSwitch("TELESCOPE_ABORT_MOTION","ABORT",e,true);
            if(e) {
                LOGP("Failed to abort %s\n",e.message().c_str());
            }
        }

        static EqCoord eod_to_j2000(EqCoord eq)
        {
            INDI::IEquatorialCoordinates eod={eq.RA,eq.DEC},j2000 = {0,0};
            INDI::ObservedToJ2000(&eod,get_jd(),&j2000);
            return EqCoord{j2000.rightascension,j2000.declination};
        }
        static EqCoord j2000_to_eod(EqCoord eq)
        {
            INDI::IEquatorialCoordinates j2000={eq.RA,eq.DEC},eod = {0,0};
            double jd = get_jd();
            INDI::J2000toObserved(&j2000,jd,&eod);
            EqCoord res{eod.rightascension,eod.declination};
            return res;
        }
    
        virtual int supported_proto(MountErrorCode &e) override
        {
            guard_type g(lock_);
            INDI::PropertySwitch p=device_.getProperty("CONNECTION_MODE");
            if(!p.isValid()) {
                return 0;
            }
            int res = 0;
            if(p.findWidgetByName("CONNECTION_TCP"))
                res |= proto_inet;
            auto wser = p.findWidgetByName("CONNECTION_SERIAL");
            if(wser) {
                if(!disable_serial_) {
                    res |= proto_serial;
                }
                else  {
                    if(wser->getState() == ISS_ON) {
                        switchTo(p,"CONNECTION_TCP",e);
                    }
                }
            }
            return res;
        }

        virtual MountProto get_proto(MountErrorCode &e) override
        {
            guard_type g(lock_);
            INDI::PropertySwitch p=device_.getProperty("CONNECTION_MODE");
            if(!p.isValid()) {
                return proto_none;
            }
            INDI::WidgetViewSwitch *w;
            if((w=p.findWidgetByName("CONNECTION_TCP"))!=nullptr && w->getState() == ISS_ON)
                return proto_inet;
            if((w=p.findWidgetByName("CONNECTION_SERIAL"))!=nullptr && w->getState() == ISS_ON) {
                if(disable_serial_) {
                    switchTo(p,"CONNECTION_TCP",e);
                    return proto_inet;
                }
                return proto_serial;
            }
            return proto_none;
        }

        virtual void set_proto(MountProto proto,MountErrorCode &e) override
        {
            guard_type g(lock_);
            char const *new_name = nullptr;
            switch(proto) {
            case proto_inet:
                new_name = "CONNECTION_TCP";
                break;
            case proto_serial:
                if(disable_serial_) {
                    e = "Unsupported protocol serial";
                    return;
                }
                new_name = "CONNECTION_SERIAL";
                break;
            default:
                e = "Unsupported protocol " + std::to_string(proto);
                return;
            }
            setPropSwitch("CONNECTION_MODE",new_name,e);
        }

        void to_lower(std::string &v)
        {
            for(char &e:v) {
                e = std::tolower(e);
            }
        }
        void to_upper(std::string &v)
        {
            for(char &e:v) {
                e = std::toupper(e);
            }
        }
        virtual std::string get_conn_string(MountProto p,MountErrorCode &e) override
        {
            guard_type g(lock_);
            std::string res;
            switch(p) {
            case proto_inet:
                {
                    MountErrorCode err;
                    res = getPropText("DEVICE_ADDRESS","ADDRESS",err) + ":" + getPropText("DEVICE_ADDRESS","PORT",err);
                    INDI::PropertySwitch ctype = device_.getProperty("CONNECTION_TYPE");
                    if(ctype.isValid()) {
                        for(unsigned i=0;i<ctype.count();i++) {
                            if(ctype[i].getState() == ISS_ON) {
                                std::string type = ctype[i].getName();
                                to_lower(type);
                                res = type + "/" + res;
                                break;
                            }
                        }
                    }
                    if(err)
                        return "";
                }
                break;
            case proto_serial:
                {
                    MountErrorCode err;
                    res = getPropText("DEVICE_PORT","PORT",err);
                    if(err)
                        return "";
                }
                break;
            default:
                e = "Unsupported protocol op " + std::to_string(p);
            }
            return res;
        }
        virtual MountMeridianFlip get_meridian(MountErrorCode &) override
        {
            guard_type g(lock_);
            INDI::PropertySwitch p = device_.getProperty("MERIDIAN_ACTION");
            if(p.isValid()) {
                for(unsigned i=0;i<p.count();i++) {
                    if(p[i].getState() == ISS_ON) {
                        if(p[i].getName() == std::string("IOP_MB_STOP"))
                            return on_meridian_stop;
                        else if(p[i].getName() == std::string("IOP_MB_FLIP"))
                            return on_meridian_flip;
                    }
                }
                return on_meridian_unsupported;
            }
            p = device_.getProperty("TELESCOPE_PIER_SIDE");
            if(!p.isValid())
                return on_meridian_unsupported;
            p = device_.getProperty("TELESCOPE_TRACK_STATE");
            if(!p.isValid())
                return on_meridian_unsupported;
            if(!p.findWidgetByName("TRACK_ON"))
                return on_meridian_unsupported;
            return flip_;
        }

        virtual void set_meridian(MountMeridianFlip flip,MountErrorCode &e) override
        {
            guard_type g(lock_);
            if(flip == on_meridian_unsupported) {
                e="Can't set unsuppored config";
                return;
            }

            INDI::PropertySwitch p = device_.getProperty("MERIDIAN_ACTION");
            if(p.isValid()) {
                switchTo(p,(flip == on_meridian_flip ? "IOP_MB_FLIP" : "IOP_MB_STOP"),e);
                return;
            }
            p = device_.getProperty("TELESCOPE_PIER_SIDE");
            if(!p.isValid()) {
                e = "Meridian action is not supported";
            }
            flip_ = flip;
        }

        virtual void set_conn_string(MountProto p,std::string const &s,MountErrorCode &e) override
        {
            guard_type g(lock_);
            switch(p) {
            case proto_serial:
                setPropText("DEVICE_PORT",{std::string("PORT"),s},e);
                break;
            case proto_inet:
                {
                    std::regex r("((.+)/)?(.+):(.+)");
                    std::smatch m;
                    if(!std::regex_match(s,m,r)) {
                        e="Invalud address string " + s;
                        return;
                    }

                    setPropText("DEVICE_ADDRESS",{ "ADDRESS",m[3].str(), "PORT",m[4].str() },e);
                    if(m[1].matched) {
                        std::string type = m[2].str();
                        to_upper(type);
                        std::cerr << "!! setting " << type << std::endl;
                        setPropSwitch("CONNECTION_TYPE",type,e);
                    }
                }
                break;
            default:
                e="Unsupported protocol type";
            }
        }
        virtual void connect(MountErrorCode & /*e*/) override
        {
            guard_type g(lock_);
            LOGP("Connecting to %s\n",device_name_.c_str());
            connectDevice(device_name_.c_str());
        }

        virtual void set_lat_lon(double lat,double lon,MountErrorCode &/*e*/)  override
        {
            guard_type g(lock_);
            pending_lat_long_ = true;
            lat_ = lat;
            lon_ = lon;
            set_geolocation(device_.getProperty("GEOGRAPHIC_COORD"));
        }
        virtual std::pair<double,double> get_lat_lon(MountErrorCode &) override
        {
            guard_type g(lock_);
            if(pending_lat_long_) {
                return std::make_pair(lat_,lon_);
            }
            INDI::PropertyNumber p = device_.getProperty("GEOGRAPHIC_COORD");
            auto wlat = p.findWidgetByName("LAT");
            auto wlon = p.findWidgetByName("LONG");
            double lat=0,lon=0;
            if(wlat && wlon) {
                lat = wlat->getValue();
                lon = wlon->getValue();
            }
            lat_ = lat;
            lon_ = lon;
            return std::make_pair(lat,lon);
        }
        virtual EqCoord get_current(MountErrorCode &/*e*/) override
        {
            guard_type g(lock_);
            return eod_to_j2000(EqCoord{RA_,DEC_});
        }
        virtual std::pair<int,int> get_slew_rate(MountErrorCode &/*e*/) override
        {
            guard_type g(lock_);
            INDI::PropertySwitch p = device_.getProperty("TELESCOPE_SLEW_RATE");
            if(p.isValid()) {
                int max_v = p.size();
                int cur_v = std::min(max_v/2,max_v-1);
                for(int i=0;i<max_v;i++) {
                    if(p[i].getState()==ISS_ON) {
                        cur_v = i;
                        break;
                    }
                }
                return std::make_pair(cur_v,max_v);
            }
            return std::make_pair(2,4); // just some default
        }
        bool check_altitude_in_limits(double alt,MountErrorCode &e)
        {
            if(alt < low_alt_ || alt > high_alt_) {
                std::ostringstream ss;
                ss << "Altitude of the target " << alt << " is not withing limits [" <<low_alt_ << ","<<high_alt_ << "]";
                e = ss.str();
                return false;
            }
            return true;
        }
        void go_to_jnow_no_check(EqCoord coord,MountErrorCode &e) 
        {
            setPropSwitch("ON_COORD_SET","TRACK",e,true);
            if(e)
                return;
            setPropNumeric("EQUATORIAL_EOD_COORD",{
                { "RA",  coord.RA  },
                { "DEC", coord.DEC }
            },e,true);
        }
        virtual void go_to(EqCoord coord,MountErrorCode &e) override
        {
            guard_type g(lock_);
            coord = j2000_to_eod(coord);
            double alt = get_altitude_for_jnow(coord).Alt;
            if(!check_altitude_in_limits(alt,e))
                return;
            go_to_jnow_no_check(coord,e);
        }

        virtual void sync(EqCoord coord,MountErrorCode &e) override
        {
            guard_type g(lock_);
            coord = j2000_to_eod(coord);
            setPropSwitch("ON_COORD_SET","SYNC",e,true);
            if(e)
                return;
            setPropNumeric("EQUATORIAL_EOD_COORD",{
                { "RA",  coord.RA  },
                { "DEC", coord.DEC }
            },e,true);
            if(e)
                return;
            save_alignment(e);
        }
        virtual void slew(MountSlew direction,int speed,MountErrorCode &e) override
        {
            guard_type g(lock_);
            if(direction == slew_stop) {
                setPropSwitch("TELESCOPE_MOTION_NS","",e,true);
                setPropSwitch("TELESCOPE_MOTION_WE","",e,true);
                return;
            }
            INDI::PropertySwitch p = device_.getProperty("TELESCOPE_SLEW_RATE");
            if(p.isValid()) {
                speed = std::min(speed,int(p.size())-1);
                bool update = false;
                for(int i=0;i<int(p.size());i++) {
                    auto state = (i==speed) ? ISS_ON:ISS_OFF;
                    if(p[i].getState()!=state) {
                        p[i].setState(state);
                        update = true;
                    }
                }
                if(update)
                    sendNewProperty(p);
            }
            
            if(direction & slew_NS_mask) {
                if((direction & slew_NS_mask) == slew_N)
                    setPropSwitch("TELESCOPE_MOTION_NS","MOTION_NORTH",e);
                else
                    setPropSwitch("TELESCOPE_MOTION_NS","MOTION_SOUTH",e);
            }
            if(direction & slew_WE_mask) {
                if((direction & slew_WE_mask) == slew_W)
                    setPropSwitch("TELESCOPE_MOTION_WE","MOTION_WEST",e);
                else
                    setPropSwitch("TELESCOPE_MOTION_WE","MOTION_EAST",e);
            }
        }
        
        virtual MountTrac get_tracking(MountErrorCode &e) override
        {
            guard_type g(lock_);
            MountTrac t = trac_sidereal;
            INDI::PropertySwitch p = device_.getProperty("TELESCOPE_TRACK_MODE");
            if(!p.isValid())
                return t;
            for(unsigned i=0;i<p.size();i++) {
                if(p[i].getState()==ISS_ON) {
                    std::string name = p[i].getName();
                    if(name == "TRACK_SIDEREAL")
                        return trac_sidereal;
                    else if(name == "TRACK_SOLAR")
                        return trac_solar;
                    else if(name == "TRACK_LUNAR")
                        return trac_lunar;
                    else {
                        switchTo(p,"TRACK_SIDEREAL",e);
                        return trac_sidereal;
                    }
                }
            }
            switchTo(p,"TRACK_SIDEREAL",e);
            return trac_sidereal;
        }
        virtual void set_tracking(MountTrac t,MountErrorCode &e) override
        {
            guard_type g(lock_);
            switch(t) {
            case trac_sidereal: setPropSwitch("TELESCOPE_TRACK_MODE","TRACK_SIDEREAL",e); break;
            case trac_solar:    setPropSwitch("TELESCOPE_TRACK_MODE","TRACK_SOLAR",e); break;
            case trac_lunar:    setPropSwitch("TELESCOPE_TRACK_MODE","TRACK_LUNAR",e); break;
            }
        }
         
    protected:

        virtual void newMessage(INDI::BaseDevice baseDevice, int messageID) override
        {
            guard_type g(lock_);
            std::string msg = baseDevice.messageQueue(messageID);
            LOGP("GOT MESSAGE: id=%d: %s\n",messageID,msg.c_str());
            if(msg.find("[ERROR]")!=std::string::npos && callback_) {
                callback_(EqCoord{0,0},AltAzCoord{0,0},msg);
            }
        }

        void sendNewProperty(INDI::Property prop)
        {
            log_property("SND",prop);
            INDI::BaseClient::sendNewProperty(prop);
        }

        virtual void updateProperty(INDI::Property prop) override
        {
            guard_type g(lock_);
            handle(false,prop);
        }
        virtual void newProperty(INDI::Property prop) override
        {
            guard_type g(lock_);
            handle(true,prop);
        }


        void handle(bool is_new,INDI::Property prop)
        {
            log_property(is_new?"NEW":"UPD",prop);
            if(prop.isNameMatch("DRIVER_INFO") && !device_loaded_) {
                INDI::PropertyText p(prop);
                auto interface = p.findWidgetByName("DRIVER_INTERFACE");
                if(interface && atoi(interface->getText()) & INDI::BaseDevice::TELESCOPE_INTERFACE) {
                    if(device_name_.empty() || prop.getDeviceName() == device_name_) {
                        device_name_ = prop.getDeviceName();
                        device_ = prop.getBaseDevice();
                        watchDevice(device_name_.c_str());
                        device_loaded_ = true;
                    }
                }
            }
            else if(prop.isNameMatch("EQUATORIAL_EOD_COORD")) {
                log_ra_de(prop);
                coord_update_time_ = now();
            }
            else if(prop.isNameMatch("GEOGRAPHIC_COORD")) {
                if(pending_lat_long_) {
                    set_geolocation(prop);
                }
                else {
                    MountErrorCode e;
                    get_lat_lon(e);
                }
            }
            else if(prop.isNameMatch("CONNECTION")) {
                handle_connection(prop);
            }
            else if(prop.isNameMatch("ALIGNMENT_POINTSET_ACTION")) {
                align_pointset_action_ = true;
            }
            else if(prop.isNameMatch("ALIGNMENT_POINTSET_COMMIT")) {
                align_pointset_commit_ = true;
            }
            else if(prop.isNameMatch("POLLING_PERIOD") && is_new) {
                handle_polling(prop);
            }
            else if(coord_update_time_ != 0 && (now() - coord_update_time_) > 1.1)  {
                // in case there are updates but not coordinates update
                INDI::PropertyNumber p=device_.getProperty("EQUATORIAL_EOD_COORD");
                if(p.isValid()) {
                    log_ra_de(p);
                    coord_update_time_ = now();
                }
            }
            if(align_pointset_commit_ && align_pointset_action_ && !align_pointset_loaded_) {
                align_pointset_loaded_ = true;
                load_alignment();
            }
       }
    private:
        static double now()
        {
            struct timeval tv;
            gettimeofday(&tv,nullptr);
            double unix_timestamp = tv.tv_sec;
            unix_timestamp += tv.tv_usec * 1e-6;
            return unix_timestamp;
        }
        void handle_polling(INDI::PropertyNumber p)
        {
            if(!p.isValid() || p.count() != 1) {
                LOGP("Invalid polling prop format\n");
                return;
            }
            double val = p[0].getValue();
            if(val < 1000.0) {
                p[0].setValue(1000.0);
                sendNewProperty(p);
            }
        }
        void load_alignment()
        {
            MountErrorCode e;
            setPropSwitch("ALIGNMENT_POINTSET_ACTION","LOAD DATABASE",e,true);
            if(e) {
                LOGP("No load database for mount: %s\n",e.message().c_str());
                return;
            }
            setPropSwitch("ALIGNMENT_POINTSET_COMMIT","ALIGNMENT_POINTSET_COMMIT",e,true);
            if(e) {
                LOGP("Can't commit for load database mount: %s\n",e.message().c_str());
                return;
            }
        }
        
        void save_alignment(MountErrorCode &e)
        {
            if(!(align_pointset_action_ && align_pointset_commit_)) {
                return; // No alignment supported
            }
            setPropSwitch("ALIGNMENT_POINTSET_ACTION","SAVE DATABASE",e,true);
            if(e) {
                return;
            }
            setPropSwitch("ALIGNMENT_POINTSET_COMMIT","ALIGNMENT_POINTSET_COMMIT",e,true);
        }

        void handle_connection(INDI::PropertySwitch p)
        {
            auto ptr = p.findWidgetByName("CONNECT");
            if(ptr && ptr->getState() == ISS_ON) {
                connected_ = true;
            }
            else {
                connected_ = false;
            }
            LOGP("CONNECION status = %d\n",int(connected_));
        }

        virtual int get_alignment_points(MountErrorCode &) override
        {
            guard_type g(lock_);
            INDI::PropertyNumber p = device_.getProperty("ALIGNMENT_POINTSET_SIZE");
            if(!p.isValid())
                return -1;
            auto w = p.findWidgetByName("ALIGNMENT_POINTSET_SIZE");
            if(!w)
                return -1;
            if(w->getValue() > 3)
                return 1000;
            return w->getValue();
        }
        virtual void reset_alignment(MountErrorCode &e) override
        {
            guard_type g(lock_);
            setPropSwitch("ALIGNMENT_POINTSET_ACTION","CLEAR",e,true);
            if(e)
                return;
            setPropSwitch("ALIGNMENT_POINTSET_COMMIT","ALIGNMENT_POINTSET_COMMIT",e,true);
            if(e)
                return;
            save_alignment(e);
        }

        bool check_flip_for_RA(double RA)
        {
            INDI::PropertySwitch p = device_.getProperty("MERIDIAN_ACTION");
            if(p.isValid())
                return false; // handled by driver

            p = device_.getProperty("TELESCOPE_PIER_SIDE");
            if(!p.isValid())
                return false;
            auto pw = p.findWidgetByName("PIER_WEST");
            auto pe = p.findWidgetByName("PIER_EAST");
            if(!pw || !pe)
                return false;
            bool pier_east = pe->getState() == ISS_ON;
            p = device_.getProperty("TELESCOPE_TRACK_STATE");
            if(!p.isValid())
                return false;
            auto w_ton = p.findWidgetByName("TRACK_ON");
            if(w_ton && w_ton->getState() == ISS_OFF) 
                return false;
            auto st = device_.getProperty("EQUATORIAL_EOD_COORD").getState();
            if(st != IPS_OK && st != IPS_IDLE) 
                return false;

            
            double lst = get_local_sidereal_time(lon_);
            double ha = get_local_hour_angle(lst,RA);
            if(pier_east) {
                ha = rangeHA(ha + 12);
            }
            if(ha >= 9) {
                ha -= 24;
            }
            double minutes_to_flip = -ha * 60;
            if(0 < minutes_to_flip && minutes_to_flip <= 1.0 && (minutes_to_flip_prev_ > 1.0 || minutes_to_flip_prev_ == -24*60) ) {
                LOGP("1 Minute to reaching meridian\n");
                if(callback_) {
                    callback_(EqCoord{0,0},AltAzCoord{0,0},"One minute to reaching meridian");
                }
            }
            minutes_to_flip_prev_ = minutes_to_flip;
            bool meridian_reached = minutes_to_flip <= 0;
            if(fabs(minutes_to_flip) < 10.0/60.0) {
                LOGP("Close to meridian RA=%.5f LST=%.5f HA=%.5f minutes_to_flip=%.5f pier_east=%d reached=%d\n",RA,lst,ha,minutes_to_flip,int(pier_east),int(meridian_reached));
            }
            return meridian_reached;
        }

        void log_ra_de(INDI::PropertyNumber prop) 
        {
            if(!prop.isValid())
                return;
            auto ra = prop.findWidgetByName("RA");
            auto de = prop.findWidgetByName("DEC");
            if(!ra || !de)
                return;
            RA_  = ra->getValue();
            DEC_ = de->getValue();

            EqCoord coord = EqCoord{RA_,DEC_};
            AltAzCoord altaz = get_altitude_for_jnow(coord);
            if(callback_) {
                callback_(eod_to_j2000(coord),altaz,"");
            }

            if(check_flip_for_RA(RA_)) {
                if(flip_ == on_meridian_stop) {
                    LOGP("Meridian reached: stopping per configuration\n");
                    abort();
                    if(callback_) {
                        callback_(EqCoord{0,0},AltAzCoord{0,0},"Aborting motion, meridian reached");
                    }
                }
                else {
                    LOGP("Meridian reached: starting flip\n");
                    MountErrorCode e;
                    go_to_jnow_no_check(coord,e);
                    if(callback_) {
                        if(e) {
                            callback_(EqCoord{0,0},AltAzCoord{0,0},"Flip failed: " + e.message());
                        }
                        else {
                            callback_(EqCoord{0,0},AltAzCoord{0,0},"Initiating meridian flip");
                        }
                    }
                }
                return;
            }

            double alt = altaz.Alt;
            MountErrorCode e;
            if(!check_altitude_in_limits(alt,e)) {
                bool call_abort = true;
                if(alt > high_alt_) {
                    if(prev_alt_ > -100 && prev_alt_ >= alt) {
                        call_abort = false;
                    }
                }
                else if(alt < low_alt_) {
                    if(prev_alt_ > -100 && prev_alt_ <= alt) {
                        call_abort = false;
                    }
                }
                LOGP("Altitude is not withing limits %s, previous %f aborting = %d\n",e.message().c_str(),prev_alt_,int(call_abort));
                if(call_abort) {
                    abort();
                    if(callback_) {
                        callback_(EqCoord{0,0},AltAzCoord{0,0},"Aborting motion: " + e.message());
                    }
                }
            }
            prev_alt_ = alt;
        }
        void set_geolocation(INDI::PropertyNumber p)
        {
            if(!p.isValid())
                return;
            auto lat = p.findWidgetByName("LAT");
            auto lon = p.findWidgetByName("LONG");
            if(!lat || !lon) {
                LOGP("NO WID\n");
                return;
            }
            lat->setValue(lat_);
            lon->setValue(lon_);
            LOGP("Setting new lat/long according to local setup\n");
            sendNewProperty(p);
            pending_lat_long_ = false;
        }
        
        std::string getPropText(std::string const &prop_name,std::string const &name,MountErrorCode &e)
        {
            std::string res;
            if(e) {
                return res;
            }
            INDI::PropertyText p = device_.getProperty(prop_name.c_str());
            if(!p.isValid()) {
                e="No " + prop_name + " avalible";
                return res;
            }
            for(unsigned i=0;i<p.size();i++) {
                if(p[i].isNameMatch(name.c_str())) {
                    return p[i].getText();
                }
            }
            e = "No " + name + " in " + prop_name;
            return res;
        }
        
        void setPropNumeric(std::string const &prop_name,std::vector<std::pair<std::string,double>> key_value_pairs,MountErrorCode &e,bool force=false)
        {
            if(e) {
                return;
            }
            INDI::PropertyNumber p = device_.getProperty(prop_name.c_str());
            if(!p.isValid()) {
                e="No " + prop_name + " avalible";
                return;
            }

            bool update = force;
            for(unsigned i=0;i<key_value_pairs.size();i++) {
                std::string const &key = key_value_pairs[i].first;
                double val = key_value_pairs[i].second;
                auto ptr = p.findWidgetByName(key.c_str());
                if(!ptr) {
                    e = "No value " + key;
                    return;
                }
                if(ptr->getValue() != val) {
                    ptr->setValue(val);
                    update = true;
                }
            }
            if(update)
                sendNewProperty(p);
        }
        
        void setPropText(std::string const &prop_name,std::vector<std::string> key_value_pairs,MountErrorCode &e,bool force=false)
        {
            if(e) {
                return;
            }
            if((key_value_pairs.size() % 2)!=0) {
                e="Internal error invalud input set prop text for " + prop_name;
            }
            INDI::PropertyText p = device_.getProperty(prop_name.c_str());
            if(!p.isValid()) {
                e="No " + prop_name + " avalible";
                return;
            }

            bool update = force;
            for(unsigned i=0;i<key_value_pairs.size();i+=2) {
                std::string const &key = key_value_pairs[i];
                std::string const &val = key_value_pairs[i+1];
                auto ptr = p.findWidgetByName(key.c_str());
                if(!ptr) {
                    e = "No value " + key;
                    return;
                }
                if(ptr->getText() != val) {
                    ptr->setText(val.c_str());
                    update = true;
                }
            }
            if(update)
                sendNewProperty(p);
        }

        void setPropSwitch(std::string const &prop_name,std::string const &name,MountErrorCode &e,bool force = false)
        {
            switchTo(device_.getProperty(prop_name.c_str()),name.c_str(),e,force);
        }
        void switchTo(INDI::PropertySwitch p,std::string const &name,MountErrorCode &e,bool force = false)
        {
            if(e) return;
            if(!p.isValid()) {
                e = "Invalid property";
                return;
            }
            if(!name.empty() && !p.findWidgetByName(name.c_str())) {
                e = "No " + name + "found";
                return;
            }
            bool update = force;
            for(unsigned i=0;i<p.count();i++) {
                if(p[i].isNameMatch(name.c_str())) {
                    if(p[i].getState() != ISS_ON) {
                        p[i].setState(ISS_ON);
                        update = true;
                    }
                }
                else {
                    if(p[i].getState() != ISS_OFF) {
                        p[i].setState(ISS_OFF);
                        update = true;
                    }
                }
            }
            if(update) {
                sendNewProperty(p);
            }
        }
    
        std::string device_name_;
        MountMeridianFlip flip_ = on_meridian_stop;
        bool device_loaded_ = false;
        bool align_pointset_action_ = false;
        bool align_pointset_commit_ = false;
        bool align_pointset_loaded_ = false;
        bool pending_save_ = false;
        bool disable_serial_;
        double coord_update_time_ = 0.0;
        double prev_alt_ = -100;
        double minutes_to_flip_prev_ = -24*60;
        INDI::BaseDevice device_;
    };


#ifdef INDI_AS_LIBRARY
    class LocalIndiMountDriver : public MountDriver  {
    public:
        LocalIndiMountDriver(std::string const &path)
        {
            std::thread main([=]() {
                    indiserver_main({path});
            });
            usleep(500000);
            indiserver_main_reset_sigchld();
            worker_ =std::move(main);
        }
        virtual void shutdown() override
        {
            indiserver_main_shutdown();
            worker_.join();
        }
        virtual std::unique_ptr<Mount> get_mount(MountErrorCode &e) override
        {
            try {
                #ifdef ANDROID_SUPPORT
                bool disable_serial = true;
                #else
                bool disable_serial = false;
                #endif
                std::unique_ptr<Mount> m(new IndiMount("","localhost",7624,disable_serial));
                return m;
            }
            catch(std::exception const &err) {
                e = err;
                return std::unique_ptr<Mount>();
            }
        }

    private:
        std::thread worker_;
    };

#endif    
    class RemoteIndiMountDriver : public MountDriver  {
    public:
        RemoteIndiMountDriver(std::string const &host,int port) :
            host_(host),
            port_(port)
        {
        }
        virtual std::unique_ptr<Mount> get_mount(MountErrorCode &e) override
        {
            try {
                std::unique_ptr<Mount> m(new IndiMount("",host_,port_,false));
                return m;
            }
            catch(std::exception const &err) {
                e = err;
                return std::unique_ptr<Mount>();
            }
        }
        virtual void shutdown() override
        {
        }

    private:
        std::string host_;
        int port_;
    };


    void indi_list_drivers(std::vector<DriverInfo> &drivers,std::string const &
            #ifdef INDI_AS_LIBRARY
            libdir
            #endif
    )
    {
        drivers.push_back(DriverInfo("indi:remote","localhost:7624",true));
#ifdef INDI_AS_LIBRARY
        size_t start_pos = drivers.size();
        DIR *d = opendir(libdir.c_str());
        if(!d)
            return;
        std::string prefix = "libindidriver_telescope_";
        std::string suffix = ".so";
        for(struct dirent *de = readdir(d);de;de=readdir(d)) {
            std::string fname = de->d_name;
            size_t spos;
            if(fname.find(prefix) == 0 && (spos = fname.find(suffix))!=std::string::npos && spos + suffix.size() == fname.size()) {
                std::string driver_name = fname.substr(prefix.size(),fname.size() - suffix.size() - prefix.size());
                std::cerr << "Driver :" << driver_name<< std::endl;
                drivers.push_back(DriverInfo("indi:" + driver_name));
            }
        }
        closedir(d);
        std::sort(drivers.begin() + start_pos,drivers.end(),[](DriverInfo const &l,DriverInfo const &r) -> bool {
            return l.name < r.name;
        }); 
#endif        
    }

    std::unique_ptr<MountDriver> indi_start_driver(std::string const &d,std::string const &opt,
            #ifdef INDI_AS_LIBRARY
                std::string const &libdir,
            #else                
                std::string const &,
            #endif
            MountErrorCode &e
        )
    {
        std::unique_ptr<MountDriver> m;
        if(d == "indi:remote") {
            std::string indi_host = "localhost";
            int indi_port = 7624;
            size_t pos = opt.find(':');
            if(pos == std::string::npos) {
                if(!opt.empty())
                    indi_host = opt;
            }
            else {
                if(opt.size() > pos+1)
                    indi_port = atoi(opt.c_str()+pos+1);
                if(pos > 0)
                    indi_host = opt.substr(0,pos);
            }
            m.reset(new RemoteIndiMountDriver(indi_host,indi_port));
        }
        #ifdef INDI_AS_LIBRARY
        else if(d.find("indi:") == 0) {
            std::string driver = d.substr(5);
            std::string path = libdir + "/libindidriver_telescope_" + driver + ".so";
            if(!exists(path)) {
                e="No such file " + path;
            }
            else {
                m.reset(new LocalIndiMountDriver(path));
            }
        }
        #endif
        else {
            e = "Invalid indi driver " + d;
        }
        return m;
    }
    
} // ols
