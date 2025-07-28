#include "alpaca_client.h"
#include "alpaca_mount.h"
#include "mount.h"
#include "astronomy_calc.h"
#include "os_util.h"
#include <thread>

namespace ols {
    class AlpacaMount : public Mount {
    public:
        AlpacaMount(std::string const &conn_str,FILE *log_file):
            client_(conn_str,"telescope"),
            log_(log_file)
        {
            if(log_file)
                client_.set_logf(log_file);
            auto devices = client_.list_devices();
            if(devices.empty()) {
                throw std::runtime_error("No mount found");
            }
            client_.set_device(devices[0].second);
            client_.connect();
            if(!client_.is_connected())
                throw std::runtime_error("Failed to connect to " + conn_str);
            connected_ = true;
            thread_ = std::thread([=]() {
                poll();
            });
        }
        ~AlpacaMount()
        {
            {
                guard_type g(lock_);
                stop_ = true;
            }
            thread_.join();
        }
        void logex(std::exception const &ex)
        {
            if(log_) {
                fprintf(log_,"Exception %s\n",ex.what());
            }
        }
        void abort()
        {
            if(client_.get_value<bool>("/slewing"))
                client_.put("/abortslew",{});
            else
                client_.put("/tracking",{{"Tracking","false"}});
        }
        void poll()
        {
            bool check = true;
            while(true) {
                // every other time.
                // check stop every 1/2 second
                // send update every second
                check = !check;
                {
                    guard_type g(lock_);
                    if(stop_)
                        return;
                    if(check) {
                        EqCoord eq;
                        eq.RA  = client_.get_value<double>("/rightascension");
                        eq.DEC = client_.get_value<double>("/declination");
                        AltAzCoord altaz;
                        altaz.Alt = client_.get_value<double>("/altitude");
                        altaz.Az  = client_.get_value<double>("/azimuth");
                        if(callback_) {
                            callback_(eq,altaz,"");
                        }
                        check_alt_limits(altaz.Alt);
                    }
                }
                usleep(500000);
            }
        }
        virtual int get_alignment_points(MountErrorCode &) override
        {
            return -1;
        }
        virtual void reset_alignment(MountErrorCode &e) override
        {
            e = "Reset Alignment is not supported";
        }
        virtual MountTrac get_tracking(MountErrorCode &e) override
        {
            guard_type g(lock_);
            try {
                int state = client_.get_value<int>("/trackingrate");
                switch(state) {
                case 0: return trac_sidereal;
                case 1: return trac_lunar;
                case 2: return trac_solar;
                case 3: return trac_sidereal;
                default:
                    e = "Unknown tracking mode";
                    return trac_sidereal;
                }
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
                return trac_sidereal;
            }
        }
        virtual int get_tracking_state(MountErrorCode &e) override
        {
            guard_type g(lock_);
            try {
                return client_.get_value<bool>("/tracking");
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
                return -1;
            }

        }
        virtual void set_tracking_state(bool track,MountErrorCode &e) override
        {
            guard_type g(lock_);
            try {
                return client_.put("/tracking",{{"Tracking",std::string(track ? "true":"false")}});
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
            }
        }
        virtual void set_tracking(MountTrac t,MountErrorCode &e) override
        {
            guard_type g(lock_);
            try {
                return client_.put("/trackingrate",{{"TrackingRate",std::to_string(int(t))}});
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
            }
        }
        virtual void connect(bool ,MountErrorCode &) override
        {
        }
        virtual int supported_proto(MountErrorCode &) override
        {
            return proto_inet;
        }
        virtual void set_proto(MountProto proto,MountErrorCode &e) override
        {
            if(proto != proto_inet)
                e="Only network connection is supported";
        }
        virtual MountProto  get_proto(MountErrorCode &) override
        {
            return proto_inet;
        }
        /*
        std::string alpaca_config_file()
        {
            char const *home = getenv("HOME");
            if(home == nullptr)
                home = ".";
            std::string file = home;
            file += "/.alpaca_mount_config.txt";
            return file;
        }
        cppcms::json::value get_config()
        {
            std::ifstream f(alpaca_config_file());
            cppcm::json::value v;
            if(f && v.load(f,true)) {
                return v;
            }
            v=cppcms::json::object();
            return v;
        }
        void set_config(cppcms::json::value v)
        {
            std::ofstream f(alpaca_config_file());
            f<<v << std::flush;
        }*/
        virtual std::string get_conn_string(MountProto ,MountErrorCode &) override
        {
            guard_type g(lock_);
            return "";
        }
        virtual void set_conn_string(MountProto ,std::string const &,MountErrorCode &) override
        {
        }
        virtual void set_lat_lon(double lat,double lon,MountErrorCode &e) override
        {
            guard_type g(lock_);
            try {
                client_.put("/sitelatitude", {{"SiteLatitude",std::to_string(lat)}});
                client_.put("/sitelongitude",{{"SiteLongitude",std::to_string(lon)}});
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
            }
        }
        virtual std::pair<double,double> get_lat_lon(MountErrorCode &e) override
        {
            guard_type g(lock_);
            try {
                double lat = client_.get_value<double>("/sitelatitude");
                double lon = client_.get_value<double>("/sitelongitude");
                return std::make_pair(lat,lon);
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
                return std::make_pair(0.0,0.0);
            }
        }
        virtual EqCoord get_current(MountErrorCode &e) override
        {
            guard_type g(lock_);
            try {
                EqCoord coord;
                coord.RA  = client_.get_value<double>("/rightascension");
                coord.DEC = client_.get_value<double>("/declination");
                return coord;
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
                return EqCoord();
            }
        }
        virtual std::pair<int,int> get_slew_rate(MountErrorCode &e) override
        {
            guard_type g(lock_);
            try {
                if(rates_.empty()) {
                    cppcms::json::array ar0 = client_.get("/axisrates",{{"Axis","0"}})["Value"].array();
                    cppcms::json::array ar1 = client_.get("/axisrates",{{"Axis","1"}})["Value"].array();
                    for(unsigned i=0;i<std::min(ar0.size(),ar1.size());i++) {
                        double r0 = 0.5 * (ar0[i].get<double>("Minimum") + ar0[i].get<double>("Maximum"));
                        double r1 = 0.5 * (ar1[i].get<double>("Minimum") + ar1[i].get<double>("Maximum"));
                        rates_.push_back({r0,r1});
                    }
                    if(rates_.empty())
                        return {1,1};
                    if(log_) {
                        for(auto rs:rates_) {
                            fprintf(log_,"Using rates %f/%f\n",rs.first,rs.second);
                        }
                    }
                    cur_rate_ = rates_.size() / 2;
                }
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
            }
            return {cur_rate_,int(rates_.size())};
        }
        virtual void go_to(EqCoord coord,MountErrorCode &e) override
        {
            guard_type g(lock_);
            try {
                double RAd = coord.RA*15;
                double DECd = coord.DEC;
                Derotator dr(client_.get_value<double>("/sitelongitude"),
                             client_.get_value<double>("/sitelatitude"),
                             RAd,DECd);
                std::pair<float,float> altaz  = dr.alt_az_from_ra_dec(RAd,DECd,time(nullptr));
                if(!check_altitude_in_limits(altaz.first,e))
                    return;
                client_.put("/tracking",{{"Tracking","true"}});
                client_.put("/slewtocoordinatesasync", {
                    { "RightAscension", std::to_string(coord.RA) },
                    { "Declination",    std::to_string(coord.DEC) }
                });
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
            }
        }
        virtual void sync(EqCoord coord,MountErrorCode &e) override
        {
            guard_type g(lock_);
            try {
                client_.put("/synctocoordinates", {
                    { "RightAscension", std::to_string(coord.RA) },
                    { "Declination",    std::to_string(coord.DEC) }
                });
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
            }
        }
        double get_rate(int axis,int speed)
        {
            if(axis == 0)
                return rates_.at(speed).first;
            else
                return rates_.at(speed).second;
        }
        virtual void slew(MountSlew direction,int speed,MountErrorCode &e) override
        {
            guard_type g(lock_);
            try {
                bool call_ra=false,call_dec=false;
                double slew_ra = 0;
                double slew_dec = 0;
                cur_rate_ = std::max(0,std::min(int(rates_.size()) - 1,speed));
                if(direction == slew_stop)
                    call_dec = call_ra = true;
                if(direction & slew_NS_mask) {
                    call_dec = true;
                    slew_dec = get_rate(1,cur_rate_);
                    if((direction & slew_NS_mask) == slew_S)
                        slew_dec = -slew_dec;
                }
                if(direction & slew_WE_mask) {
                    call_ra = true;
                    slew_ra = get_rate(0,cur_rate_);
                    if((direction & slew_WE_mask) == slew_E)
                        slew_ra = -slew_ra;
                }
                if(call_ra)
                    client_.put("/moveaxis",{{"Axis","0"},{"Rate",std::to_string(slew_ra)}});
                if(call_dec)
                    client_.put("/moveaxis",{{"Axis","1"},{"Rate",std::to_string(slew_dec)}});
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
            }
        }
        virtual MountMeridianFlip get_meridian(MountErrorCode &) override
        {
            return on_meridian_unsupported;
        }
        virtual void set_meridian(MountMeridianFlip ,MountErrorCode &e) override
        {
            e="Meridian Control is not supported";
        }

        virtual bool pulse_guiding_supported(float &ns_guide_rate, float &we_guide_rate,MountErrorCode &e) override
        {
            guard_type g(lock_);
            try {
                if(!client_.get_value<bool>("/canpulseguide"))
                    return false;
                ns_guide_rate = we_guide_rate = 0.5;
                if(client_.get_value<bool>("/cansetguiderates")) {
                    ns_guide_rate = client_.get_value<double>("/guideratedeclination");
                    we_guide_rate = client_.get_value<double>("/guideraterightascension");
                }
                return true;
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
                return false;
            }

        }
        virtual void pulse_guide(float NS_ms,float WE_ms,MountErrorCode &e) override
        {
            guard_type g(lock_);
            try {
                if(NS_ms != 0) {
                    int dir = NS_ms > 0 ? 0 : 1;
                    client_.put("/pulseguide",{{"Direction",std::to_string(dir)},{"Duration",std::to_string(abs(int(NS_ms)))}});
                }
                if(WE_ms != 0) {
                    int dir = WE_ms > 0 ? 3 : 2;
                    client_.put("/pulseguide",{{"Direction",std::to_string(dir)},{"Duration",std::to_string(abs(int(WE_ms)))}});
                }
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
            }
        }
        
    private:
        std::vector<std::pair<double,double> > rates_;
        int cur_rate_ = 0;
        std::thread thread_;
        bool stop_ = false;
        AlpacaClient client_;
        FILE *log_;
    };

    class AlpacaMountDriver : public MountDriver {
    public:
        AlpacaMountDriver(std::string const &conn) :
            conn_(conn)
        {
            create_log_file();
        }
        void create_log_file()
        {
            char name[512];
            char prev_name[512];
            char const *fname = "debug/alpaca_mount_log.txt";
            char const *prev_fname = "debug/alpaca_mount_log.1.txt";
            #ifdef ANDROID_SUPPORT
            char const *home_dir = getenv("HOME");
            #else
            char const *home_dir = getenv("OLS_DATA_DIR");
            #endif    
            if(!home_dir) {
                fprintf(stderr,"Internal error OLS_DATA_DIR/HOME are not defined\n");
                log_ = stderr;
                return;
            }
            snprintf(name,sizeof(name),"%s/%s",home_dir,fname);
            snprintf(prev_name,sizeof(prev_name),"%s/%s",home_dir,prev_fname);
            rename(name,prev_name);
            log_ = fopen(name,"w");
            if(!log_) {
                fprintf(stderr,"Failed to open file [%s]\n",name);
                log_ = stderr;
            }
            else {
                setbuf(log_,nullptr);
            }

        }
        virtual std::unique_ptr<Mount> get_mount(MountErrorCode &e) override
        {
            std::unique_ptr<Mount> m;
            try {
                m.reset(new AlpacaMount(conn_,log_));
            }
            catch(std::exception const &ex) {
                e = ex;
            }
            return m;
        }
        virtual void shutdown() override
        {
            if(log_ && log_ != stderr) {
                fclose(log_);
                log_ = nullptr;
            }
        }
        virtual ~AlpacaMountDriver()
        {
            shutdown();
        }
    private:
        FILE *log_ = nullptr;
        std::string conn_;
    };
    
    std::unique_ptr<MountDriver> alpaca_start_driver(std::string const &opt,MountErrorCode &)
    {
        std::unique_ptr<MountDriver> d(new AlpacaMountDriver(opt));
        return d;
    }

}
