#pragma once 
#include <stdexcept>
#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <memory>
#include <sstream>

namespace ols {
    struct EqCoord {
        double RA = 0;
        double DEC = 0;
    };
    struct AltAzCoord {
        double Alt = 0;
        double Az = 0;
    };

    class MountError : public std::runtime_error {
    public:
        MountError(std::string const &msg) : std::runtime_error(msg)
        {
        }
    };

    class MountErrorCode {
    public:
        MountErrorCode() : status_(false), message_("ok") {}
        MountErrorCode(std::string const &msg) :status_(true), message_(msg) {}
        MountErrorCode(std::exception const &e) : status_(true), message_(e.what())
        {
        }
        MountErrorCode &operator=(std::string const &msg)
        {
            status_ = true;
            message_ = msg;
            return *this;
        }
        MountErrorCode(MountErrorCode const &) = default;
        MountErrorCode &operator=(MountErrorCode const &) = default;
        explicit operator bool() const
        {
            return status_;
        }
        std::string const &message() const
        {
            return message_;
        }
        void check()
        {
            if(status_)
                throw MountError(message_);
        }
    private:
        bool status_;
        std::string message_;
    };

    enum MountProto {
        proto_none   = 0,
        proto_serial = 1,
        proto_inet   = 2,
    };

    enum MountTrac {
        trac_sidereal,
        trac_solar,
        trac_lunar
    };

    enum MountMeridianFlip {
        on_meridian_unsupported,
        on_meridian_flip,
        on_meridian_stop,
    };

    enum MountSlew {
        slew_stop    = 0,
        slew_NS_mask = (3<<1),
        slew_N       = (1<<1),
        slew_S       = (3<<1),
        slew_WE_mask = (3<<3),
        slew_W       = (1<<3),
        slew_E       = (3<<3),
    };

    typedef std::function<void(EqCoord,AltAzCoord,std::string const &)> mount_callback_type;
    class Mount {
    public:
        typedef std::unique_lock<std::recursive_mutex> guard_type;
        
        Mount();
        virtual ~Mount();

        static double parseRA(int h,int m,int s)
        {
            double hours = (h + (60 * m + s) / 3600.0 );
            return hours;
        }

        static double parseDEC(int d,int m,int s)
        {
            int sig=1;
            if(d < 0) {
                sig = -1;
                d = -d;
            }
            return sig*(d + (60 * m + s) / 3600.0);
        }

        static void splitRA(double RA,int &rh,int &rm,int &rs)
        {
            rh = int(RA);
            rs = int(RA*3600) % 3600;
            rm = rs / 60;
            rs = rs % 60;
        }
        static void splitDEC(double DEC,int &dd,int &dm,int &ds)
        {
            int s = 1;
            if(DEC < 0) {
                s=-1;
                DEC=-DEC;
            }
            dd = int(DEC);
            ds = int(DEC*3600) % 3600;
            dm = ds / 60;
            ds = ds % 60;
            dd = dd * s;
        }

        static std::string formatRA(double RA)
        {
            int h,m,s;
            splitRA(RA,h,m,s);
            char buf[64];
            snprintf(buf,sizeof(buf),"%02d:%02d:%02d",h,m,s);
            return buf;
        }
        static std::string formatDEC(double DEC)
        {
            int d,m,s;
            splitDEC(DEC,d,m,s);
            char buf[64];
            snprintf(buf,sizeof(buf),"%02d:%02d:%02d",d,m,s);
            return buf;
        }

        static double parseRA(std::string const &s,MountErrorCode &e)
        {
            std::istringstream ss(s);
            int rh=0,rm=0,rs=0;
            char c;
            ss >> rh >>c >> rm >> c >> rs;
            if(!ss) {
                e = "Invalid RA string " + s;
                return 0;
            }
            return parseRA(rh,rm,rs);
        }
        static double parseDEC(std::string const &s,MountErrorCode &e)
        {
            std::istringstream ss(s);
            int dd=0,dm=0,ds=0;
            char c;
            ss >> dd >>c >> dm >> c >> ds;
            if(!ss) {
                e = "Invalid DEC string " + s;
                return 0;
            }
            return parseDEC(dd,dm,ds);
        }

        virtual std::pair<int,int> get_alt_limits(MountErrorCode &)
        {
            return std::make_pair(low_alt_,high_alt_);
        }
        virtual void set_alt_limits(int low,int high,MountErrorCode &e)
        {
            check_alt_limits(low,high,e);
            low_alt_ = low;
            high_alt_ = high;
            save_alt(e);
        }
        virtual int get_alignment_points(MountErrorCode &e) = 0;
        virtual void reset_alignment(MountErrorCode &e) = 0;
        virtual MountTrac get_tracking(MountErrorCode &e) = 0;
        // return -1 if not reported 1 is tracking 0 is not
        virtual int get_tracking_state(MountErrorCode &e) = 0;
        virtual void set_tracking_state(bool track,MountErrorCode &e) = 0;
        virtual void set_tracking(MountTrac t,MountErrorCode &e) = 0;
        virtual void connect(bool load_alignent,MountErrorCode &e) = 0;
        virtual int supported_proto(MountErrorCode &e) = 0;
        virtual void set_proto(MountProto proto,MountErrorCode &e) = 0;
        virtual MountProto  get_proto(MountErrorCode &e) = 0;
        virtual std::string get_conn_string(MountProto p,MountErrorCode &e) = 0;
        virtual void set_conn_string(MountProto p,std::string const &s,MountErrorCode &e) = 0;
        virtual void set_lat_lon(double lat,double lon,MountErrorCode &e) = 0;
        virtual std::pair<double,double> get_lat_lon(MountErrorCode &e) = 0;
        virtual EqCoord get_current(MountErrorCode &e) = 0; 
        virtual std::pair<int,int> get_slew_rate(MountErrorCode &e) = 0;
        virtual void go_to(EqCoord coord,MountErrorCode &e) = 0;
        virtual void sync(EqCoord coord,MountErrorCode &e) = 0;
        virtual void slew(MountSlew direction,int speed,MountErrorCode &e) = 0;
        virtual MountMeridianFlip get_meridian(MountErrorCode &e) = 0;
        virtual void set_meridian(MountMeridianFlip flip,MountErrorCode &e) = 0;

        virtual bool pulse_guiding_supported(float &ns_guide_rate, float &we_guide_rate,MountErrorCode &e) = 0;
        virtual void pulse_guide(float NS_ms,float WE_ms,MountErrorCode &e) = 0;
        
        virtual bool connected()
        {
            guard_type g(lock_);
            return connected_;
        }

        virtual void set_update_callback(mount_callback_type cb)
        {
            guard_type g(lock_);
            callback_ = cb;
        }
        virtual void reset_update_callback()
        {
            guard_type g(lock_);
            callback_ = nullptr;
        }

    protected:
        
        virtual void check_alt_limits(int low,int high,MountErrorCode &e);
        void save_alt(MountErrorCode &e);
        void load_alt();
        std::string ols_config_file();

        int high_alt_ = 90;
        int low_alt_ = -90;
        std::recursive_mutex lock_;
        bool connected_ = false;
        double RA_ = 0,DEC_ = 0;
        mount_callback_type callback_;
        bool pending_lat_long_ = false;
        double lat_,lon_;
    };
    

    struct DriverInfo {
        DriverInfo(std::string n="",std::string o="",bool ho=false) :
            name(n),
            option(o),
            has_option(ho)
        {
        }
        std::string name;
        std::string option;
        bool has_option = false;
    };

    class MountDriver {
    public:
        virtual std::unique_ptr<Mount> get_mount(MountErrorCode &e) = 0;
        virtual void shutdown() = 0;
        virtual ~MountDriver()
        {
        }
    protected:
    };

    class MountInterface {
    public:

        typedef std::unique_lock<std::recursive_mutex> guard_type;
        MountInterface();
        virtual ~MountInterface();

        guard_type guard()
        {
            guard_type g(lock_);
            return g;
        }
        
        void config_libdir(std::string const &libdir);
        std::vector<DriverInfo> drivers_list();
        void load_driver(std::string const &d,std::string const &opt);
        void load_mount();

        void shutdown_mount();

        bool driver_is_loaded();
        bool mount_is_loaded();
        Mount *client();
        
    protected:
        std::vector<DriverInfo> drivers_;
        std::unique_ptr<MountDriver> driver_;
        std::unique_ptr<Mount> client_;
        std::string libdir_;
        std::recursive_mutex lock_;
    };
    


} // namespace ols
