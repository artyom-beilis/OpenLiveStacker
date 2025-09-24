#include "focuser.h"
#include "alpaca_focuser.h"
#include "alpaca_client.h"

namespace ols {
    class AlpacaFocuser: public Focuser {
    public:
        AlpacaFocuser(std::string const &conn):
            client_(conn,"focuser"),
            log_(create_log_file())
        {
            try {
                client_.set_logf(log_);
                auto devices = client_.list_devices();
                if(devices.empty()) {
                    throw std::runtime_error("No focuser found");
                }
                client_.set_device(devices[0].second);
                client_.connect();
                if(!client_.is_connected())
                    throw std::runtime_error("Failed to connect to device");
            }
            catch(...) {
                close_log();
                throw;
            }
        }
        void close_log()
        {
            if(log_ != stderr && log_ != nullptr)
                fclose(log_);
            log_ = nullptr;
        }
        virtual ~AlpacaFocuser()
        {
        }
        
        static FILE *create_log_file()
        {
            char name[512];
            char prev_name[512];
            char const *fname = "debug/alpaca_focuser_log.txt";
            char const *prev_fname = "debug/alpaca_focuser_log.1.txt";
            #ifdef ANDROID_SUPPORT
            char const *home_dir = getenv("HOME");
            #else
            char const *home_dir = getenv("OLS_DATA_DIR");
            #endif    
            if(!home_dir) {
                fprintf(stderr,"Internal error OLS_DATA_DIR/HOME are not defined\n");
                return stderr;
            }
            snprintf(name,sizeof(name),"%s/%s",home_dir,fname);
            snprintf(prev_name,sizeof(prev_name),"%s/%s",home_dir,prev_fname);
            rename(name,prev_name);
            FILE *log = fopen(name,"w");
            if(!log) {
                fprintf(stderr,"Failed to open file [%s]\n",name);
                log = stderr;
            }
            else {
                setbuf(log,nullptr);
            }
            return log;
        }

        virtual bool is_absolute(FocuserErrorCode &e) override
        {
            try {
                if(abs_ready_)
                    return is_abs_;
                is_abs_ = client_.get_value<bool>("/absolute");
                abs_ready_ = true;
                return is_abs_;
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
            }
            return false;
        }
        virtual void abort(FocuserErrorCode &e) override
        {
            try {
                client_.put("/halt",{});
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
            }
        }
        virtual bool is_moving(FocuserErrorCode &e) override
        {
            try {
                return client_.get_value<bool>("/ismoving");
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
            }
            return false;
        }

        virtual int max_range(FocuserErrorCode &e) override
        {
            try {
                if(max_step_ready_) {
                    return max_step_;
                }
                max_step_ =  client_.get_value<int>("/maxstep");
                max_step_ready_ = true;
                return max_step_;
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
            }
            return 0;
        }
        void logex(std::exception const &ex)
        {
            if(log_) {
                fprintf(log_,"Exception %s\n",ex.what());
            }
        }

    protected:
        virtual int position_impl(FocuserErrorCode &e)  override
        {
            try {
                return client_.get_value<int>("/position");
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
            }
            return 0;
        }
        void move_impl(int n,FocuserErrorCode &e)
        {
            try {
                return client_.put("/move",{{"Position",std::to_string(n)}});
            }
            catch(std::exception const &ex) {
                logex(ex);
                e = ex;
            }
        }
        virtual void move_to_impl(int n,FocuserErrorCode &e) override
        {
            move_impl(n,e);
        }
        virtual void move_by_impl(int n,FocuserErrorCode &e) override
        {
            move_impl(n,e);
        }
        AlpacaClient client_;
        FILE *log_;
        bool abs_ready_ = false;
        bool is_abs_ = true;
        bool max_step_ready_ = false;
        int max_step_ = 0;
    };



    std::unique_ptr<Focuser> load_alpaca_focuser(std::string const &opt,FocuserErrorCode &e)
    {
        std::unique_ptr<Focuser> r;
        try {
            r.reset(new AlpacaFocuser(opt));
        }
        catch(std::exception const &er) {
            e = er;
        }
        return r;
    }
}
