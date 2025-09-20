#pragma once
#include <stdexcept>
#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <memory>
#include <sstream>

namespace ols {
    class FocuserError : public std::runtime_error {
    public:
        FocuserError(std::string const &msg) : std::runtime_error(msg)
        {
        }
    };

    class FocuserErrorCode {
    public:
        FocuserErrorCode() : status_(false), message_("ok") {}
        FocuserErrorCode(std::string const &msg) :status_(true), message_(msg) {}
        FocuserErrorCode(std::exception const &e) : status_(true), message_(e.what())
        {
        }
        FocuserErrorCode &operator=(std::string const &msg)
        {
            status_ = true;
            message_ = msg;
            return *this;
        }
        FocuserErrorCode(FocuserErrorCode const &) = default;
        FocuserErrorCode &operator=(FocuserErrorCode const &) = default;
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
                throw FocuserError(message_);
        }
    private:
        bool status_;
        std::string message_;
    };
    
    class Focuser {
    public:

        int position(FocuserErrorCode &e)
        {
            bool is_abs = is_absolute(e);
            if(e)
                return 0;
            if(is_abs){
                return accum_position_;
            }
            else {
                return position_impl(e);
            }

        }

        void move_by(int delta,FocuserErrorCode &e)
        {
            bool is_abs = is_absolute(e);
            if(e)
                return;
            if(is_abs) {
                int pos = position(e);
                if(e)
                    return;
                move_to_impl(pos + delta,e);
            }
            else {
                move_by_impl(delta,e);
                if(!e)
                    accum_position_ += delta;
            }
        }
        void move_to(int newpos,FocuserErrorCode &e)
        {
            bool is_abs = is_absolute(e);
            if(e)
                return;
            if(is_abs) {
                move_to_impl(newpos,e);
            }
            else {
                move_by_impl(newpos - accum_position_,e);
                if(!e)
                    accum_position_ = newpos;
            }
        }

        virtual ~Focuser() 
        {
        }

        virtual bool is_absolute(FocuserErrorCode &e) = 0;
        virtual void abort(FocuserErrorCode &e) = 0;
        virtual bool is_moving(FocuserErrorCode &e) = 0;
        virtual int max_range(FocuserErrorCode &e) = 0;

    protected:
        virtual int position_impl(FocuserErrorCode &e) = 0;
        virtual void move_to_impl(int n,FocuserErrorCode &e) = 0;
        virtual void move_by_impl(int n,FocuserErrorCode &e) = 0;
        int accum_position_ = 0;
    };
   
    struct FocuserInfo {
        std::string name;
        std::string opt;
    };

    class FocuserInterface {
    public:

        typedef std::unique_lock<std::recursive_mutex> guard_type;

        static FocuserInterface &instance();
        static Focuser *focuser();

        guard_type guard()
        {
            guard_type g(lock_);
            return g;
        }
        
        std::vector<FocuserInfo> drivers_list();
        void load_driver(FocuserInfo const &info,FocuserErrorCode &e);
        
    protected:
        FocuserInterface();
        virtual ~FocuserInterface();

        std::vector<FocuserInfo> drivers_;
        std::unique_ptr<Focuser> focuser_;
        std::recursive_mutex lock_;
    };

}//
