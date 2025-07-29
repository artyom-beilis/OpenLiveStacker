#pragma once

#if defined(_WIN32)
#include <chrono>
#include <thread>
#include <time.h>

namespace ols_util {
    struct timeval {
        time_t tv_sec;
        int tv_usec;
    };
    inline int gettimeofday(struct timeval* tp, struct timezone*) {
        namespace sc = std::chrono;
        std::chrono::system_clock::duration d = sc::system_clock::now().time_since_epoch();
        sc::seconds s = sc::duration_cast<sc::seconds>(d);
        tp->tv_sec = s.count();
        tp->tv_usec = sc::duration_cast<sc::microseconds>(d - s).count();
        return 0;
    }
    inline void usleep(int us)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(us));
    }
} // ols_util

using ols_util::timeval;
using ols_util::gettimeofday;
using ols_util::usleep;

#else

#include <sys/time.h>
#include <unistd.h>

#endif // _WIN32
