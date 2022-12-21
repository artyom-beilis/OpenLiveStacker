#include "util.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <system_error>

namespace ols {
    void make_dir(std::string const &path)
    {
        int r = mkdir(path.c_str(),0777);
        if(r < 0) {
            int e=errno;
            if(e == EEXIST)
                return;
            throw std::system_error(e,std::generic_category(),"mkdir failed");
        }
    }
    bool exists(std::string const &path)
    {
        return access(path.c_str(),F_OK) == 0;
    }
    std::string ftime(std::string const &pattern,time_t ts)
    {
        char buf[256];
        strftime(buf,sizeof(buf),pattern.c_str(),localtime(&ts));
        return buf;
    }
}
