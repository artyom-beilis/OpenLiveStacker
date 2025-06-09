#include <stdio.h>
#include <string>

namespace ols {
    struct TempFileGuard {
        std::string fname;
        bool status;
        char const *error_message = "";
        TempFileGuard(void *ptr,size_t len)
        {
            status = false;
            #ifdef ANDROID_SUPPORT
            char const *home_dir = getenv("HOME");
            #else
            char const *home_dir = getenv("OLS_DATA_DIR");
            #endif    
            if(!home_dir) {
                home_dir = "./";
            }
            fname = home_dir + std::string("/ols_tmp.fits");
            FILE *f = fopen(fname.c_str(),"wb");
            if(!f) {
                error_message = "Failed to open file";
                return;
            }
            if(fwrite(ptr,1,len,f) != len) {
                error_message = "Failed to write to file";
                fclose(f);
                return;
            }
            if(fclose(f)!=0) {
                error_message = "Failed to close file";
                f=nullptr;
                return;
            }
            f=nullptr;
            status = true;
        }
        operator bool() const
        {
            return status;
        }
        ~TempFileGuard()
        {
            std::remove(fname.c_str());
        }
    };
} // namespace

