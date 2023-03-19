#pragma once
#include <string>
#include <memory>
namespace ols {
    std::string ftime(std::string const &pattern,time_t ts);
    void make_dir(std::string const &path);
    bool exists(std::string const &path);

    class DirWatch {
    public:
        DirWatch(std::string const &path); 
        ~DirWatch();
        DirWatch(DirWatch const &) = delete;
        DirWatch &operator=(DirWatch const &) = delete;
        
        std::string wait_for_new_file(int time_ms);
    private:
        bool has_enough();
        int fd_;
        std::string dir_;
        struct data;
        std::unique_ptr<data> d;

    };
}
