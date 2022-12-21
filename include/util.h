#pragma once
#include <string>
namespace ols {
    std::string ftime(std::string const &pattern,time_t ts);
    void make_dir(std::string const &path);
    bool exists(std::string const &path);
}
