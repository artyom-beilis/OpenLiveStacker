#include <string>
#include <functional>
namespace ols {
    bool zip_download(std::string const &url,std::string const &target_dir,std::string &error_message,std::function<bool(char const *)> new_file_callback);
}
