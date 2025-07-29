#pragma once
#include <string>
#include <vector>
#include <memory>
#include <cppcms/json.h>

namespace httplib_for_ols {
    class Client;
}
namespace httplib = httplib_for_ols;

namespace cv { class Mat; }

namespace ols {
    class AlpacaClient {
    public:

        AlpacaClient(std::string const &base_url,std::string const &type);
        ~AlpacaClient();
        void set_logf(FILE *f);
        std::vector<std::pair<std::string,int> > list_devices();
        void set_device(int id);
        bool is_connected();
        void connect();
        template<typename T>
        T get_value(std::string const &what)
        {
            return get(what).get<T>("Value");
        }

        template<typename T>
        bool try_get_value(std::string const &what,T &value)
        {
            try {
                value = get(what).get<T>("Value");
                return true;
            }
            catch(std::exception const &) {
                return false;
            }
        }

        void get_binary_image(cv::Mat &m);
        
        cppcms::json::value get(std::string const &what);
        cppcms::json::value get(std::string const &what,std::multimap<std::string,std::string> const &params);
        void put(std::string const &where,std::multimap<std::string,std::string> const &params);
    private:
        static std::string discover(std::string const &type);
        std::multimap<std::string, std::string> std_params();
        static std::string to_lower(std::string s);

        std::string prefix_;
        int client_id_;
        int transaction_id_ = 0;
        std::string device_type_;
        int device_no_ = -1;
        cppcms::json::value get_json(std::string const &url,std::multimap<std::string,std::string> const &params = std::multimap<std::string,std::string>());
        std::unique_ptr<httplib::Client> client_;
        FILE *log_ = nullptr;
    };
}; // ols
