#pragma once
#include "ctl_app.h"

#include <cppcms/application.h>
#include <cppcms/http_context.h>
#include <cppcms/http_response.h>
#include <cppcms/url_dispatcher.h>
#include <cppcms/service.h>
#include <booster/log.h>
#include <mutex>

namespace ols {
    class ConfigApp : public ControlAppBase {
    public:
        ConfigApp(cppcms::service &srv,
                  std::string data_dir):
            ControlAppBase(srv)
        {
            config_path_ = data_dir + "/config.json";
            dispatcher().map("POST","/(\\w+)",&ConfigApp::set,this,1);
            dispatcher().map("GET", "/(\\w+)",&ConfigApp::get,this,1);
        }
        void get(std::string id)
        {
            cppcms::json::value v=config();
            auto &kv = v.object();
            if(kv.find(id) == kv.end()) {
                response_["value"] = cppcms::json::null();
            }
            else {
                response_["value"] = kv[id];
            }
        }
        void set(std::string id)
        {
            std::unique_lock<std::recursive_mutex> g(lock_);
            cppcms::json::value v = config();
            v[id] = content_["value"];
            std::ofstream f(config_path_);
            v.save(f,cppcms::json::readable);
        }

        cppcms::json::value config()
        {
            std::unique_lock<std::recursive_mutex> g(lock_);
            std::ifstream f(config_path_);
            if(!f) {
                return cppcms::json::object();
            }
            cppcms::json::value v;
            if(!v.load(f,true) || v.type() != cppcms::json::is_object) {
                return cppcms::json::object();
            }
            return v;
        }
    private:
        static std::recursive_mutex lock_;
        std::string config_path_;
    };
};

