#pragma once
#include <cppcms/application.h>
#include <cppcms/http_context.h>
#include <cppcms/http_response.h>
#include <cppcms/http_request.h>
#include <cppcms/json.h>

namespace ols {
    class ControlAppBase : public cppcms::application {
    public:
        using cppcms::application::application;
        void main(std::string url)
        {
            content_  = cppcms::json::value();
            response_ = cppcms::json::value();
            response().set_content_header("application/json");
            try {
                if(request().request_method() == "POST") {
                    std::pair<void *,size_t> post_data = request().raw_post_data();
                    std::istringstream ss(std::string(reinterpret_cast<char const *>(post_data.first),post_data.second));
                    cppcms::json::value request;
                    if(!request.load(ss,true)) {
                        throw CamError("Invalid JSON");
                    }
                    content_.swap(request);
                    std::cerr << "GOT:" << url << std::endl;
                    content_.save(std::cerr,cppcms::json::readable);
                }

                cppcms::application::main(url);

                if(response_.is_undefined()) {
                    response_["status"] = "ok";
                }
                response_.save(response().out(),cppcms::json::readable);
                std::cerr << "RESP:" << url << std::endl;
                response_.save(std::cerr,cppcms::json::readable);

            }
            catch(std::exception const &e) {
                cppcms::json::value err;
                err["status"]="fail";
                err["error"] = e.what();
                response().out() << err;
            }
        }
        void fail(std::string const &msg)
        {
            cppcms::json::value err;
            err["status"]="fail";
            err["error"] = msg;
            response_.swap(err);
        }
    protected:
        cppcms::json::value content_;
        cppcms::json::value response_;

    };
}
