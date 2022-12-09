#pragma once
#include <cppcms/application.h>
#include <cppcms/http_context.h>
#include <cppcms/http_response.h>
#include <cppcms/url_dispatcher.h>
#include <cppcms/service.h>
#include <booster/log.h>
#include <sstream>
#include <iomanip>
#include "ctl_app.h"

namespace ols {
    class CameraControlApp : public ControlAppBase {
    public:
        typedef std::unique_lock<std::recursive_mutex> guard;
        CameraControlApp(cppcms::service &srv,CameraInterface *cam) : 
            ControlAppBase(srv),
            cam_(cam)
        {
            dispatcher().map("GET","/?",&CameraControlApp::list_cameras,this);
            dispatcher().map("POST","/?",&CameraControlApp::start_stop,this);
            dispatcher().map("GET","/formats/?",&CameraControlApp::formats,this);
            dispatcher().map("POST","/stream/?",&CameraControlApp::stream,this);
        }

        void list_cameras()
        {
            guard g(cam_->lock()); 
            std::vector<std::string> cams = cam_->driver().list_cameras();
            for(size_t i=0;i<cams.size();i++) {
                response_[i]["id"] = i;
                response_[i]["name"] = cams[i];
            }
        }
        void start_stop()
        {
            std::string op = content_.get<std::string>("operation");
            if(op == "open")
                cam_->open_camera(content_.get<int>("id"));
            else if(op == "close")
                cam_->close_camera();
            else 
                fail("Invalid operation " + op);
        }
        void stream()
        {
            std::string op = content_.get<std::string>("op");
            if(op == "stop") {
                cam_->stop_stream();
            }
            else if(op == "start") {
                cppcms::json::value v;
                load_formats(v);
                std::string format_id = content_.get<std::string>("format_id");
                auto p = formats_.find(format_id);
                if(p == formats_.end()) {
                    throw CamError("No such format: " + format_id);
                }
                cam_->start_stream(p->second);
            }
            else {
                throw CamError("Invalid operation " + op);
            }
        }
        void formats()
        {
            load_formats(response_);
        }
    private:
        void load_formats(cppcms::json::value &v)
        {
            guard g(cam_->lock());
            auto fmts = cam_->cam().formats();
            formats_.clear();
            for(size_t i=0;i<fmts.size();i++) {
                auto &fmt = fmts[i];
                std::string fmt_name = stream_type_to_str(fmt.format);
                v[i]["width"] = fmt.width;
                v[i]["height"] = fmt.height;
                v[i]["format"] = fmt_name;
                v[i]["framerate"] = fmt.framerate;
                std::string name = fmt_name + ":" + std::to_string(fmt.width) + "x" + std::to_string(fmt.height);
                if(fmt.framerate > 0) {
                    std::ostringstream ss;
                    ss << "@" <<  std::fixed << std::setprecision(1) << fmt.framerate;
                    name += ss.str();
                }
                v[i]["format_id"] = name;
                formats_[name] = fmt;
            }
        }
        CameraInterface *cam_;
        std::map<std::string,CamStreamFormat> formats_;
    };
}
