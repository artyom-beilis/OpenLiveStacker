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
            dispatcher().map("GET","/status/?",&CameraControlApp::status,this);
            dispatcher().map("POST","/stream/?",&CameraControlApp::stream,this);
            dispatcher().map("GET","/options/?",&CameraControlApp::options,this);
            dispatcher().map("POST","/option/(\\w+)",&CameraControlApp::set_opt,this,1);
        }

        void status()
        {
            switch(cam_->status()) {
            case CameraInterface::cam_closed:
                response_["status"]="closed";
                break;
            case CameraInterface::cam_open:
                response_["status"]="open";
                break;
            case CameraInterface::cam_streaming:
                response_["status"]="streaming";
                {
                    CamStreamFormat format = cam_->stream_format();
                    response_["width"]=format.width;
                    response_["height"]=format.height;
                }
                break;
            }
        }

        void set_opt(std::string id)
        {
            double value = content_.get<double>("value");
            CamOptionId opt = cam_option_id_from_string_id(id);
            guard g(cam_->lock());
            CamErrorCode e;
            cam_->cam().set_parameter(opt,value,e);
            e.check();
        }
        void options()
        {
            std::vector<CamParam> params;
            {
                guard g(cam_->lock());
                auto &cam = cam_->cam();
                CamErrorCode e;
                auto opts = cam.supported_options(e);
                e.check();
                for(auto opt:opts) {
                    CamParam param = cam.get_parameter(opt,e);
                    e.check();
                    params.push_back(param);
                }
            }
            for(unsigned i=0;i<params.size();i++) {
                CamParam param = params[i];
                response_[i]["option_id"] = cam_option_id_to_string_id(param.option);
                response_[i]["name"] = cam_option_id_to_name(param.option);
                response_[i]["type"] = cam_option_type_to_str(param.type);
                response_[i]["step"] = param.step_size;
                response_[i]["cur"] = param.cur_val;
                response_[i]["min"] = param.min_val;
                response_[i]["max"] = param.max_val;
                response_[i]["default"] = param.def_val;
            }
        }

        void list_cameras()
        {
            guard g(cam_->lock()); 
            CamErrorCode e;
            std::vector<std::string> cams = cam_->driver().list_cameras(e);
            e.check();
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
                response_["status"]="ok";
                response_["width"]=p->second.width;
                response_["height"]=p->second.height;
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
            CamErrorCode e;
            auto fmts = cam_->cam().formats(e);
            e.check();
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
