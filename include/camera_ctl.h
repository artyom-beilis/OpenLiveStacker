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
        CameraControlApp(cppcms::service &srv,CameraInterface *cam,queue_pointer_type video_gen_queue) : 
            ControlAppBase(srv),
            cam_(cam),
            generator_queue_(video_gen_queue)
        {
            dispatcher().map("GET","/?",&CameraControlApp::list_cameras,this);
            dispatcher().map("POST","/?",&CameraControlApp::start_stop,this);
            dispatcher().map("GET","/formats/?",&CameraControlApp::formats,this);
            dispatcher().map("GET","/status/?",&CameraControlApp::status,this);
            dispatcher().map("POST","/stream/?",&CameraControlApp::stream,this);
            dispatcher().map("GET","/options/?",&CameraControlApp::options,this);
            dispatcher().map("POST","/options/?",&CameraControlApp::set_options,this);
            dispatcher().map("POST","/option/(\\w+)",&CameraControlApp::set_opt,this,1);
            dispatcher().map("GET","/option/(\\w+)",&CameraControlApp::get_opt,this,1);
            external_set_defaults();
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
                    response_["bin"]=format.bin;
                }
                break;
            }
        }

        void get_opt(std::string id)
        {
            CamOptionId opt = cam_option_id_from_string_id(id);
            double val;
            {
                guard g(cam_->lock());
                CamErrorCode e;
                auto res = is_external_option(opt) ? external_option_get(opt,e) : cam_->cam().get_parameter(opt,true,e);
                e.check();
                val = res.cur_val;
            }
            response_["value"] = val;
        }
        void set_opt(std::string id)
        {
            double value = content_.get<double>("value");
            CamOptionId opt = cam_option_id_from_string_id(id);
            guard g(cam_->lock());
            CamErrorCode e;
            if(is_external_option(opt))
                external_option_set(opt,value,e);
            else
                cam_->cam().set_parameter(opt,value,e);
            e.check();
        }
        void set_options()
        {
            cppcms::json::array const &opts = content_.array();
            for(cppcms::json::value const &opt: opts) {
                double value = opt.get<double>("value");
                std::string opt_id = opt.get<std::string>("id");
                CamOptionId cam_opt = cam_option_id_from_string_id(opt_id);
                guard g(cam_->lock());
                CamErrorCode e;
                if(is_external_option(cam_opt))
                    external_option_set(cam_opt,value,e);
                else
                    cam_->cam().set_parameter(cam_opt,value,e);
                e.check();
            }
        }
        void external_options(std::vector<CamOptionId> &params)
        {
            params.insert(params.begin(),opt_live_stretch);
        }
        CamParam external_option_get(CamOptionId opt,CamErrorCode &e)
        {
            CamParam r;
            switch(opt) {
            case opt_live_stretch:
                r.option = opt;
                r.type = type_bool;
                r.step_size = 1;
                r.min_val = 0;
                r.max_val = 1;
                r.step_size = 1;
                r.def_val = 0;
                r.cur_val = live_stretch_;
                break;
            default:
               e=CamErrorCode("Internal error - non-extra option"); 
            }
            return r;
        }
        void external_option_set(CamOptionId opt,double value,CamErrorCode &e)
        {
            switch(opt) {
            case opt_live_stretch:
                if(value !=0 && value!=1) {
                    e=CamErrorCode("Invalid option value"); 
                    return;
                }
                live_stretch_ = int(value);
                break;
            default:
               e=CamErrorCode("Internal error - non-extra option"); 
               return;
            }
            generator_queue_->push(data_pointer_type(new LiveControl(opt,value)));
        }
        void external_set_defaults()
        {
            if(defaults_configured_++ != 0)
                return;
            CamOptionId opts[] = { opt_live_stretch };
            CamErrorCode e;
            for(CamOptionId opt : opts) {
                auto res = external_option_get(opt,e);
                e.check();
                external_option_set(opt,res.def_val,e);
                e.check();
            }
        }
        void options()
        {
            std::vector<CamParam> params;
            {
                guard g(cam_->lock());
                auto &cam = cam_->cam();
                CamErrorCode e;
                auto opts = cam.supported_options(e);
                external_options(opts);
                e.check();
                for(auto opt:opts) {
                    CamParam param = is_external_option(opt) ? external_option_get(opt,e) : cam.get_parameter(opt,e);
                    e.check();
                    params.push_back(param);
                }
            }
            for(unsigned i=0;i<params.size();i++) {
                CamParam param = params[i];
                response_[i]["option_id"] = cam_option_id_to_string_id(param.option);
                response_[i]["read_only"] = param.read_only;
                response_[i]["name"] = cam_option_id_to_name(param.option);
                response_[i]["type"] = cam_option_type_to_str(param.type);
                response_[i]["step"] = param.step_size;
                response_[i]["cur"] = param.cur_val;
                response_[i]["min"] = param.min_val;
                response_[i]["max"] = param.max_val;
                response_[i]["default"] = param.def_val;
                if(param.type == type_selection) {
                    response_[i]["options"] = param.names;
                }
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
            if(op == "open") {
                int cam_id = content_.get<int>("id",-1);
                if(cam_id) {
                    throw std::runtime_error("No camera was selected");
                }
                cam_->open_camera(cam_id);
            }
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
                double max_framerate = content_.get<double>("max_framerate");
                auto p = formats_.find(format_id);
                if(p == formats_.end()) {
                    throw CamError("No such format: " + format_id);
                }
                cam_->start_stream(p->second,max_framerate);
                response_["status"]="ok";
                response_["width"]=p->second.width;
                response_["height"]=p->second.height;
                response_["bin"]=p->second.bin;
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
                v[i]["bin"] = fmt.bin;
                v[i]["format"] = fmt_name;
                v[i]["framerate"] = fmt.framerate;
                std::string name = fmt_name + ":" + std::to_string(fmt.width) + "x" + std::to_string(fmt.height);
                if(fmt.bin>1) {
                    name += ":bin" + std::to_string(fmt.bin);
                }
                if(fmt.roi_num > 1 || fmt.roi_den > 1) {
                    if(fmt.bin == 1)
                        name += ":";
                    else
                        name += ",";
                    name += "roi=" + std::to_string(fmt.roi_num) + "/" + std::to_string(fmt.roi_den);
                }
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
        queue_pointer_type generator_queue_;
        std::map<std::string,CamStreamFormat> formats_;
        static std::atomic<int> live_stretch_;
        static std::atomic<int> defaults_configured_;
    };
}
