#pragma once
#include "data_items.h"
#include "ctl_app.h"
#include "camera_iface.h"

#include <cppcms/application.h>
#include <cppcms/http_context.h>
#include <cppcms/http_response.h>
#include <cppcms/url_dispatcher.h>
#include <cppcms/service.h>
#include <booster/log.h>
#include "util.h"
namespace ols {
    class StackerControlApp : public ControlAppBase {
    public:
        StackerControlApp(cppcms::service &srv,
                          CameraInterface *iface,
                          std::string data_dir,
                          queue_pointer_type queue): 
            ControlAppBase(srv),
            cam_(iface),
            data_dir_(data_dir),
            queue_(queue)
        {
            stacked_path_ = data_dir_ + "/stacked";
            calibration_path_ = data_dir_ + "/calibration";
            dispatcher().map("POST","/start/?",&StackerControlApp::start,this);
            dispatcher().map("POST","/control/?",&StackerControlApp::control,this);
            dispatcher().map("GET", "/status/?",&StackerControlApp::status,this);
        }
        void status()
        {
            response_["status"] = status_;
        }
        void control()
        {
            std::shared_ptr<StackerControl> cmd(new StackerControl());
            std::string op = content_.get<std::string>("operation");
            if(op == "pause") {
                cmd->op = StackerControl::ctl_pause;
                status_ = "paused";
            }
            else if(op == "save") {
                cmd->op = StackerControl::ctl_save;
                status_ = "idle";
            }
            else if(op == "resume") {
                cmd->op = StackerControl::ctl_resume;
                status_ = "stacking";
            }
            else if(op == "save_and_continue"){
                cmd->op = StackerControl::ctl_save_and_continue;
            }
            else
                throw std::runtime_error("Unknown operation " + op);
            queue_->push(cmd);
        }
        void start()
        {
            std::shared_ptr<StackerControl> cmd(new StackerControl());
            cmd->op = StackerControl::ctl_init;
            auto format = cam_->stream_format();
            cmd->width = format.width;
            cmd->height = format.height;
            cmd->calibration = content_.get("type","dso") == "calibration";
            cmd->name = content_.get<std::string>("name");
            cmd->save_inputs = content_.get("save_data",false);
            if(!cmd->calibration) {
                if(!cmd->name.empty())
                    cmd->name += "_";
                cmd->name += ftime("%Y%m%d_%H%M%S",time(nullptr));
                cmd->output_path = stacked_path_ + "/" + cmd->name; 
            }
            else {
                cmd->output_path = calibration_path_;
            }
            try {
                if(format.format == stream_mjpeg || format.format == stream_yuv2) { 
                    std::unique_lock<std::recursive_mutex> guard(cam_->lock());
                    cmd->source_gamma = cam_->cam().get_parameter(opt_gamma,true).cur_val;
                }
                else
                    cmd->source_gamma = 1.0;
            }
            catch(CamError const &) {
                cmd->source_gamma = 1.0;
            }
            cmd->lat = content_.get("location.lat",cmd->lat);
            cmd->lon = content_.get("location.lon",cmd->lon);
            cmd->ra = content_.get("target.ra",cmd->ra);
            cmd->de = content_.get("target.de",cmd->de);
            cmd->derotate_mirror = content_.get("image_flip",cmd->derotate_mirror);
            cmd->derotate = content_.get("field_derotation",cmd->derotate);
            cmd->darks_path = content_.get("darks",cmd->darks_path);
            cmd->auto_stretch = content_.get("auto_stretch",cmd->auto_stretch);
            if(!cmd->darks_path.empty())
                cmd->darks_path = calibration_path_ + "/" + cmd->darks_path + ".tiff";
            status_ = "stacking";
            queue_->push(cmd);
        }
    private:
        CameraInterface *cam_;
        std::string data_dir_;
        std::string stacked_path_;
        std::string calibration_path_;
        queue_pointer_type queue_;
        std::string status_ = "idle";
    };
};

