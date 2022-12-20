#pragma once
#include "data_items.h"
#include "ctl_app.h"

#include <cppcms/application.h>
#include <cppcms/http_context.h>
#include <cppcms/http_response.h>
#include <cppcms/url_dispatcher.h>
#include <cppcms/service.h>
#include <booster/log.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace ols {
    class StackerControlApp : public ControlAppBase {
    public:
        StackerControlApp(cppcms::service &srv,
                          std::string data_dir,
                          queue_pointer_type queue): 
            ControlAppBase(srv),
            data_dir_(data_dir),
            queue_(queue)
        {
            stacked_path_ = data_dir_ + "/stacked";
            mkdir(stacked_path_.c_str(),0777);
            calibration_path_ = data_dir_ + "/calibration";
            mkdir(calibration_path_.c_str(),0777);
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
            cmd->calibration = content_.get("type","dso") == "calibration";
            cmd->name = content_.get<std::string>("name");
            cmd->save_inputs = content_.get("save_data",false);
            if(!cmd->calibration) {
                char ts[256];
                time_t now=time(nullptr);
                strftime(ts,sizeof(ts),"_%Y%m%d_%H%M%S",localtime(&now));
                cmd->output_path = stacked_path_ + "/" + cmd->name + ts;
            }
            else {
                cmd->output_path = calibration_path_;
            }
            cmd->source_gamma = content_.get("source_gamma",cmd->source_gamma);
            cmd->lat = content_.get("target.lat",cmd->lat);
            cmd->lon = content_.get("target.lon",cmd->lon);
            cmd->ra = content_.get("location.ra",cmd->ra);
            cmd->de = content_.get("location.de",cmd->de);
            cmd->derotate_mirror = content_.get("image_flip",cmd->derotate_mirror);
            cmd->derotate = content_.get("field_rotation",cmd->derotate);
            cmd->darks_path = content_.get("darks",cmd->darks_path);
            if(!cmd->darks_path.empty())
                cmd->darks_path = calibration_path_ + "/" + cmd->darks_path + ".tiff";
            status_ = "stacking";
            queue_->push(cmd);
        }
    private:
        std::string data_dir_;
        std::string stacked_path_;
        std::string calibration_path_;
        queue_pointer_type queue_;
        std::string status_ = "idle";
    };
};

