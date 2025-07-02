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
#include <booster/regex.h>

#include "server_sent_events.h"
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
            dispatcher().map("POST","/stretch/?",&StackerControlApp::stretch,this);
            dispatcher().map("POST","/sharpen/?",&StackerControlApp::sharpen,this);
            dispatcher().map("GET", "/status/?",&StackerControlApp::status,this);
            dispatcher().map("DELETE", "/calibration/([A-Za-z0-9_.\\-]*)",&StackerControlApp::del_calibration,this,1);
        }
        void del_calibration(std::string id)
        {
            std::string db_path = calibration_path_ + "/index.json";
            cppcms::json::value db;
            {
                std::ifstream js(db_path);
                if(!js) 
                    throw std::runtime_error("Failed to open " + db_path);
                if(!db.load(js,true)) 
                    throw std::runtime_error("Failed to parse " + db_path);
            }
            cppcms::json::array &items = db.array();
            for(size_t i=0;i<items.size();i++) {
                if(items[i].get<std::string>("id") == id) {
                    std::string tiff = calibration_path_ + "/" + items[i].get<std::string>("path");
                    BOOSTER_INFO("stacker") << "Removing " << tiff; 
                    std::remove(tiff.c_str());
                    items.erase(items.begin() + i);
                    break;
                }
            }
            {
                std::ofstream js(db_path);
                db.save(js,cppcms::json::readable);
                js.close();
            }

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
            }
            else if(op == "resume") {
                cmd->op = StackerControl::ctl_resume;
                status_ = "stacking";
            }
            else if(op == "cancel"){
                cmd->op = StackerControl::ctl_cancel;
                status_ = "idle";
            }
            else
                throw std::runtime_error("Unknown operation " + op);
            queue_->push(cmd);
        }
        void sharpen()
        {
            std::shared_ptr<StackerControl> cmd(new StackerControl());
            cmd->op = StackerControl::ctl_update;
            cmd->deconv_sig = content_.get("deconv_sig",cmd->deconv_sig);
            cmd->deconv_iters = content_.get("deconv_iters",cmd->deconv_iters);
            cmd->unsharp_sig = content_.get("unsharp_sig",cmd->unsharp_sig);
            cmd->unsharp_strength = content_.get("unsharp_strength",cmd->unsharp_strength);
            queue_->push(cmd);
        }
        void stretch()
        {
            std::shared_ptr<StackerControl> cmd(new StackerControl());
            cmd->op = StackerControl::ctl_update;
            cmd->auto_stretch = content_.get("auto_stretch",cmd->auto_stretch);
            cmd->stretch_low = content_.get("stretch_low",cmd->stretch_low);
            cmd->stretch_high = content_.get("stretch_high",cmd->stretch_high);
            cmd->stretch_gamma = content_.get("stretch_gamma",cmd->stretch_gamma);
            queue_->push(cmd);
        }
        void validate_name(std::string const &name)
        {
            if(!booster::regex_match(name,valid_name_))
                throw std::runtime_error("Invalid object name `"+name + "'");
        }
        void start()
        {
            std::shared_ptr<StackerControl> cmd(new StackerControl());
            cmd->op = StackerControl::ctl_init;
            auto format = cam_->stream_format();
            cmd->mono = is_mono_stream(format.format);
            cmd->format = stream_type_to_str(cam_->stream_format().format);
            cmd->bin = cam_->stream_format().bin;
            cmd->width = format.width;
            cmd->height = format.height;
            std::string method = content_.get("type","dso");
            if(method == "dso")
                cmd->method = stack_dso;
            else if(method == "calibration")
                cmd->method = stack_calibration;
            else if(method == "planetary")
                cmd->method = stack_planetary;
            else if(method == "dynamic")
                cmd->method = stack_dynamic;
            else
                throw std::runtime_error("Unsupported stacking method:" + method);
            cmd->name = content_.get<std::string>("name");
            validate_name(cmd->name);
            cmd->save_inputs = content_.get("save_data",false);
            if(cmd->method != stack_calibration) {
                if(!cmd->name.empty())
                    cmd->name += "_";
                cmd->name += ftime("%Y%m%d_%H%M%S",time(nullptr));
                cmd->output_path = stacked_path_ + "/" + cmd->name; 
            }
            else {
                cmd->output_path = calibration_path_;
            }
            cmd->source_gamma = 1.0;
            {
                CamErrorCode e;
                std::unique_lock<std::recursive_mutex> guard(cam_->lock());
                auto opts = cam_->cam().get_all_parameters(true,e);
                e.check();
                for(auto param:opts) {
                    cmd->camera_config[param.option] = param.cur_val;
                    if(param.option == opt_gamma)
                        cmd->source_gamma = param.cur_val;
                }
            }
                
            cmd->lat = content_.get("location.lat",cmd->lat);
            cmd->lon = content_.get("location.lon",cmd->lon);
            cmd->ra = content_.get("target.ra",cmd->ra);
            cmd->de = content_.get("target.de",cmd->de);
            cmd->deconv_sig = content_.get("deconv_sig",cmd->deconv_sig);
            cmd->deconv_iters = content_.get("deconv_iters",cmd->deconv_iters);
            cmd->unsharp_sig = content_.get("unsharp_sig",cmd->unsharp_sig);
            cmd->unsharp_strength = content_.get("unsharp_strength",cmd->unsharp_strength);
            
            cmd->synthetic_exposure_mpl = content_.get("synthetic_exposure_mpl",cmd->synthetic_exposure_mpl);
            cmd->remove_gradient = content_.get("remove_gradient",cmd->remove_gradient);
            cmd->remove_hot_pixels = content_.get("remove_hot_pixels",cmd->remove_hot_pixels);
            cmd->derotate_mirror = content_.get("image_flip",cmd->derotate_mirror);
            cmd->derotate = content_.get("field_derotation",cmd->derotate);
            cmd->rollback_on_pause = content_.get("rollback_on_pause",cmd->rollback_on_pause);
            cmd->darks_path = content_.get("darks",cmd->darks_path);
            cmd->flats_path = content_.get("flats",cmd->flats_path);
            cmd->dark_flats_path = content_.get("dark_flats",cmd->dark_flats_path);
            cmd->auto_stretch = content_.get("auto_stretch",cmd->auto_stretch);
            cmd->stretch_low = content_.get("stretch_low",cmd->stretch_low);
            cmd->stretch_high = content_.get("stretch_high",cmd->stretch_high);
            cmd->stretch_gamma = content_.get("stretch_gamma",cmd->stretch_gamma);
            cmd->remove_satellites = content_.get("remove_satellites",cmd->remove_satellites);
            cmd->save_after = content_.get("save_after",cmd->save_after);
            cmd->save_tiff = content_.get("save_tiff",cmd->save_tiff);

            cmd->filters.remove_first=content_.get("filters.remove_first",cmd->filters.remove_first);
            cmd->filters.min_stat_size=content_.get("filters.min_stat_size",cmd->filters.min_stat_size);
            cmd->filters.dynamic_delay=content_.get("filters.dynamic_delay",cmd->filters.dynamic_delay);
            cmd->filters.sharpness_percentile=content_.get("filters.sharpness_percentile",cmd->filters.sharpness_percentile);
            cmd->filters.reg_score_percentile=content_.get("filters.reg_score_percentile",cmd->filters.reg_score_percentile);
            cmd->filters.avg_brightness_sigma=content_.get("filters.avg_brightness_sigma",cmd->filters.avg_brightness_sigma);


            cmd->dither_radius = content_.get("dither_radius",cmd->dither_radius);
            cmd->dither_frequency = content_.get("dither_frequency",cmd->dither_frequency);
            cmd->dither_delay = content_.get("dither_delay",cmd->dither_delay);
            cmd->guide_rate_ns = content_.get("guide_rate_ns",cmd->guide_rate_ns);
            cmd->guide_rate_we = content_.get("guide_rate_we",cmd->guide_rate_we);


            if(!cmd->darks_path.empty())
                cmd->darks_path = calibration_path_ + "/" + cmd->darks_path + ".tiff";

            if(!cmd->flats_path.empty())
                cmd->flats_path = calibration_path_ + "/" + cmd->flats_path + ".tiff";

            if(!cmd->dark_flats_path.empty())
                cmd->dark_flats_path = calibration_path_ + "/" + cmd->dark_flats_path + ".tiff";

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
        booster::regex valid_name_ = booster::regex("[A-Za-z0-9_.\\-]*");
    };

    class StackerStatsNotification: public cppcms::application {
    public:
        StackerStatsNotification(cppcms::service &srv):
            cppcms::application(srv)
        {
            stream_ = sse::bounded_event_queue::create(srv.get_io_service(),16);
        }
        void main(std::string /*url*/)
        {
            stream_->accept(release_context());
        }
        std::function<void(data_pointer_type)> get_callback()
        {
            booster::intrusive_ptr<StackerStatsNotification> self = this;
            return [=](data_pointer_type p) {
                self->stats_handler(p);
            };
        }
        // thread safe method
        void stats_handler(data_pointer_type stats)
        {
            service().post([=](){
                update_stats(stats);
            });
        }
        void update_stats(data_pointer_type p)
        {
            std::shared_ptr<StatsData> data = std::dynamic_pointer_cast<StatsData>(p);
            std::shared_ptr<ErrorNotificationData> error = std::dynamic_pointer_cast<ErrorNotificationData>(p);
            std::shared_ptr<MountPositionNotification> mount = std::dynamic_pointer_cast<MountPositionNotification>(p);
            std::ostringstream ss;
            cppcms::json::value info;
            if(data) {
                info["type"] = "stats";
                info["stacked"] = data->stacked;
                info["missed"] = data->missed;
                info["dropped"] = data->dropped;
                info["since_saved_s" ] = data->since_saved_s;
                info["histogramm"] = data->histogramm;
            }
            else if(error) {
                info["type"] = "error";
                info["message"] = error->message;
                info["source"] = error->source;
            }
            else if(mount) {
                info["type"] = "mount";
                info["ra"]   = mount->RA;
                info["dec"]  = mount->DEC;
                info["alt"]  = mount->Alt;
                info["az"]   = mount->Az;
            }
            else
                return;
            ss << info;
            stream_->enqueue(ss.str());
        }
    private:
        std::shared_ptr<sse::bounded_event_queue> stream_;
    };
};

