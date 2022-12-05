#include "ols.h"
#include <algorithm>
#include <cppcms/json.h>
#include <cppcms/applications_pool.h>
#include <cppcms/mount_point.h>

namespace ols {
OpenLiveStacker::OpenLiveStacker()
{
}

OpenLiveStacker::~OpenLiveStacker()
{
}

void OpenLiveStacker::init(std::string driver_name,int id)
{
    auto drivers = CameraDriver::drivers();
    auto driver_it = std::find(drivers.begin(),drivers.end(),driver_name);
    if(driver_it == drivers.end())
        throw CamError("No such driver " + driver_name);
    driver_ = std::move(CameraDriver::get(driver_it - drivers.begin()));
    camera_ = std::move(driver_->open_camera(id));

    cppcms::json::value config;
    config["service"]["api"]="http";
    config["service"]["port"]=http_port;
    config["service"]["ip"] = http_ip;
    // change for service
    config["service"]["disable_global_exit_handling"] = false;
    config["file_server"]["enable"]=true;
    config["file_server"]["document_root"]=document_root;
    config["http"]["script"]="/api";
    config["http"]["timeout"]=5;
    config["logging"]["stderr"] = true;
    config["logging"]["level"] = "debug";

    web_service_ = std::shared_ptr<cppcms::service>(new cppcms::service(config));
    
    video_generator_app_ = new VideoGeneratorApp(*web_service_);
    web_service_->applications_pool().mount(video_generator_app_,cppcms::mount_point("/video/live",0));
    video_generator_.reset(new VideoGenerator(converter_queue_,video_generator_app_));
}

void OpenLiveStacker::handle_video_frame(CamFrame const &cf)
{
    std::shared_ptr<CameraFrame> frame(new CameraFrame());
    frame->format.format = cf.format;
    frame->format.width  = cf.width;
    frame->format.height = cf.height;
    frame->timestamp = cf.unix_timestamp;
    frame->source_frame = std::shared_ptr<VideoFrame>(new VideoFrame(cf.data,cf.data_size));
    converter_queue_->push(frame);
}
void OpenLiveStacker::run()
{
    video_generator_thread_ = std::move(std::thread([=](){ video_generator_->run(); }));
    camera_->start_stream(camera_->formats().at(0),[=](CamFrame const &cf) {
        handle_video_frame(cf);
    });
    web_service_->run();
    stop();
    
}
void OpenLiveStacker::shutdown()
{
    web_service_->shutdown();
}
void OpenLiveStacker::stop()
{
    camera_->stop_stream();
    converter_queue_->push(std::shared_ptr<QueueData>(new ShutDownData()));
    video_generator_thread_.join();
    video_generator_app_ = nullptr;
    video_generator_.reset();
    camera_.reset();
    driver_.reset();
}
}
