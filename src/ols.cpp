#include "ols.h"
#include <algorithm>
#include <cppcms/json.h>
#include <cppcms/applications_pool.h>
#include <cppcms/mount_point.h>
#include "camera_ctl.h"

namespace ols {
OpenLiveStacker::OpenLiveStacker()
{
}

OpenLiveStacker::~OpenLiveStacker()
{
}

CameraInterface::CamStatus OpenLiveStacker::status()
{
    guard g(camera_lock_);
    if(!camera_)
        return cam_closed;
    if(stream_active_)
        return cam_streaming;
    else
        return cam_open;

}

void OpenLiveStacker::open_camera(int id) 
{
    close_camera();
    guard g(camera_lock_);
    camera_ = std::move(driver_->open_camera(id));
    stream_active_ = false;
}
Camera &OpenLiveStacker::cam() 
{
    guard g(camera_lock_);
    if(!camera_)
        throw CamError("Camera is not open");
    return *camera_;
}
void OpenLiveStacker::close_camera()
{
    guard g(camera_lock_);
    if(!camera_)
        return;
    camera_->stop_stream();
    camera_.reset();
    stream_active_ = false;
}

void OpenLiveStacker::stop_stream()
{
    guard g(camera_lock_);
    cam().stop_stream();
    stream_active_ = false;
}
void OpenLiveStacker::start_stream(CamStreamFormat format)
{
    guard g(camera_lock_);
    cam().start_stream(format,[=](CamFrame const &cf) {
        handle_video_frame(cf);
    });
    current_format_ = format;
    stream_active_ = true;
}

void OpenLiveStacker::init(std::string driver_name)
{
    auto drivers = CameraDriver::drivers();
    auto driver_it = std::find(drivers.begin(),drivers.end(),driver_name);
    if(driver_it == drivers.end())
        throw CamError("No such driver " + driver_name);
    driver_ = std::move(CameraDriver::get(driver_it - drivers.begin()));

    cppcms::json::value config;
    config["service"]["api"]="http";
    config["service"]["port"]=http_port;
    config["service"]["ip"] = http_ip;
    config["service"]["worker_threads"] = 3;
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
    web_service_->applications_pool().mount(cppcms::create_pool<CameraControlApp>(this),cppcms::mount_point("/camera((/.*)?)",1));
}

void OpenLiveStacker::handle_video_frame(CamFrame const &cf)
{
    std::shared_ptr<CameraFrame> frame(new CameraFrame());
    frame->format.format = cf.format;
    frame->format.width  = cf.width;
    frame->format.height = cf.height;
    frame->timestamp = cf.unix_timestamp;
    frame->source_frame = std::shared_ptr<VideoFrame>(new VideoFrame(cf.data,cf.data_size));
    video_generator_queue_->push(frame);
}
void OpenLiveStacker::run()
{
    video_generator_thread_ = std::move(start_generator(video_generator_queue_,video_display_queue_));
    video_display_queue_->call_on_push(video_generator_app_->get_callback());
    web_service_->run();
    stop();
    
}
void OpenLiveStacker::shutdown()
{
    web_service_->shutdown();
}
void OpenLiveStacker::stop()
{
    if(camera_)
        camera_->stop_stream();
    video_generator_queue_->push(std::shared_ptr<QueueData>(new ShutDownData()));
    video_generator_queue_.reset();
    video_display_queue_.reset();
    video_generator_thread_.join();
    camera_.reset();
    driver_.reset();
}
}
