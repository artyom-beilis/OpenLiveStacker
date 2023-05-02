#include "ols.h"
#include <algorithm>
#include <cppcms/json.h>
#include <cppcms/applications_pool.h>
#include <cppcms/mount_point.h>
#include <fstream>
#include "camera_ctl.h"
#include "stacker_ctl_app.h"
#include "processors.h"
#include "common_utils.h"
#include "plate_solver.h"
#include "plate_solver_ctl_app.h"

namespace ols {

std::atomic<int> OpenLiveStacker::received_;

OpenLiveStacker::OpenLiveStacker(std::string data_dir)
{
    data_dir_ = data_dir;
    make_dir(data_dir_);
    debug_dir_ = data_dir + "/debug";
    make_dir(debug_dir_);
    make_dir(data_dir_ + "/stacked");
    make_dir(data_dir_ + "/calibration");
    if(!exists(data_dir_ + "/calibration/index.json")) {
        std::ofstream f(data_dir_ + "/calibration/index.json");
        if(!f)
            throw std::runtime_error("Failed to create calibration index");
        f<<"[]"<<std::endl;
    }
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
    CamErrorCode e;
    camera_ = std::move(driver_->open_camera(id,e));
    e.check();
    stream_active_ = false;
}
Camera &OpenLiveStacker::cam() 
{
    guard g(camera_lock_);
    if(!camera_)
        throw CamError("Camera is not open");
    return *camera_;
}

void OpenLiveStacker::disableCVThreads()
{
    cv::setNumThreads(0);
}

void OpenLiveStacker::close_camera()
{
    guard g(camera_lock_);
    if(!camera_)
        return;
    CamErrorCode e;
    camera_->stop_stream(e);
    e.check();
    camera_.reset();
    stream_active_ = false;
}

void OpenLiveStacker::stop_stream()
{
    guard g(camera_lock_);
    CamErrorCode e;
    cam().stop_stream(e);
    e.check();
    stream_active_ = false;
}
void OpenLiveStacker::start_stream(CamStreamFormat format,double max_framerate)
{
    guard g(camera_lock_);
    CamErrorCode e;
    max_framerate_ = max_framerate;
    int c = is_mono_stream(format.format) ? 1 : 3;
    int red=0,green=192,blue=0;
    if(c==1)
        red=192;
    auto frame = generate_dummy_frame(format.width,format.height,c,red,green,blue);
    dropped_since_last_update_ = 0;
    video_generator_queue_->push(frame);

    cam().start_stream(format,[=](CamFrame const &cf) {
        handle_video_frame(cf);
    },e);
    e.check();
    current_format_ = format;
    stream_active_ = true;
}

void OpenLiveStacker::init(std::string driver_name,int external_option)
{
    auto drivers = CameraDriver::drivers();
    auto driver_it = std::find(drivers.begin(),drivers.end(),driver_name);
    if(driver_it == drivers.end())
        throw CamError("No such driver " + driver_name);
    int driver_id = driver_it - drivers.begin();
    driver_ = std::move(CameraDriver::get(driver_id,external_option));

    cppcms::json::value config;
    config["service"]["api"]="http";
    config["service"]["port"]=http_port;
    config["service"]["ip"] = http_ip;
    config["service"]["worker_threads"] = 3;
#ifdef ANDROID_SUPPORT
    config["service"]["disable_global_exit_handling"] = true;
#else
    // change for service
    config["service"]["disable_global_exit_handling"] = false;
#endif    
    config["file_server"]["enable"]=true;
    config["file_server"]["document_root"]=document_root;
    config["file_server"]["alias"][0]["url"] = "/data";
    config["file_server"]["alias"][0]["path"] = data_dir_;
    config["http"]["script"]="/api";
    config["http"]["timeout"]=5;
#ifndef ANDROID_SUPPORT
    config["logging"]["stderr"] = true;
#endif    
    config["logging"]["level"] = "info";
    config["logging"]["file"]["name"] = debug_dir_ + "/log.txt";
    config["logging"]["file"]["max_files"] = 10;

    web_service_ = std::shared_ptr<cppcms::service>(new cppcms::service(config));
    
    video_generator_app_ = new VideoGeneratorApp(*web_service_,"Real time video");
    stacked_video_generator_app_ = new VideoGeneratorApp(*web_service_,"Stacked video");
    stats_stream_app_ = new StackerStatsNotification(*web_service_);
    web_service_->applications_pool().mount(video_generator_app_,cppcms::mount_point("/video/live",0));
    web_service_->applications_pool().mount(stacked_video_generator_app_,cppcms::mount_point("/video/stacked",0));
    web_service_->applications_pool().mount(cppcms::create_pool<CameraControlApp>(this),cppcms::mount_point("/camera((/.*)?)",1));
    web_service_->applications_pool().mount(cppcms::create_pool<StackerControlApp>(this,data_dir_,video_generator_queue_),
                                            cppcms::mount_point("/stacker((/.*)?)",1),
                                            cppcms::app::asynchronous);
    web_service_->applications_pool().mount(stats_stream_app_,cppcms::mount_point("/updates",0));
    web_service_->applications_pool().mount(cppcms::create_pool<PlateSolverControlApp>(data_dir_),cppcms::mount_point("/plate_solver((/.*)?)",1));
}



void OpenLiveStacker::handle_video_frame(CamFrame const &cf)
{
    auto now = booster::ptime::now();
    if(max_framerate_ > 0 && booster::ptime::to_number(now - last_frame_ts_) < 1.0/max_framerate_)
        return;
    last_frame_ts_ = now;

    received_ ++;
    if(video_generator_queue_->items > 20) {
        dropped_since_last_update_ ++;
        BOOSTER_WARNING("stacker") << "Processing is overloaded, dropping frame #" << (++dropped_);
        return;
    }
    std::shared_ptr<CameraFrame> frame(new CameraFrame());
    frame->format.format = cf.format;
    frame->format.width  = cf.width;
    frame->format.height = cf.height;
    frame->bayer = cf.bayer;
    frame->timestamp = cf.unix_timestamp;
    frame->source_frame = std::shared_ptr<VideoFrame>(new VideoFrame(cf.data,cf.data_size));
    frame->dropped = dropped_since_last_update_;
    dropped_since_last_update_ = 0;
    video_generator_queue_->push(frame);
}

void OpenLiveStacker::set_plate_solving_image(data_pointer_type p)
{
    std::shared_ptr<CameraFrame> frame = std::dynamic_pointer_cast<CameraFrame>(p);
    if(!frame)
        return;
    
    PlateSolver::set_image(frame->frame,frame->frame_dr);
}

void OpenLiveStacker::run()
{
    video_display_queue_->call_on_push(video_generator_app_->get_callback());
    stack_display_queue_->call_on_push(stacked_video_generator_app_->get_callback());
    stacker_stats_queue_->call_on_push(stats_stream_app_->get_callback());
    plate_solving_queue_->call_on_push(set_plate_solving_image);

    video_generator_thread_ = std::move(start_generator(video_generator_queue_,
                                                        preprocessor_queue_,
                                                        video_display_queue_,
                                                        debug_save_queue_,
                                                        plate_solving_queue_));

    debug_save_thread_ = std::move(start_debug_saver(debug_save_queue_,debug_dir_));
    preprocessor_thread_ = std::move(start_preprocessor(preprocessor_queue_,stacker_queue_));
    stacker_thread_ = std::move(start_stacker(stacker_queue_,
                                              stack_display_queue_,
                                              stacker_stats_queue_,
                                              plate_solving_queue_,
                                              data_dir_));
    web_service_->run();
    stop();
    
}
void OpenLiveStacker::shutdown()
{
    web_service_->shutdown();
}
void OpenLiveStacker::stop()
{
    if(camera_) {
        CamErrorCode e;
        camera_->stop_stream(e);
        if(e) {
            BOOSTER_ERROR("stacker") << "Can't stop stream:" << e.message();
        }
    }
    video_generator_queue_->push(std::shared_ptr<QueueData>(new ShutDownData()));

    video_generator_queue_.reset();
    preprocessor_queue_.reset();
    stacker_queue_.reset();
    video_display_queue_.reset();
    debug_save_queue_.reset();
    stack_display_queue_.reset();

    video_generator_thread_.join();
    debug_save_thread_.join();
    preprocessor_thread_.join();
    stacker_thread_.join();

    camera_.reset();
    driver_.reset();
}
}
