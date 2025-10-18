#include "ols.h"
#include <algorithm>
#include <iomanip>
#include <cppcms/json.h>
#include <cppcms/applications_pool.h>
#include <cppcms/mount_point.h>
#include <fstream>
#include "camera_ctl.h"
#include "mount_ctl.h"
#include "stacker_ctl_app.h"
#include "processors.h"
#include "common_utils.h"
#include "plate_solver.h"
#include "allocator.h"
#include "plate_solver_ctl_app.h"
#include "astap_db_download_app.h"
#include "allocator.h"
#include "config_app.h"

namespace ols {

std::atomic<int> OpenLiveStacker::received_;
std::atomic<int> CameraControlApp::live_stretch_;
std::atomic<int> CameraControlApp::defaults_configured_;
std::recursive_mutex ConfigApp::lock_;

OpenLiveStacker::OpenLiveStacker(std::string data_dir)
{
    data_dir_ = data_dir;
    make_dir(data_dir_);
    debug_dir_ = data_dir + "/debug";
    make_dir(debug_dir_);
    make_dir(data_dir_ + "/stacked");
    make_dir(data_dir_ + "/calibration");
    auto db_path=PlateSolver::db_path();
    if(!db_path.empty())
        make_dir(db_path);
    if(!exists(data_dir_ + "/calibration/index.json")) {
        std::ofstream f(data_dir_ + "/calibration/index.json");
        if(!f)
            throw std::runtime_error("Failed to create calibration index");
        f<<"[]"<<std::endl;
    }
}

std::string OpenLiveStacker::log_queues()
{
    std::vector<std::pair<queue_pointer_type,std::string> > queues = {
        { video_generator_queue_, "video_generator" },
        { video_display_queue_, "video_display" },
        { preprocessor_queue_, "preprocessor" },
        { stacker_queue_, "stacker" },
        { pp_queue_, "pp" },
        { stack_display_queue_, "stack_display" },
        { debug_save_queue_, "debug_save" },
        { stacker_stats_queue_, "stacker_stats" },
        { plate_solving_queue_, "plate_solving" }
    };

    std::ostringstream ss;
    for(auto &p:queues) {
        size_t size = p.first->size();
        if(size > 0) {
            ss<<std::setw(5) << size << " " << p.second << "\n";
        }
    }
    return ss.str(); 
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
    camera_ = driver_->open_camera(id,e);
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
    driver_ = CameraDriver::get(driver_id,external_option);

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
    config.set("file_server.check_symlink",false);
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
    vr_generator_app_ = new VideoGeneratorApp(*web_service_,"VR video");
    stats_stream_app_ = new StackerStatsNotification(*web_service_);
    web_service_->applications_pool().mount(video_generator_app_,cppcms::mount_point("/video/live",0));
    web_service_->applications_pool().mount(stacked_video_generator_app_,cppcms::mount_point("/video/stacked",0));
    web_service_->applications_pool().mount(vr_generator_app_,cppcms::mount_point("/video/vr",0));
    web_service_->applications_pool().mount(cppcms::create_pool<ConfigApp>(data_dir_),cppcms::mount_point("/config((/.*)?)",1));
    web_service_->applications_pool().mount(cppcms::create_pool<CameraControlApp>(this,video_generator_queue_),cppcms::mount_point("/camera((/.*)?)",1));
    web_service_->applications_pool().mount(cppcms::create_pool<MountControlApp>(stacker_stats_queue_,this),cppcms::mount_point("/mount((/.*)?)",1));
    web_service_->applications_pool().mount(cppcms::create_pool<StackerControlApp>(this,data_dir_,video_generator_queue_),
                                            cppcms::mount_point("/stacker((/.*)?)",1),
                                            cppcms::app::asynchronous);
    web_service_->applications_pool().mount(cppcms::create_pool<AstapDBDownloadApp>(PlateSolver::db_path()),
                                            cppcms::mount_point("/astap_db((/.*)?)",1),
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

    bool drop_mem = false,drop_queue = false;
    size_t allocated_mb = 0;
    if(mem_limit_mb > 0) {
        size_t mem_limit_bytes = size_t(1024ull*1024ull) * mem_limit_mb;
        size_t allocated = AllocatorGuard::allocated();
        allocated_mb = allocated >> 20;
        if(allocated >= mem_limit_bytes) {
            drop_mem = true;
        }
    }
    int queue_items = video_generator_queue_->items;
    constexpr int queue_size_limit = 10;
    if(queue_items > queue_size_limit) {
        drop_queue = true;
    }

    if(drop_queue || drop_mem) {
        dropped_since_last_update_ ++;
        BOOSTER_WARNING("stacker") << "Processing is overloaded, dropping frame #" << (++dropped_) 
            << " Mem used " << allocated_mb << "MB out of " << mem_limit_mb << " Items in queue " << queue_items << " out of " << queue_size_limit;
        auto now = booster::ptime::now();
        if(booster::ptime::to_number(now - last_dropped_frame_ts_) > 1.0) {
            BOOSTER_WARNING("stacker") << "Queues status\n" << log_queues();
            last_dropped_frame_ts_ = now;
        }
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
    
    cv::Mat ps_frame;
    if(frame->format.format == stream_yuv2) // unsupported by ASTAP
        ps_frame = frame->frame;
    else
        ps_frame = frame->raw;
    
    PlateSolver::set_image(ps_frame,frame->live_is_stretched);
}

void OpenLiveStacker::guide(data_pointer_type p)
{
    std::shared_ptr<PulseGuide> guide_ptr = std::dynamic_pointer_cast<PulseGuide>(p);
    if(!guide_ptr) {
        BOOSTER_ERROR("ols") << "Wrong type for guiding";
        return;
    }
    guard_type g = guard(); 
    if(!mount_is_loaded()) {
        BOOSTER_ERROR("ols") << "Mount is not loaded";
        return;
    }
    Mount *mcl = client();
    if(!mcl->connected()) {
        BOOSTER_ERROR("ols") << "Mount is not connected";
        return;
    }
    MountErrorCode e;
    mcl->pulse_guide(guide_ptr->NS_ms,guide_ptr->WE_ms,e);
    if(e) {
        BOOSTER_ERROR("ols") << "Pulse Guide Failed " << e.message();
    }
}

void OpenLiveStacker::run()
{
    video_display_queue_->call_on_push(video_generator_app_->get_callback());
    stack_display_queue_->call_on_push(stacked_video_generator_app_->get_callback());
    vr_display_queue_->call_on_push(vr_generator_app_->get_callback());
    stacker_stats_queue_->call_on_push(stats_stream_app_->get_callback());
    plate_solving_queue_->call_on_push(set_plate_solving_image);
    guide_queue_->call_on_push([=](data_pointer_type p) { this->guide(p); });

    video_generator_thread_ = start_generator(video_generator_queue_,
                                                        preprocessor_queue_,
                                                        video_display_queue_,
                                                        debug_save_queue_,
                                                        plate_solving_queue_,
                                                        guide_queue_,
                                                        stacker_stats_queue_,
                                                        vr_queue_);

    debug_save_thread_ = start_debug_saver(debug_save_queue_,stacker_stats_queue_,debug_dir_);
    preprocessor_thread_ = start_preprocessor(preprocessor_queue_,stacker_queue_,stacker_stats_queue_);
    stacker_thread_ = start_stacker(stacker_queue_,pp_queue_);
    pp_thread_ = start_post_processor(pp_queue_,
                                              stack_display_queue_,
                                              stacker_stats_queue_,
                                              plate_solving_queue_,
                                              vr_queue_,
                                              data_dir_);

    vr_thread_ = start_vr_processor(vr_queue_,vr_display_queue_);
    try {
        web_service_->run();
    }
    catch(...) {
        stop();
        throw;
    }
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
    pp_thread_.join();
    vr_thread_.join();

    camera_.reset();
    driver_.reset();
    shutdown_mount();
}
}
