#include "camera.h"
#include <iostream>
#include <fstream>
#include "os_util.h"

int main()
{   
    try {
        ols::CamErrorCode e;
        auto drivers = ols::CameraDriver::drivers();
        for(auto driver: drivers)
            std::cout << "Driver " << driver << std::endl;
        std::unique_ptr<ols::CameraDriver> driver = ols::CameraDriver::get(0,-1);
        auto cameras = driver->list_cameras(e);
        e.check();
        for(auto camera: cameras)
            std::cout << "Camera " << camera << std::endl;
        int cam_id = 0;
        if(cameras.size() > 1) {
            std::cout << "Select :" << std::flush;
            std::cin >> cam_id;
        }
        std::unique_ptr<ols::Camera> camera = driver->open_camera(cam_id,e);
        e.check();
        std::string name = camera->name(e);
        e.check();
        std::cout << "Opened " << name << std::endl;
        
        auto formats = camera->formats(e);
        e.check();
        for(size_t i=0;i<formats.size();i++) {
            std::cout << "  " << i <<":"<< formats[i] << std::endl;
        }
        int fmt_id = 0;
        std::cin >> fmt_id;
        camera->start_stream(formats.at(fmt_id),[](ols::CamFrame frame) {
            std::cout <<" Got #" << frame.frame_counter << ": " << frame.width << "x"<<frame.height << " buffer " << frame.data_size << std::endl;
            if(frame.format == ols::stream_mjpeg) {
                std::ofstream f("/tmp/uvc_frame_" + std::to_string(frame.frame_counter) + ".jpeg");
                f.write((char*)(frame.data),frame.data_size);
                f.close();
            }
        },e);
        e.check();
        for(int i=0;i<20;i++) {
            usleep(999000);
        }
        camera->stop_stream(e);
        e.check();
        camera.reset();
        driver.reset();
    }
    catch(std::exception const &e) {
        std::cerr << "failed:" << e.what() << std::endl;
        return 1;
    }
    return 0;
}
