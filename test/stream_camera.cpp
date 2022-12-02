#include "camera.h"
#include <iostream>
#include <fstream>
#include <unistd.h>

int main()
{   
    try {
        auto drivers = ols::CameraDriver::drivers();
        for(auto driver: drivers)
            std::cout << "Driver " << driver << std::endl;
        std::unique_ptr<ols::CameraDriver> driver = ols::CameraDriver::get(0);
        auto cameras = driver->list_cameras();
        for(auto camera: cameras)
            std::cout << "Camera " << camera << std::endl;
        int cam_id = 0;
        if(cameras.size() > 1) {
            std::cout << "Select :" << std::flush;
            std::cin >> cam_id;
        }
        std::unique_ptr<ols::Camera> camera = driver->open_camera(cam_id);
        std::cout << "Opened " << camera->name() << std::endl;
        
        auto formats = camera->formats();
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
        });
        sleep(20);
        camera->stop_stream();
        camera.reset();
        driver.reset();
    }
    catch(std::exception const &e) {
        std::cerr << "failed:" << e.what();
        return 1;
    }
    return 0;
}
