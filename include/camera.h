#pragma once
#include <stdexcept>
#include <string>
#include <ostream>
#include <vector>
#include <memory>
#include <functional>

namespace ols {

    class CamError : public std::runtime_error {
    public:
        CamError(std::string const &msg) : std::runtime_error(msg) {}
    };

    enum CamOptionId {
        opt_auto_exp,
        opt_auto_wb,
        opt_exp,
        opt_wb,
        opt_gain,
        opt_gamma,
        opt_count
    };

    enum CamOptionType {
        type_bool,
        type_int,
        type_float
    };

    enum CamOptionUnit {
        unit_generic,
        unit_sec,
        unit_msec,
        unit_percent,
        unit_kelvin,
    };

    struct CamParam {
        CamOptionId option;
        CamOptionType type;
        double step_size;
        double min_val;
        double max_val;
        double def_val;
        double cur_val;
    };

    enum CamStreamType {
        stream_yuv2,
        stream_mjpeg,
        stream_error,
    };

    struct CamStreamFormat {
        CamStreamType format;
        int width,height;
        float framerate; // -1 for unknown
    };

    std::ostream &operator<<(std::ostream &out,CamStreamFormat const &fmt);

    struct CamFrame {
        CamStreamType format;
        int frame_counter;
        double unix_timestamp;
        int width;
        int height;
        void const *data;
        size_t data_size;
    };

    typedef std::function<void(CamFrame const &)> frame_callback_type;

    class Camera {
    public:
        Camera() {}
        Camera(Camera const &) = delete;
        Camera(Camera &&) = delete;
        void operator = (Camera const &) = delete;
        void operator = (Camera &&) = delete;
        
        virtual std::string name() = 0;
        virtual std::vector<CamStreamFormat> formats() = 0;
        virtual void start_stream(CamStreamFormat format,frame_callback_type callback) = 0;
        virtual void stop_stream() = 0;
        virtual std::vector<CamOptionId> supported_options() = 0;
        virtual CamParam get_parameter(CamOptionId id) = 0;
        virtual void set_parameter(CamOptionId id,double value) = 0;
        virtual ~Camera() {}
    };

    class CameraDriver {
    public:
        static std::vector<std::string> drivers();
        static std::unique_ptr<CameraDriver> get(int id);
        virtual std::vector<std::string> list_cameras() = 0;
        virtual std::unique_ptr<Camera> open_camera(int id) = 0;
        virtual ~CameraDriver() {}
    };

} // namespace
