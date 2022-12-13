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
        opt_brightness,
        opt_contrast,
        opt_count
    };

    //int str_to_enum(std::string const &name,char const **names,size_t names_size);
    //std::string enum_to_str(int v,char const **names,size_t names_size);

    std::string cam_option_id_to_string_id(CamOptionId id);
    std::string cam_option_id_to_name(CamOptionId id);
    CamOptionId cam_option_id_from_string_id(std::string const &name);

    enum CamOptionType {
        type_bool,
        type_number,
        type_msec,
        type_percent,
        type_kelvin,
    };

    std::string cam_option_type_to_str(CamOptionType type);
    CamOptionType cam_option_type_from_str(std::string const &name);

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

    std::string stream_type_to_str(CamStreamType s);
    CamStreamType stream_type_from_str(std::string s);

    struct CamStreamFormat {
        CamStreamType format;
        int width,height;
        float framerate; // -1 for unknown
    };

    std::ostream &operator<<(std::ostream &out,CamStreamFormat const &fmt);

    ///
    /// frame received by \a frame_callback_type 
    ///
    struct CamFrame {
        CamStreamType format; /// image format
        int frame_counter;    /// frame counter since camera started
        double unix_timestamp; /// time in sconds since Jan 1, 1970 when image was captured, inlcuding subsecond units
        int width;  /// image width
        int height; /// image height
        void const *data; /// data - pointer becomes invalid when callback ends, points to null terminamted char const * for \a stream_error format to indicate caputre error
        size_t data_size; /// size of data in bytes
    };

    /// callback to pass to camera - called from separate thread when frame is ready
    typedef std::function<void(CamFrame const &)> frame_callback_type;


    ///
    /// Generic Camera interface - to be implemented for new camera driver
    ///
    class Camera {
    public:
        Camera() {}
        Camera(Camera const &) = delete;
        Camera(Camera &&) = delete;
        void operator = (Camera const &) = delete;
        void operator = (Camera &&) = delete;
        
        /// Camera name
        virtual std::string name() = 0;
        /// Return list of suppored video formats
        virtual std::vector<CamStreamFormat> formats() = 0;

        /// Start a video stream with provided callback 
        virtual void start_stream(CamStreamFormat format,frame_callback_type callback) = 0;

        /// stop the stream - once function ends callback will not be called any more
        virtual void stop_stream() = 0;

        /// list of camera controls that the camera supports
        virtual std::vector<CamOptionId> supported_options() = 0;
        /// get camera control
        virtual CamParam get_parameter(CamOptionId id,bool current_only = false) = 0;
        /// set camera control
        virtual void set_parameter(CamOptionId id,double value) = 0;
        virtual ~Camera() {}
    };


    /// Base class for driver to implement
    class CameraDriver {
    public:
        /// list supported drivers
        static std::vector<std::string> drivers();
        /// get driver - by its order in the \a drivers result
        static std::unique_ptr<CameraDriver> get(int id);

        /// list all cameras connected and supported by this driver
        virtual std::vector<std::string> list_cameras() = 0;

        /// get camera object for camera according to the index in list_cameras
        virtual std::unique_ptr<Camera> open_camera(int id) = 0;

        virtual ~CameraDriver() {}
    };

} // namespace
