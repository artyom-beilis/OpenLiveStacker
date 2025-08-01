#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#endif

#include "camera.h"
#include "toupcam.h"
#include "toup_oem.h"
#include "shift_bit.h"
#include <mutex>
#include <sstream>
#include <iostream>
#include <atomic>
#include <map>
#include <string.h>
#include <thread>
#include <algorithm>
#include <cmath>
#ifdef _WIN32
#include <booster/locale/utf.h>
#endif
#include "os_util.h"

#ifdef _WIN32
namespace {
        
    template<typename CharOut,typename CharIn>
    std::basic_string<CharOut>
    convert_impl(std::basic_string<CharIn> const &s)
    {
        using namespace booster::locale;
        std::basic_string<CharOut> result;
        CharIn const *begin = s.c_str();
        CharIn const *end = begin + s.size();
        result.reserve(end-begin);
        typedef std::back_insert_iterator<std::basic_string<CharOut> > inserter_type;
        inserter_type inserter(result);
        utf::code_point c;
        while(begin!=end) {
            c=utf::utf_traits<CharIn>::template decode<CharIn const *>(begin,end);
            if(c==utf::illegal || c==utf::incomplete) {
                break;
            }
            else {
                utf::utf_traits<CharOut>::template encode<inserter_type>(c,inserter);
            }
        }
        return result;
    }
    inline std::wstring convert(std::string const &s)
    {
        return convert_impl<wchar_t>(s);
    }
    inline std::string convert(std::wstring const &s)
    {
        return convert_impl<char>(s);
    }
}
#else
namespace {
    inline std::string convert(std::string const &s)
    {
        return s;
    }
}
#endif

namespace ols
{
    static const unsigned int TOUPCAM_ERROR_OPEN_BY_INDEX = -1;
    static const unsigned int TOUPCAM_ERROR_OPEN_BY_INFO = -2;
    static const unsigned int TOUPCAM_ERROR_INVALID_OPTION = -3;
    static const unsigned int TOUPCAM_ERROR_OPEN_MODEL = -4;
    static std::string make_message(std::string const &msg, unsigned int code)
    {
        std::string str;
        if (SUCCEEDED(code))
        {
            str = "Success";
        }
        else
            switch (code)
            {
            case TOUPCAM_ERROR_OPEN_BY_INDEX:
                str = "no camera connected or index value out of boundary";
                break;
            case TOUPCAM_ERROR_OPEN_BY_INFO:
                str = "Strange error";
                break;
            case TOUPCAM_ERROR_INVALID_OPTION:
                str = "Invalid option";
                break;
            case TOUPCAM_ERROR_OPEN_MODEL:
                str = "Toupcam_query_Model returned nullptr";
                break;
            case 0x80004005:
                str = "Unspecified failure";
                break;
            case 0x80070005:
                str = "General access denied error";
                break;
            case 0x80070057:
                str = "One or more arguments are not valid";
                break;
            case 0x80004001:
                str = "Not supported or not implemented";
                break;
            case 0x80004002:
                str = "Interface not supported";
                break;
            case 0x80004003:
                str = "Pointer that is not valid";
                break;
            case 0x8000FFFF:
                str = "Unexpected failure";
                break;
            case 0x8007000E:
                str = "Out of memory";
                break;
            case 0x8001010E:
                str = "Call function in the wrong thread";
                break;
            case 0x8007001F:
                str = "Device not functioning";
                break;
            case 0x8000000A:
                str = "The data necessary to complete this operation is not yet available";
                break;
            default:
                str = "unknown code: " + std::to_string(code);
            };
        return msg + ": " + str;
    }
    static const unsigned int AVERAGE_BINNING_FLAG = 0x80; // usefull option for bin > 1
    static const double TOUPCAM_2_OLS_TEMP = 0.1;          // in 0.1℃ units (32 means 3.2℃, -35 means -3.5℃).
    static const double TOUPCAM_2_OLS_TIME = 1000.0;       // Toupcom API in microseconds, OLS API in ms
    static const unsigned int ROI_MIN_WIDTH_HEIGHT = 16;   // xOffset, yOffset, xWidth, yHeight: must be even numbers, xWidth, yHeight >= 16
    /** https://www.cloudynights.com/topic/875680-openlivestacker-getting-ready-for-google-play-discussions/?p=12823060
    * "The HCG mode for my camera decreases the read noise at the cost of well capacity.
    * So HCG should be better for very short exposures.
    * LCG for long exposure. I assume with this phone app you would go for unguided short exposures, so maybe HCG should be the default"
    */
    static const unsigned int CG_OLS_DEFAULT_VAL = 1;
    
    void StartPullCallback(unsigned nEvent, void *pCallbackCtx);
    void AutoWB(const int nTemp, const int nTint, void *pCtx);
    class ToupcamCamera : public Camera
    {
    public:
        typedef std::unique_lock<std::recursive_mutex> cam_guard_type;
        /// for Android
#ifndef _WIN32
        ToupcamCamera(std::string const &android_id,std::string const &camera_name, CamErrorCode &e)
        {
            memset(&info_,0,sizeof(info_));
            stream_active_ = 0;
            hcam_ = Toupcam_Open(android_id.c_str());
            if (NULL == hcam_)
            {
                e = make_message("Failed to open camera with id/name = " + android_id + "/" + camera_name + ", hcam_ is NULL", TOUPCAM_ERROR_OPEN_BY_INFO);
                return;
            }
            strncpy(info_.id,android_id.c_str(),sizeof(info_.id)-1);
            strncpy(info_.displayname,camera_name.c_str(),sizeof(info_.displayname)-1);
            info_.model = Toupcam_query_Model(hcam_);
            if(info_.model == nullptr) {
                e = make_message("Failed to access model for camera " + android_id + "/" + camera_name, TOUPCAM_ERROR_OPEN_MODEL);
                return;
            }

            init(e);
        }
#endif        
        ToupcamCamera(ToupcamDeviceV2 info, CamErrorCode &e) : info_(info)
        {
            stream_active_ = 0;
            hcam_ = Toupcam_Open(info.id);
            if (NULL == hcam_)
            {
                e = make_message("Failed to open camera, hcam_ is NULL", TOUPCAM_ERROR_OPEN_BY_INFO);
                return;
            }
            init(e);
        }
        virtual ~ToupcamCamera()
        {
            CamErrorCode e;
            stop_stream(e);
            if (e)
            {
                fprintf(stderr, "Failed to close stream %s\n", e.message().c_str());
            }
            if (NULL != hcam_)
                Toupcam_Close(hcam_);
            hcam_ = NULL;
        }
        /// Camera name
        virtual std::string name(CamErrorCode &) override
        {
            return name_;
        }
        /// Return list of suppored video formats
        virtual std::vector<CamStreamFormat> formats(CamErrorCode &) override
        {
            cam_guard_type g(camera_lock_);
            std::vector<CamStreamFormat> res;
            std::vector<CamStreamType> video_formats;
            bool isColor = !(info_.model->flag & TOUPCAM_FLAG_MONO);
            if(isColor)
                video_formats.push_back(stream_rgb24);
            video_formats.push_back(isColor ? stream_raw16 : stream_mono16);

            unsigned preview = info_.model->preview; // number of preview resolution, same as Toupcam_get_ResolutionNumber()

            for (unsigned i = 0; i < preview; ++i)
            {
                CamStreamFormat sfmt;
                sfmt.width = info_.model->res[i].width;
                sfmt.height = info_.model->res[i].height;
                sfmt.bin = (int)std::round(1.0 * info_.model->res[0].width / info_.model->res[i].width);

                for (auto video_format : video_formats)
                {
                    sfmt.format = video_format;
                    res.push_back(sfmt);
                }
            }
            auto ret = res; // duplicate before adding ROI options
            if(info_.model->flag & TOUPCAM_FLAG_ROI_HARDWARE) // Let's support HW ROI only
            for (auto sfmt : res) // resolutions x ROI options
            {
                int prevW = 1;
                sfmt.roi_den = 1;
                for(int scale = 2;scale < 50;scale++) {
                    int newW = (info_.model->res[0].width / sfmt.bin) / scale / ROI_MIN_WIDTH_HEIGHT * ROI_MIN_WIDTH_HEIGHT;
                    if(prevW == newW) continue;
                    int newH = (info_.model->res[0].height / sfmt.bin) / scale / ROI_MIN_WIDTH_HEIGHT * ROI_MIN_WIDTH_HEIGHT; // not really required 16 pix granulatity, but let make larg steps 
                    if(std::min(newW,newH) < 320)
                        break;
                    prevW = newW;
                    sfmt.width = newW;
                    sfmt.height = newH;
                    sfmt.roi_den = scale;
                    ret.push_back(sfmt);
                }
            }
            return ret;
        }
        void handle_error(const char *msg)
        {
            CamFrame frm = {};
            frm.format = stream_error;
            frm.data = msg;
            frm.data_size = strlen(msg);
            std::unique_lock<std::mutex> guard(lock_);
            if (callback_)
            {
                callback_(frm);
            }
        }
        void handle_error(std::string const &info,int code)
        {
            cam_guard_type g(camera_lock_);
            CamFrame frm = {};
            std::string msg =make_message(info,code);
            frm.format = stream_error;
            frm.data = msg.c_str();
            frm.data_size =msg.size();
            std::unique_lock<std::mutex> guard(lock_);
            if (callback_)
            {
                callback_(frm);
            }
        }
        void handle_frame()
        {
            cam_guard_type g(camera_lock_);
            CamFrame frm;
            int hr;
            int nW = 1, nH = 1;
            unsigned xOffset, yOffset, xWidth, yHeight;
            if (FAILED(hr = Toupcam_get_Roi(hcam_, &xOffset, &yOffset, &xWidth, &yHeight))) {
                handle_error("Toupcam_get_Roi",hr);
                return;
            }
            if(xOffset | yOffset | xWidth | yHeight) // Roi is set
            {
                nW =  (int)xWidth;
                nH =  (int)yHeight;             
            }
            else if (FAILED(hr = Toupcam_get_Size(hcam_, &nW, &nH))) {
                handle_error("Toupcam_get_Size",hr);
                return; 
            }

            int binning = -1;
            hr = Toupcam_get_Option(hcam_, TOUPCAM_OPTION_BINNING, &binning);
            if (FAILED(hr))
            {
                handle_error("Toupcam_get_Option BINNING",hr);
                return;
            }
            binning &= ~AVERAGE_BINNING_FLAG; // remove flag before calculation
            updateWHByBin(nW, nH, binning);
            int raw = -1;
            hr = Toupcam_get_Option(hcam_, TOUPCAM_OPTION_RAW, &raw);
            if (FAILED(hr))
            {
                handle_error("Toupcam_get_Option RAW",hr);
                return;
            }
            int bpp = -1;
            if (raw == 0)
            {
                bpp = 3;
                frm.format = stream_rgb24;
            }
            else if (raw == 1)
            {
                bpp = 2; // update to 1/2 if TOUPCAM_OPTION_BITDEPTH 0/1 for now using 1 unconditionaly
                frm.format = !(info_.model->flag & TOUPCAM_FLAG_MONO) ? stream_raw16 : stream_mono16;
            }
            // printf("handle_frame=%dx%d(raw=%d, bpp=%d)\n", nW, nH, raw, bpp);
            size_t bufSize = nW * nH * bpp;
            if (buf_.size() != bufSize) // Just in case
                buf_.resize(bufSize);
            unsigned w = 0, h = 0;
            hr = Toupcam_PullImage(hcam_, buf_.data(), bpp * 8, &w, &h);
            if (FAILED(hr)) {
                handle_error("Toupcam_PullImage",hr);
                return;
            }

/*            // printf("handle_frame: Toupcam_PullImage=(%ux%u)\n", w, h);
            hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_FLUSH, 2);
            if (FAILED(hr))
            {
                handle_error("Toupcam_put_Option FLUSH",hr);
                // printf("handle_frame:Toupcam_put_Option(TOUPCAM_OPTION_FLUSH, 2)=%d\n", hr);
            }*/

            if(bpp == 2) {
                int bits=16;
                if(info_.model->flag & TOUPCAM_FLAG_RAW10)
                    bits = 10;
                else if(info_.model->flag & TOUPCAM_FLAG_RAW12)
                    bits = 12;
                else if(info_.model->flag & TOUPCAM_FLAG_RAW14)
                    bits = 14;
                if(bits != 16) 
                    shift_to16(bits,reinterpret_cast<uint16_t *>(buf_.data()),buf_.size()/2);
            }
            
#ifdef _WIN32
            ::ols_util::timeval tv;
            ::ols_util::gettimeofday(&tv, nullptr);
#else            
            struct timeval tv;
            gettimeofday(&tv, nullptr);
#endif            
            frm.unix_timestamp = tv.tv_sec;
            frm.unix_timestamp += tv.tv_usec * 1e-6;
            frm.width = w;
            frm.height = h;
            frm.data = buf_.data();
            frm.data_size = buf_.size();
            frm.frame_counter = frame_counter_++;
            frm.bayer = bayerPattern_;
            {
                std::unique_lock<std::mutex> guard(lock_);
                if (callback_)
                {
                    callback_(frm);
                }
            }
        }

        /// Start a video stream with provided callback
        virtual void start_stream(CamStreamFormat format, frame_callback_type callback, CamErrorCode &e) override
        {
            cam_guard_type g(camera_lock_);
            //std::stringstream ss;
            int hr;
            //ss << format;
            //std::string s = ss.str();
            // printf("\n\nstart_stream: %s\n", s.c_str());
            if (stream_active_ != 0)
            {
                stop_stream(e);
                if (e)
                    return;
            }
            int bpp = -1;
            int raw = 1; // raw data mode, read the sensor "raw" data. This can be set only BEFORE Toupcam_StartXXX(). 0 = rgb, 1 = raw, default value: 0
            switch (format.format)
            {
            case stream_rgb24:
                bpp = 3;
                raw = 0;
                break;
            case stream_raw16:
            case stream_mono16:
                bpp = 2;
                break;
            default:
                e = "Invalid format";
                return;
            }
            int binning = format.bin;
            int binning_avg = binning;
            if (average_bin_support_ != NULL && *average_bin_support_)
            {
                binning_avg |= AVERAGE_BINNING_FLAG;
            }

            hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_BINNING, binning_avg);
            if (FAILED(hr))
            {
                e = make_message("Failed to Toupcam_put_Option(TOUPCAM_OPTION_BINNING)", hr);
                return;
            }
            int width, height;
            if (FAILED(hr = Toupcam_get_Size(hcam_, &width, &height)))
            {
                e = make_message("Failed to Toupcam_get_Size", hr);
                return;
            }

            updateWHByBin(width, height, binning);

            // Data buffer
            buf_.resize(width * height * bpp);
            hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_RAW, raw);
            if (FAILED(hr))
            {
                e = make_message("Failed to Toupcam_put_Option(TOUPCAM_OPTION_RAW)", hr);
                return;
            }
            /**
             * TOUPCAM_OPTION_FRAMERATE	limit the frame rate, range=[0, 63]. 0 (means no limit)
             */
            // int frameRate = format.framerate;
            // For astroimage 0 seems no sense
            /*if (frameRate < 1)
                frameRate = 1;
            else if (frameRate > 63)
                frameRate = 0;
            hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_FRAMERATE, frameRate);
            if (FAILED(hr))
            {
                e = make_message("Failed to Toupcam_put_Option(TOUPCAM_OPTION_FRAMERATE)", hr);
                return;
            }*/

            setROI(format, e);
            if(e)
                return;

            hr = Toupcam_StartPullModeWithCallback(hcam_, StartPullCallback, this);
            if (FAILED(hr))
            {
                // printf("\n\nToupcam_StartPullModeWithCallback = %x\n\n", hr);
                e = make_message("Failed to Toupcam_StartPullModeWithCallback", hr);
                return;
            }
            /* 1 = hard flush, discard frames cached by camera DDR (if any)
               2 = soft flush, discard frames cached by toupcam.dll (if any)
               3 = both flush */
/*            hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_FLUSH, 3);
            if (FAILED(hr))
            {
                // printf("start_stream:Toupcam_put_Option(TOUPCAM_OPTION_FLUSH, 3)=%d\n", hr);
            }*/
            {
                std::unique_lock<std::mutex> guard(lock_);
                callback_ = callback;
            }

            stream_active_ = 1;


#ifndef MAKEFOURCC
#define MAKEFOURCC(a, b, c, d) ((unsigned)(unsigned char)(a) | ((unsigned)(unsigned char)(b) << 8) | ((unsigned)(unsigned char)(c) << 16) | ((unsigned)(unsigned char)(d) << 24))
#endif
            /**
             * TODO: Not yet tested
             */
            unsigned nFourCC;
            unsigned bitsperpixel;
            hr = Toupcam_get_RawFormat(hcam_, &nFourCC, &bitsperpixel);
            bayerPattern_ = bayer_na;
            if (SUCCEEDED(hr))
                switch (nFourCC)
                {
                case MAKEFOURCC('G', 'B', 'R', 'G'):
                    // printf("Toupcam_get_RawFormat(%s, %d)\n", "GBRG", bitsperpixel);
                    bayerPattern_ = bayer_gr;
                    break;
                case MAKEFOURCC('R', 'G', 'G', 'B'):
                    // printf("Toupcam_get_RawFormat(%s, %d)\n", "RGGB", bitsperpixel);
                    bayerPattern_ = bayer_bg;
                    break;
                case MAKEFOURCC('B', 'G', 'G', 'R'):
                    // printf("Toupcam_get_RawFormat(%s, %d)\n", "BGGR", bitsperpixel);
                    bayerPattern_ = bayer_rg;
                    break;
                case MAKEFOURCC('G', 'R', 'B', 'G'):
                    // printf("Toupcam_get_RawFormat(%s, %d)\n", "GRBG", bitsperpixel);
                    bayerPattern_ = bayer_gb;
                    break;
                case MAKEFOURCC('Y', 'Y', 'Y', 'Y'):
                    // printf("Toupcam_get_RawFormat(%s, %d)\n", "YYYY", bitsperpixel);
                    break; // monochromatic sensor
                default:
                    break;
                }
        }

        /// stop the stream - once function ends callback will not be called any more
        virtual void stop_stream(CamErrorCode &e) override
        {
            cam_guard_type g(camera_lock_);
            // printf("\n\nstop_stream\n");
            if (stream_active_ == 0)
                return;
            {
                std::unique_lock<std::mutex> guard(lock_);
                callback_ = nullptr;
            }
            int hr = Toupcam_Stop(hcam_);
            if (FAILED(hr))
            {
                e = make_message("Failed to stop stream", hr);
                return;
            }
            stream_active_ = 0;
            /* 1 = hard flush, discard frames cached by camera DDR (if any)
               2 = soft flush, discard frames cached by toupcam.dll (if any)
               3 = both flush */
/*            hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_FLUSH, 3);
            if (FAILED(hr))
            {
                e = make_message("Toupcam_put_Option(TOUPCAM_OPTION_FLUSH, 3)", hr);
                return;
            }*/
        }

        /// list of camera controls that the camera supports
        virtual std::vector<CamOptionId> supported_options(CamErrorCode &) override
        {
            cam_guard_type g(camera_lock_);
            std::vector<CamOptionId> opts;
            // printf("\n\nsupported_options %llx\n", info_.model->flag);
            // Unconditionally supported
            opts.push_back(opt_auto_exp);
            opts.push_back(opt_gain);
            opts.push_back(opt_exp);
            // Conditionally supported
            if (!(info_.model->flag & TOUPCAM_FLAG_MONO)) {
#ifndef MEADECAM
                opts.push_back(opt_auto_wb);
#endif                
                opts.push_back(opt_wb);
            }
            if (info_.model->flag & TOUPCAM_FLAG_TEC) // Thermoelectric Cooler Present
            {
                opts.push_back(opt_cooler_power_perc);
                opts.push_back(opt_cooler_on);
                if (info_.model->flag & TOUPCAM_FLAG_TEC_ONOFF) // Thermoelectric Cooler can be turn on or off, support to set the target temperature of TEC 
                    opts.push_back(opt_cooler_target);
            }
            if (info_.model->flag & TOUPCAM_FLAG_GETTEMPERATURE) // Support to get the temperature of the sensor
                opts.push_back(opt_temperature);
            if (average_bin_support_ != NULL)
                opts.push_back(opt_average_bin);
            if (info_.model->flag & TOUPCAM_FLAG_BLACKLEVEL)
                opts.push_back(opt_black_level);
            if (info_.model->flag & TOUPCAM_FLAG_CGHDR)
                opts.push_back(opt_conv_gain_hdr); // 0-2
            else if (info_.model->flag & TOUPCAM_FLAG_CG)
                opts.push_back(opt_conv_gain_hcg); // 0-1
            if (info_.model->flag & TOUPCAM_FLAG_LOW_NOISE)
                opts.push_back(opt_low_noise);
            if (info_.model->flag & TOUPCAM_FLAG_HIGH_FULLWELL)
                opts.push_back(opt_high_fullwell);
            return opts;
        }
        /// get camera control
        virtual CamParam get_parameter(CamOptionId id, bool /*current_only*/, CamErrorCode &e) override
        {
            cam_guard_type g(camera_lock_);
            
            // printf("\n\nget_parameter: %s\n", cam_option_id_to_string_id(id).c_str());
            HRESULT hr;
            CamParam r = {};
            r.option = id;

            switch (id)
            {
            case opt_exp:
            {
                r.type = type_msec;
                unsigned nMin = 1;
                unsigned nMax = 2;
                unsigned nDef = 1;
                unsigned nCur = 1;
                if (FAILED(hr = Toupcam_get_ExpTimeRange(hcam_, &nMin, &nMax, &nDef)))
                {
                    e = make_message("Failed to Toupcam_get_ExpTimeRange()", hr);
                    return r;
                }
                r.min_val = nMin / TOUPCAM_2_OLS_TIME; // Toupcom API in microseconds, OLS API in ms
                r.max_val = nMax / TOUPCAM_2_OLS_TIME;
                r.def_val = nDef / TOUPCAM_2_OLS_TIME;
                if (FAILED(hr = Toupcam_get_ExpoTime(hcam_, &nCur)))
                {
                    e = make_message("Failed to Toupcam_get_ExpoTime()", hr);
                    return r;
                }
                r.cur_val = nCur / TOUPCAM_2_OLS_TIME;
                r.step_size = 1;
            }
            break;
            case opt_auto_exp:
            {
                r.type = type_bool;
                r.min_val = 0;
                r.max_val = 1;
                r.step_size = 1;
                int bAutoExp = 0;
                if (FAILED(hr = Toupcam_get_AutoExpoEnable(hcam_, &bAutoExp)))
                {
                    e = make_message("Failed to Toupcam_get_AutoExpoEnable()", hr);
                    return r;
                }
                r.cur_val = bAutoExp ? 1 : 0;
                r.def_val = r.cur_val;
                return r;
            }
            break;
            case opt_auto_wb:
            {
                r.type = type_bool;
                r.min_val = 0;
                r.max_val = 1;
                r.step_size = 1;
                r.cur_val = auto_wb_ ? 1 : 0;
                r.def_val = r.cur_val;
                return r;
            }
            break;
            case opt_wb:
            {
                int temp=6503,tint=1000;
                Toupcam_get_TempTint(hcam_,&temp,&tint);
                r.type = type_kelvin;
                r.min_val = 2000;
                r.max_val = 15000;
                r.step_size = 1;
                r.cur_val = temp;
                r.def_val = 6503;
                return r;
            }
            case opt_average_bin:
            {
                r.type = type_bool;
                r.min_val = 0;
                r.max_val = 1;
                r.step_size = 1;
                r.cur_val = average_bin_ ? 1 : 0;
                r.def_val = r.cur_val;
                return r;
            }
            break;
            case opt_black_level:
            {
                r.type = type_number;
                r.read_only = false;
                r.min_val = TOUPCAM_BLACKLEVEL_MIN;
                r.max_val = TOUPCAM_BLACKLEVEL8_MAX;
                if (info_.model->flag & TOUPCAM_FLAG_RAW10)
                    r.max_val = TOUPCAM_BLACKLEVEL10_MAX;
                if (info_.model->flag & TOUPCAM_FLAG_RAW12)
                    r.max_val = TOUPCAM_BLACKLEVEL12_MAX;
                if (info_.model->flag & TOUPCAM_FLAG_RAW14)
                    r.max_val = TOUPCAM_BLACKLEVEL14_MAX;
                if (info_.model->flag & TOUPCAM_FLAG_RAW16)
                    r.max_val = TOUPCAM_BLACKLEVEL16_MAX;
                r.step_size = 1;
                int bl;
                if (FAILED(hr = Toupcam_get_Option(hcam_, TOUPCAM_OPTION_BLACKLEVEL, &bl)))
                {
                    e = make_message("Failed to Toupcam_get_Option(TOUPCAM_OPTION_BLACKLEVEL)", hr);
                    return r;
                }
                r.cur_val = bl;
                r.def_val = TOUPCAM_BLACKLEVEL_MIN;
                return r;
            }
            case opt_conv_gain_hcg:
                return setCG(1, r, e);
            case opt_conv_gain_hdr:
                return setCG(2, r, e);
            case opt_cooler_power_perc:
            {
                r.type = type_percent;
                r.read_only = true;
                r.min_val = 0;
                r.max_val = 100.0;
                r.step_size = 1.0;
                r.cur_val = getTECVoltage(e);
                r.def_val = 0;
                // printf("FAN(0:%d)\n",info_.model->maxfanspeed);
                return r;
            }
            break;
            case opt_gain:
            {
                r.type = type_number;
                unsigned short nMin, nMax, nDef, nGain;
                if (FAILED(hr = Toupcam_get_ExpoAGainRange(hcam_, &nMin, &nMax, &nDef)))
                {
                    e = make_message("Failed to Toupcam_get_ExpoAGainRange", hr);
                    return r;
                }
                r.min_val = nMin;
                r.max_val = nMax;
                r.step_size = 1;
                r.def_val = nDef;
                if (FAILED(hr = Toupcam_get_ExpoAGain(hcam_, &nGain)))
                {
                    e = make_message("Failed to Toupcam_get_ExpoAGain", hr);
                    return r;
                }
                r.cur_val = nGain;
                return r;
            }
            break;
            case opt_temperature:
            {
                r.type = type_celsius;
                r.read_only = true;
                short temp = 320;
                if (FAILED(hr = Toupcam_get_Temperature(hcam_, &temp)))
                {
                    e = make_message("Failed to Toupcam_get_Temperature", hr);
                    return r;
                }
                // printf("Toupcam_get_Temperature = %d -> %d\n", hr, temp);

                r.cur_val = std::round(10.0 * temp * TOUPCAM_2_OLS_TEMP) / 10.0; // in 0.1℃ units (32 means 3.2℃, -35 means -3.5℃).
                r.def_val = 0;
            }
            break;
            case opt_cooler_target:
            {
                r.type = type_celsius;
                r.read_only = false;
                short temp = 1000;
                if (FAILED(hr = Toupcam_get_Temperature(hcam_, &temp)))
                {
                    e = make_message("Failed to Toupcam_get_Temperature", hr);
                    return r;
                }
                

                // values from old sdk
                r.min_val = -50; // Toupcam API in 0.1℃ units (32 means 3.2℃, -35 means -3.5℃)
                r.max_val = 40; 
                r.def_val = 0;
                r.cur_val = temp * TOUPCAM_2_OLS_TEMP;
                r.step_size = TOUPCAM_2_OLS_TEMP;
            }
            break;
            case opt_cooler_on:
            {
                r.type = type_bool;
                r.read_only = false;
                int cooler_on;
                if (FAILED(hr = Toupcam_get_Option(hcam_, TOUPCAM_OPTION_TEC, &cooler_on)))
                {
                    e = make_message("Failed to Toupcam_get_Option(TOUPCAM_OPTION_TEC)", hr);
                    return r;
                }

                r.min_val = 0;
                r.max_val = 1;
                r.def_val = 0;
                r.cur_val = cooler_on;
                r.step_size = 1;
            }
            break;
            case opt_low_noise:
            {
                r.type = type_bool;
                r.read_only = false;
                int low_noise;
                if (FAILED(hr = Toupcam_get_Option(hcam_, TOUPCAM_OPTION_LOW_NOISE, &low_noise)))
                {
                    e = make_message("Failed to Toupcam_get_Option(TOUPCAM_OPTION_LOW_NOISE)", hr);
                    return r;
                }

                r.min_val = 0;
                r.max_val = 1;
                r.def_val = 1;
                r.cur_val = low_noise;
                r.step_size = 1;
            }
            break;
            case opt_high_fullwell:
            {
                r.type = type_bool;
                r.read_only = false;
                int high_fullwell;
                if (FAILED(hr = Toupcam_get_Option(hcam_, TOUPCAM_OPTION_HIGH_FULLWELL, &high_fullwell)))
                {
                    e = make_message("Failed to Toupcam_get_Option(TOUPCAM_OPTION_HIGH_FULLWELL)", hr);
                    return r;
                }

                r.min_val = 0;
                r.max_val = 1;
                r.def_val = 1;
                r.cur_val = high_fullwell;
                r.step_size = 1;
            }
            break;
            default:
                ///
                /// we can't use cam_option_id_to_string_id I can't link with OLS because runtime of the library
                /// isn't clear
                /// e = make_message(cam_option_id_to_string_id(id), TOUPCAM_ERROR_INVALID_OPTION);
                e = make_message(std::to_string(int(id)), TOUPCAM_ERROR_INVALID_OPTION);
                break;
            }
            return r;
        }
        /// set camera control
        virtual void set_parameter(CamOptionId id, double value, CamErrorCode &e) override
        {
            // printf("\n\nset_parameter: %s -> %f\n", cam_option_id_to_string_id(id).c_str(), value);
            cam_guard_type g(camera_lock_);
            HRESULT hr = -1;

            switch (id)
            {
            case opt_auto_exp:
            {
                hr = Toupcam_put_AutoExpoEnable(hcam_, value ? 1 : 0);
            }
            break;
#ifndef MEADECAM            
            case opt_auto_wb:
            {
                auto_wb_ = !!value;
                // TODO: not yet tested, but it seems the callback dictates if Auto White balance is enabled
                hr = Toupcam_AwbOnce(hcam_, auto_wb_ ? AutoWB : NULL, auto_wb_ ? this : NULL);
            }
            break;
#endif            
            case opt_wb:
            {
                int temp = value;
                int tint = 1000; // default
                hr = Toupcam_put_TempTint(hcam_,temp,tint);
            }
            break;
            case opt_average_bin:
            {
                if (NULL == average_bin_support_)
                    break;
                average_bin_ = !!value;
                int binning = 0;
                hr = Toupcam_get_Option(hcam_, TOUPCAM_OPTION_BINNING, &binning);
                if (FAILED(hr))
                    break;
                if ((binning & ~AVERAGE_BINNING_FLAG) <= 1) // No binning (bin == 1), averaging is not supported
                    break;
                if (average_bin_) // Set
                    binning |= AVERAGE_BINNING_FLAG;
                else // Clean
                    binning &= ~AVERAGE_BINNING_FLAG;
                hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_BINNING, binning);
            }
            break;
            case opt_exp:
            {
                unsigned usTime = (unsigned)(value * 1000);
                hr = Toupcam_put_ExpoTime(hcam_, usTime);
            }
            break;
            // case opt_cooler_power_perc:
            // {
            //     int perCent = (int)(value);
            //     hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_FAN, perCent);
            // }
            // break;
            case opt_black_level:
            {
                int bl = (int)(value);
                hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_BLACKLEVEL, bl);
            }
            break;
            case opt_conv_gain_hcg:
            case opt_conv_gain_hdr:
            {
                int cg = (int)(value);
                hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_CG, cg);
            }
            break;
            case opt_gain:
            {
                unsigned short nGain = (unsigned short)value;
                hr = Toupcam_put_ExpoAGain(hcam_, nGain);
            }
            break;
            case opt_cooler_target:
            {
                short temp = (short)(value / TOUPCAM_2_OLS_TEMP);
                hr = Toupcam_put_Temperature(hcam_, temp);
                if (FAILED(hr))
                {
                    e = make_message("Toupcam_put_Temperature()", hr);
                    return;
                }
                hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_TEC, 1);
                if (FAILED(hr))
                {
                    e = make_message("Toupcam_put_Option(TOUPCAM_OPTION_TEC)", hr);
                    return;
                }
                if (info_.model->flag & TOUPCAM_FLAG_FAN)
                    hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_FAN, 1);
                if (FAILED(hr))
                {
                    e = make_message("Toupcam_put_Option(TOUPCAM_OPTION_FAN=1)", hr);
                    return;
                }
            }
            break;
            case opt_cooler_on:
            {
                hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_TEC, value ? 1 : 0);
                if (FAILED(hr))
                {
                    e = make_message("Toupcam_put_Option(TOUPCAM_OPTION_TEC)", hr);
                    return;
                }
                if (info_.model->flag & TOUPCAM_FLAG_FAN)
                    hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_FAN, value ? 1 : 0);
            }
            break;
            case opt_low_noise:
            {
                hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_LOW_NOISE, value ? 1 : 0);
                if (FAILED(hr))
                {
                    e = make_message("Toupcam_put_Option(TOUPCAM_OPTION_LOW_NOISE)", hr);
                    return;
                }
            }
            break;
            case opt_high_fullwell:
            {
                hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_HIGH_FULLWELL, value ? 1 : 0);
                if (FAILED(hr))
                {
                    e = make_message("Toupcam_put_Option(TOUPCAM_OPTION_HIGH_FULLWELL)", hr);
                    return;
                }
            }
            break;
            case opt_temperature:
            {
                short temp = (short)(value / TOUPCAM_2_OLS_TEMP);
                hr = Toupcam_put_Temperature(hcam_, temp);
            }
            break;
            default:
                e = "Unimplemented";
                return;
            }
            if (FAILED(hr))
            {
                e = make_message("Failed to set option", hr);
                return;
            }
        }

    private:
        void set_name()
        {
            name_ = convert(info_.displayname) + "/" + convert(info_.model->name);
        }
        /**
         * API does not provide info on types of binnig, so let's check it
         * 1. Save current binning
         * 2. Set TOUPCAM_OPTION_BINNING, 2 | AVERAGE_BINNING_FLAG 
         * 3. If attmpt was successful note it
         * 5. Restores current binnig
         */
        void checkAverageBinning(CamErrorCode &e)
        {
            if (NULL == hcam_)
            {
                e = make_message("Failed to open camera, hcam_ is NULL", TOUPCAM_ERROR_OPEN_BY_INFO);
                return;
            }

            int defaultBinValue = 1;
            int h = Toupcam_get_Option(hcam_, TOUPCAM_OPTION_BINNING, &defaultBinValue);
            if (FAILED(h))
            {
                e = make_message("Failed to Toupcam_get_Option(TOUPCAM_OPTION_BINNING)", h);
                return;
            }
            // In this way we can find both HW and SW bins
            // for (int i = 1; i < 128; ++i) // Assume no more than 127 ;-) bins
            // {
            //     h = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_BINNING, i);
            //     if (SUCCEEDED(h))
            //         bins_.push_back(i);
            //     else
            //         break;
            // }
            h = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_BINNING, 2 | AVERAGE_BINNING_FLAG); // 0x80 - Average flag, 2 - min bin with average flag set
            if (SUCCEEDED(h))
            {
                average_bin_support_ = &average_bin_;
                average_bin_ = false;
            }
            h = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_BINNING, defaultBinValue);
            if (FAILED(h))
            {
                e = make_message("Failed to Toupcam_put_Option(TOUPCAM_OPTION_BINNING, default)", h);
            }
        }
        /**
         * Rounds down image dimensions to an even number for binning > 1, see: toupcamsdk/doc/en.htm
         */
        static void updateWHByBin(int &width, int &height, int &binning)
        {
            if (binning <= 1)
                return;
            width /= binning;
            height /= binning;

            width -= width % 2; // The final image size is rounded down to an even number, such as 640/3 to get 212
            height -= height % 2;
        }
        /**
		 * Set center ROI
		 * Expectation: Toupcam_put_Size done before this method gets called
         */
        void setROI(CamStreamFormat sfmt, CamErrorCode &e)
        {
            int w,h,hr;
            
            if(sfmt.roi_den == 1) // ROI is not set
            {
                if (FAILED(hr = Toupcam_put_Roi(hcam_, 0, 0, 0, 0)))
                {
                    e = make_message("setROI: Failed to Toupcam_put_Roi(100%)", hr);
                }
                // printf("\n\nROI=(0,0,0,0)\n");
                return;
            }

            if(FAILED(hr = Toupcam_get_Size(hcam_, &w, &h)))
            {
                e = make_message("setROI: Failed to Toupcam_get_Size", hr);
                return;
            }

            // printf("\n\n\nsetROI: val= %f, roiW=%u, hoiH=%u\n", roiPercent_, );
            unsigned roiW = sfmt.width * sfmt.bin;
            if(roiW < ROI_MIN_WIDTH_HEIGHT)
                roiW = ROI_MIN_WIDTH_HEIGHT;
            unsigned roiH = sfmt.height * sfmt.bin;
            if(roiH < ROI_MIN_WIDTH_HEIGHT)
                roiH = ROI_MIN_WIDTH_HEIGHT;

            // xOffset, yOffset, xWidth, yHeight: must be even numbers
            roiW -= roiW % 2;
            roiH -= roiH % 2;
            unsigned roiX = (w - roiW) / 2;
            roiX -= roiX % 2;
            unsigned roiY = (h - roiH) / 2;
            roiY -= roiY % 2;

            // printf("\n\nROI=(%u,%u)-(%ux%u)\n", roiX, roiY, roiW, roiH);
            if(FAILED(hr = Toupcam_put_Roi(hcam_, roiX, roiY, roiW, roiH)))
            {
                e = make_message("setROI: Failed to Toupcam_put_Roi", hr);
                return;
            }
        }
        /**
         * https://www.cloudynights.com/topic/875680-openlivestacker-getting-ready-for-google-play-discussions/?p=12823060
         * "The HCG mode for my camera decreases the read noise at the cost of well capacity.
         * So HCG should be better for very short exposures.
         * LCG for long exposure. I assume with this phone app you would go for unguided short exposures, so maybe HCG should be the default"
        */
        CamParam setCG(double maxCG, CamParam r, CamErrorCode &e)
        {
            r.type = type_number;
            r.read_only = false;
            r.min_val = 0;
            r.max_val = maxCG;
            r.step_size = 1;
            int cg, hr;
            if (FAILED(hr = Toupcam_get_Option(hcam_, TOUPCAM_OPTION_CG, &cg)))
            {
                e = make_message("Failed to Toupcam_get_Option(TOUPCAM_OPTION_CG)", hr);
                return r;
            }
            r.cur_val = cg;
            r.def_val = CG_OLS_DEFAULT_VAL;
            return r;
        }
        double getTECVoltage(CamErrorCode &e)
        {
            struct timeval tv = {};
            gettimeofday(&tv, nullptr);
            // Please do not get this value too frequently, the recommended interval is 2 seconds or more
            if(tv.tv_sec - lastTECVoltageMeasure_ < 2l)
            {
                lastTECVoltageMeasure_ = tv.tv_sec;
                return lastTECVoltage_;
            }
            int hr, voltage;
            if (FAILED(hr = Toupcam_get_Option(hcam_, TOUPCAM_OPTION_TEC_VOLTAGE, &voltage)))
            {
                e = make_message("Failed to Toupcam_get_Option(TOUPCAM_OPTION_TEC_VOLTAGE)", hr);
            }
            return lastTECVoltage_ = std::round(1000.0 * voltage / TECMaxVoltage_) / 10.0; // one signigivant digit after decimal point
        }
        void init(CamErrorCode &e)
        {
            HRESULT hr;

            // Some cameras support the bit depth which is more than 8 such as 10, 12, 14, 16.
            // 0 = use 8 bits depth.
            // 1 = use the maximum bits depth of this camera.
            // Maximum bit depth seems a default choice for astroimageing
            if (FAILED(hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_BITDEPTH, 1)))
            {
                e = make_message("Failed to Toupcam_put_Option(TOUPCAM_OPTION_BITDEPTH, max)", hr);
                return;
            }
#ifndef MEADECAM
            unsigned maxTime;
            unsigned short maxAGain;
            if (FAILED(hr = Toupcam_get_MaxAutoExpoTimeAGain(hcam_, &maxTime, &maxAGain)))
            {
                e = make_message("Failed to Toupcam_get_MaxAutoExpoTimeAGain", hr);
                return;
            }
            // printf("init(maxTime=%u, &maxAGain=%u)\n", maxTime, maxAGain);
            unsigned minTime;
            unsigned short minAGain;
            if (FAILED(hr = Toupcam_get_MinAutoExpoTimeAGain(hcam_, &minTime, &minAGain)))
            {
                e = make_message("Failed to Toupcam_get_MinAutoExpoTimeAGain", hr);
                return;
            }
            // printf("init(minTime=%u, &minAGain=%u)\n", minTime, minAGain);
            maxTime = 5 * 1000 * 1000; // Default auto exposure time is ~350 ms, it looks too low for astro imaging
            if (FAILED(hr = Toupcam_put_MaxAutoExpoTimeAGain(hcam_, maxTime, maxAGain)))
            {
                e = make_message("Failed to Toupcam_put_MaxAutoExpoTimeAGain", hr);
                return;
            }
#endif            
            // Low noise seems a default choice for astroimageing
            if (((info_.model->flag & TOUPCAM_FLAG_LOW_NOISE)) && FAILED(hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_LOW_NOISE, 1)))
            {
                e = make_message("Failed to Toupcam_put_Option(TOUPCAM_OPTION_LOW_NOISE, 1)", hr);
                return;
            }
            // High Fullwell capacity seems a default choice for astroimageing
            if (((info_.model->flag & TOUPCAM_FLAG_HIGH_FULLWELL)) && FAILED(hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_HIGH_FULLWELL, 1)))
            {
                e = make_message("Failed to Toupcam_put_Option(TOUPCAM_OPTION_HIGH_FULLWELL, 1)", hr);
                return;
            }
            hr = Toupcam_put_Size(hcam_, info_.model->res[0].width, info_.model->res[0].height);
            if (FAILED(hr))
            {
                e = make_message("Failed to Toupcam_put_Size", hr);
                return;
            }
            set_name();
            checkAverageBinning(e);
            if (info_.model->flag & (TOUPCAM_FLAG_CG | TOUPCAM_FLAG_CGHDR))
            {
                if (FAILED(hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_CG, CG_OLS_DEFAULT_VAL)))
                {
                    e = make_message("Failed to Toupcam_put_Option(TOUPCAM_OPTION_CG, 1)", hr);
                    return;
                }
            }
            if (info_.model->flag & TOUPCAM_FLAG_TEC)
            {
                if (FAILED(hr = Toupcam_put_Option(hcam_, TOUPCAM_OPTION_TEC, 0)))
                {
                    e = make_message("Failed to Toupcam_put_Option(TOUPCAM_OPTION_TEC, 0)", hr);
                    return;
                }

                if (FAILED(hr = Toupcam_get_Option(hcam_, TOUPCAM_OPTION_TEC_VOLTAGE_MAX, &TECMaxVoltage_)))
                {
                    e = make_message("Failed to Toupcam_get_Option(TOUPCAM_OPTION_TEC_VOLTAGE_MAX)", hr);
                    return;
                }
                getTECVoltage(e);
            }
        }
        ToupcamDeviceV2 info_;
        HToupcam hcam_ = nullptr;
        std::string name_;
        bool average_bin_;
        bool *average_bin_support_ = NULL;
        std::vector<unsigned char> buf_;
        std::atomic<int> stream_active_;
        int frame_counter_ = 0;
        double lastTECVoltage_;
        int TECMaxVoltage_;
        time_t lastTECVoltageMeasure_ = 0;
        CamBayerType bayerPattern_ = bayer_na;
        bool auto_wb_ = false;
        // protected by mutex
        std::recursive_mutex camera_lock_;
        std::mutex lock_;
        frame_callback_type callback_;
    };
    /**
     * Toupcam_StartPullModeWithCallback: Handles TOUPCAM_EVENT_IMAGE - image is ready for Pull
     */
    void StartPullCallback(unsigned nEvent, void *pCallbackCtx)
    {
        ToupcamCamera *pToupcamCamera = reinterpret_cast<ToupcamCamera *>(pCallbackCtx);
        switch (nEvent)
        {
        case TOUPCAM_EVENT_IMAGE:
            pToupcamCamera->handle_frame();
            break;
        case TOUPCAM_EVENT_ERROR:
            pToupcamCamera->handle_error("Generic error");
            break;
        case TOUPCAM_EVENT_DISCONNECTED:
            pToupcamCamera->handle_error("Camera disconnected");
            break;
        case TOUPCAM_EVENT_NOFRAMETIMEOUT:
            pToupcamCamera->handle_error("No frame timeout error");
            break;
        case TOUPCAM_EVENT_NOPACKETTIMEOUT:
            pToupcamCamera->handle_error("No packet timeout");
            break;
        default:
            break;
        }
    }
    /**
     * Auto White Balance update callback
     */
    void AutoWB(const int /*nTemp*/, const int /*nTint*/, void *)
    {
        // printf("AutoWB Temp=%d, Tint=%d\n", nTemp, nTint);
    }
#ifndef _WIN32
    class SingleToupcamCameraDriver : public CameraDriver
    {
    public:
        /**
         * Loops over connected Toupcam cameras, prepares display name, and saves camera info
         */
        SingleToupcamCameraDriver(std::string const &id,std::string const &name) : id_(id),name_(name)
        {
        }
        virtual std::vector<std::string> list_cameras(CamErrorCode &)
        {
            std::string vp;
            if(id_.size()>=9)
                vp = "/" + id_.substr(id_.size()-9,4) + ":" + id_.substr(id_.size()-4);
            return {name_ + vp};
        }
        virtual std::unique_ptr<Camera> open_camera(int id, CamErrorCode &e)
        {
            // printf("\n\nSingleToupcamCameraDriver::open_camera: %d\n", id);
            std::unique_ptr<Camera> cam;
            if (id != 0)
            {
                e = "No such camera " + std::to_string(id);
                return cam;
            }
            cam = std::unique_ptr<Camera>(new ToupcamCamera(id_,name_,e));
            if (e)
                cam = nullptr;
            return cam;
        }

    private:
        std::string id_;
        std::string name_;
    };
#endif
    class ToupcamCameraDriver : public CameraDriver
    {
    public:

        ToupcamCameraDriver(bool oem=false):oem_(oem)
        {
        }

        /**
         * Loops over connected Toupcam cameras, prepares display name, and saves camera info
         * Returns: list of camera names
         */
#if defined MEADECAM || defined _WIN32
        int enumOEM(ToupcamDeviceV2 info[TOUPCAM_MAX]) { return 0; }
#else        
        int enumOEM(ToupcamDeviceV2 info[TOUPCAM_MAX])
        {
            int N = 0;
            FILE *f=popen(R"xxx(lsusb | sed 's/Bus \(.*\) Device \(.*\): ID \(....\):\(....\) \(.*\)/\1,\2,\3,\4,\5/i')xxx","r");
            char line[256];
            while(fgets(line,sizeof(line),f)) {
                int bus=-1,dev=-1;
                int vendor_id=-1,product_id=-1;
                int len = strlen(line);
                if(line[len-1]=='\n') {
                    line[len-1]=0;
                }
                if(!sscanf(line,"%d,%d,%x,%x",&bus,&dev,&vendor_id,&product_id))
                    break;
                std::string name = len >= (3*2+4*2+4) ? line +  (3*2+4*2+4) : "Camera";
                int tp_vid = vendor_id, tp_pid = product_id;
                oem_to_touptek(tp_vid,tp_pid);
                // non OEM - skip
                if(tp_vid == vendor_id && tp_pid == product_id)
                    continue;
                if(N>=TOUPCAM_MAX)
                    continue;
                snprintf(info[N].id,sizeof(info[N].id),"tp-%d-%d-%d-%d",bus,dev,tp_vid,tp_pid);
                snprintf(info[N].displayname,sizeof(info[N].displayname),"%s",name.c_str());
                info[N].model = Toupcam_get_Model(0x547,tp_pid);
                if(!info[N].model)
                    continue;
                N++;
            }
            pclose(f);
            return N;
        }
#endif
        virtual std::vector<std::string> list_cameras(CamErrorCode &e)
        {
            names_.clear();
            cams_.clear();
            ToupcamDeviceV2 info[TOUPCAM_MAX] = {};
            unsigned N = oem_ ? enumOEM(info) : Toupcam_EnumV2(info);
            for (unsigned i = 0; i < N; i++)
            {
                #ifdef _WIN32
                std::wstring wname = std::wstring(info[i].displayname) + L"/" + info[i].model->name + L"/" + std::to_wstring(i);
                std::string name = convert(wname); 
                #else
                std::string name = std::string(info[i].displayname) + "/" + info[i].model->name + "/" + std::to_string(i);
                #endif
                names_.push_back(name);
                cams_.push_back(info[i]);
            }
            return names_;
        }
        /**
         * Creates ToupcamCamera instance by camera info index
         */
        virtual std::unique_ptr<Camera> open_camera(int id, CamErrorCode &e)
        {
            // printf("\n\nToupcamCameraDriver::open_camera: %d\n", id);
            if (size_t(id) >= cams_.size())
            {
                e = "No such camera " + std::to_string(id);
                return std::unique_ptr<Camera>();
            }
            std::unique_ptr<Camera> cam(new ToupcamCamera(cams_.at(id), e));
            if (e)
                cam = nullptr;
            return cam;
        }

    private:
        std::vector<ToupcamDeviceV2> cams_;
        std::vector<std::string> names_;
        bool oem_;
    };
    namespace ToupDriverConfig {
        std::string driver_config;
    }
}

extern "C" {
#ifdef MEADECAM    
    int ols_set_meade_driver_config(char const *str)
#else    
    int ols_set_toup_driver_config(char const *str)
#endif    
    {
        ols::ToupDriverConfig::driver_config = str;
        return 0;
    }
#ifdef MEADECAM    
    void ols_set_meade_driver_log(char const *,int ) {}
#else    
    void ols_set_toup_driver_log(char const *log_path,int debug)
    {
        auto path = convert(log_path);
        Toupcam_log_File(path.c_str());
        Toupcam_log_Level(debug ? 4 : 1);
    }
#endif    
#ifdef MEADECAM    
    ols::CameraDriver *ols_get_meade_driver(int /*unused*/,ols::CamErrorCode *e)
#else    
    ols::CameraDriver *ols_get_toup_driver(int /*unused*/,ols::CamErrorCode *e)
#endif    
    {
        // const ToupcamModelV2** a = Toupcam_all_Model();
        // if(a)
        // {
        //     while(*a)
        //     {
        //         printf("%s\n",(*a)->name);
        //         ++(a);
        //     }
        // }
        if(ols::ToupDriverConfig::driver_config.empty()) {
            return new ols::ToupcamCameraDriver();
        }
        else if(ols::ToupDriverConfig::driver_config == "oem") {
            return new ols::ToupcamCameraDriver(true);
        }
        else {
#ifdef _WIN32            
            *e = "Unsupported";
            return nullptr;
#else           
            size_t split_point = ols::ToupDriverConfig::driver_config.find(':');
            std::string id = ols::ToupDriverConfig::driver_config.substr(0,split_point);
            int fd=-1,vendor_id=0,product_id=0;
            if(sscanf(id.c_str(),"%d %x %x",&fd,&vendor_id,&product_id)!=3) {
                *e = ols::CamErrorCode("Invalid Android string:" + ols::ToupDriverConfig::driver_config);
                return nullptr;
            }

            // handle OEM cameras replace OEM to Topu product/vendor id
            ols::oem_to_touptek(vendor_id,product_id);

            char toup_id[256];
            // touptek android format
            snprintf(toup_id,sizeof(toup_id),"fd-%d-%04x-%04x",fd,vendor_id,product_id);
            std::string name = split_point == std::string::npos ? "Camera" : ols::ToupDriverConfig::driver_config.substr(split_point+1);
            return new ols::SingleToupcamCameraDriver(toup_id,name);
#endif            
        }
    }
}
