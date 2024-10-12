#pragma once
#include "camera.h"
#include <regex>

namespace ols {
        inline bool parse_indigo_format(std::string const &name,std::string const &label,CamStreamFormat &fmt,int &bits)
        {
            fmt = CamStreamFormat();
            static std::regex name_v1("(RAW|RGB|MON|Y|BIN)(\\d*)_(\\d+)");
            static std::regex name_v2("(RAW|RGB|MON|Y|BIN)\\s+(\\d+)\\s*(\\d+)x(\\d+)");
            static std::regex name_v3("BIN_(\\d+)x(\\d+)");
            static std::regex name_v4("0_\\d+x\\d+");
            static std::regex label_v1("(MON|RAW|RGB|Y)\\s+(\\d+)\\s+(\\d+)\\s*x\\s*(\\d+)\\s*");
            static std::regex label_v2("YUV\\s+(\\d+)\\s*x\\s*(\\d+)\\s*");
            std::smatch m;
            int bin1=1,bin2=1;
            bits = -1;
            int width=-1,height=-1;
            std::string format;
            if(std::regex_match(name,m,name_v1)) {
                std::string sformat = m[1].str();
                if(sformat != "BIN")
                    format = sformat;
                bits = atoi(m[2].str().c_str());
                bin1 = bin2 = atoi(m[3].str().c_str());
            }
            else if(std::regex_match(name,m,name_v2)) {
                std::string sformat = m[1].str();
                if(sformat != "BIN")
                    format = sformat;
                bits = atoi(m[2].str().c_str());
                bin1 = atoi(m[3].str().c_str());
                bin2 = atoi(m[4].str().c_str());
            }
            else if(std::regex_match(name,m,name_v3)) {
                bin1 = atoi(m[1].str().c_str());
                bin2 = atoi(m[2].str().c_str());
            }
            else if(std::regex_match(name,m,name_v4)) {
                bin1=bin2=1;
            }

            if(std::regex_match(label,m,label_v1)) {
                std::string sformat = m[1].str();
                if(bits < 0)
                    bits = atoi(m[2].str().c_str());
                width = atoi(m[3].str().c_str());
                height = atoi(m[4].str().c_str());
            }
            else if(std::regex_match(label,m,label_v2)) {
                format = "YUV2";
                bits = 8;
                width  = atoi(m[1].str().c_str());
                height = atoi(m[2].str().c_str());
            }
            if(bits < 8 || width <= 0 || height <=0 || format=="")
                return false;
            
            if(format == "RAW") {
                if(bits == 8)
                    fmt.format =  stream_raw8;
                else
                    fmt.format = stream_raw16;
            }
            else if(format == "Y" || format == "MON") {
                if(bits == 8)
                    fmt.format =  stream_mono8;
                else
                    fmt.format = stream_mono16;
            }
            else if(format == "RGB") {
                if(bits == 24)
                    fmt.format = stream_rgb24;
                else if(bits == 48)
                    fmt.format = stream_rgb48;
            }
            else if(format == "YUV2") {
                fmt.format = stream_rgb24; // fits supports RGB
            }
            else
                return false;
            if(bin1 != bin2)
                return false;
            fmt.bin = bin1;
            fmt.width = width;
            fmt.height = height;
            return true;
        }
}
