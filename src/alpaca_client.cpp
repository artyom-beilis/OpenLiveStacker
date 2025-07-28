#include "alpaca_client.h"
#include "httplib_for_ols.h"
//#include <booster/posix_time.h>
#include <stdexcept>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace ols {
    AlpacaClient::AlpacaClient(std::string const &url,std::string const &type)
    {
        client_.reset(new httplib::Client(url));
        client_id_ = double(rand()) / RAND_MAX * 65534 + 1;
        device_type_ = to_lower(type);
    }
    AlpacaClient::~AlpacaClient()
    {
    }
    void AlpacaClient::set_device(int id)
    {
        device_no_ = id;
        prefix_ = "/api/v1/" + device_type_ + "/" + std::to_string(device_no_);
    }
    std::vector<std::pair<std::string,int> > AlpacaClient::list_devices()
    {
        std::vector<std::pair<std::string,int> > result;
        cppcms::json::value devices=get_json("/management/v1/configureddevices");
        auto items = devices["Value"].array();
        for(cppcms::json::value const &item : items) {
            if(to_lower(item.get<std::string>("DeviceType")) != device_type_)
                continue;
            result.push_back(std::make_pair(item.get<std::string>("DeviceName"),item.get<int>("DeviceNumber")));
        }
        return result;
    }
    struct AlpacaImageMeta {
        int32_t MetadataVersion; // Bytes 0..3 - Metadata version = 1
        int32_t ErrorNumber; // Bytes 4..7 - Alpaca error number or zero for success
        uint32_t ClientTransactionID; // Bytes 8..11 - Client's transaction ID
        uint32_t ServerTransactionID; // Bytes 12..15 - Device's transaction ID
        int32_t DataStart; // Bytes 16..19 - Offset of the start of the data bytes
        int32_t ImageElementType; // Bytes 20..23 - Element type of the source image array
        int32_t TransmissionElementType; // Bytes 24..27 - Element type as sent over the network
        int32_t Rank; // Bytes 28..31 - Image array rank (2 or 3)
        int32_t Dimension1; // Bytes 32..35 - Length of image array first dimension
        int32_t Dimension2; // Bytes 36..39 - Length of image array second dimension
        int32_t Dimension3; // Bytes 40..43 - Length of image array third dimension (0 for 2D array)
    };
    void AlpacaClient::get_binary_image(cv::Mat &output)
    {
        auto result = client_->Get(prefix_ + "/imagearray",{{"Accept","application/imagebytes"}});
        if(result->status != 200)
            throw std::runtime_error("Failed get image array statis = " + std::to_string(result->status));
        if(result->get_header_value("Content-Type") != "application/imagebytes")
            throw std::runtime_error("We can't handle json image data");
        if(result->body.size() < sizeof(AlpacaImageMeta))
            throw std::runtime_error("Body is too small");
        AlpacaImageMeta meta;
        memcpy(&meta,&result->body[0],sizeof(meta));
        int bpp;
        if(meta.TransmissionElementType == 6)
            bpp = 1;
        else if(meta.TransmissionElementType == 8)
            bpp = 2;
        else
            throw std::runtime_error("Elemented type = " + std::to_string(meta.TransmissionElementType) + " is not supported");
        int height = meta.Dimension1;
        int width = meta.Dimension2;
        int rgb = meta.Dimension3;
        if(log_){
            fprintf(log_,"Got image with %d bpp, %dx%dx%d\n",bpp,height,width,rgb);
        }
        if(rgb != 0 && rgb != 3)
            throw std::runtime_error("Unsuppoted image format either mono or rgb!");
        size_t start = meta.DataStart;
        size_t size = width * height * (rgb ? 3 : 1) * bpp;
        size_t end = start + size;
        if(start < sizeof(meta) || size >= end || end > result->body.size() || size == 0)
            throw std::runtime_error("Image format mistmatch");
        void *ptr = &result->body[start];
        cv::Mat orig(height,width,(bpp == 1 ? (rgb ? CV_8UC3 : CV_8UC1) : (rgb ? CV_16UC3 : CV_16UC1)),ptr);
        cv::Mat tr = orig.t();
        if(bpp == 1) {
            tr.convertTo(output,(rgb ? CV_16UC3 : CV_16UC1));
        }
        else {
            tr.copyTo(output);
        }
        if(rgb == 3) {
            cv::cvtColor(output,output,cv::COLOR_RGB2BGR);
        }
    }

    cppcms::json::value AlpacaClient::get(std::string const &url)
    {
        return get(url,{});
    }
    cppcms::json::value AlpacaClient::get(std::string const &url,std::multimap<std::string,std::string> const &params)
    {
        if(device_no_ == -1)
            throw std::runtime_error("Device is not selected");
        cppcms::json::value v = get_json(prefix_ + url,params);
        int err = v.get<int>("ErrorNumber");
        if(err != 0) {
            throw std::runtime_error("Error for " + url + " code=" + std::to_string(err) + " message=" + v.get("ErrorMessage","unknown"));
        }
        return v;
    }

    std::string AlpacaClient::to_lower(std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(), [](char c) -> char { 
            if('A'<=c && c<='Z')
                return c-('Z' - 'z');
            return c;
        });
        return s;
    }

    void AlpacaClient::put(std::string const &where,std::multimap<std::string,std::string> const &extra_params)
    {
        auto par = std_params();
        par.insert(extra_params.begin(),extra_params.end());
        std::string url = prefix_ + where;
        auto res = client_->Put(url,par);
        if(res->status !=200) {
            throw std::runtime_error("Failed to put for " + url + " status " + std::to_string(res->status));
        }
        cppcms::json::value v;
        int lineno = -1;
        char const *body_ptr = res->body.c_str();
        if(!v.load(body_ptr,res->body.c_str() + res->body.size(),true,&lineno)) {
            throw std::runtime_error("Response ro  " + url + " is not json in line " + std::to_string(lineno));
        }
        int err = v.get<int>("ErrorNumber");
        if(err != 0) {
            throw std::runtime_error("Error for " + url + " code=" + std::to_string(err) + " message=" + v.get("ErrorMessage","unknown"));
        }
        if(log_) { 
            fprintf(log_," put to %s\n",url.c_str());
            for(auto const &p:par)
                fprintf(log_,"     %s=%s\n",p.first.c_str(),p.second.c_str());
            fprintf(log_,"   res %s\n",res->body.c_str());
        }
    }            

    bool AlpacaClient::is_connected()
    {
        auto v = get("/connected");
        return v.get<bool>("Value");
    }

    void AlpacaClient::connect()
    {
        if(is_connected())
            return;
        put("/connected",{{"Connected","true"}});
        if(!is_connected()) 
            throw std::runtime_error("Failed to connect");
    }
    
    void AlpacaClient::set_logf(FILE *f)
    {
        log_ = f;
    }

    cppcms::json::value AlpacaClient::get_json(std::string const &url,std::multimap<std::string,std::string> const &params)
    {
        auto res = client_->Get(url,params,httplib::Headers());
        if(res->status != 200)
            throw std::runtime_error("Failed status on " + url + " got " + std::to_string(res->status));
        if(log_) {
            std::ostringstream ps;
            if(!params.empty())
                ps << "?";
            for(auto kv:params)
                ps << kv.first <<"=" << kv.second << "&";
            fprintf(log_," get %s\n    %s\n",(url + ps.str()).c_str(),res->body.c_str());
        }
        cppcms::json::value v;
        int lineno = -1;
        char const *body_ptr = res->body.c_str();
        if(!v.load(body_ptr,res->body.c_str() + res->body.size(),true,&lineno))
            throw std::runtime_error("Response ro  " + url + " is not json in line " + std::to_string(lineno));
        return v;
    }
    std::multimap<std::string, std::string> AlpacaClient::std_params()
    {
        transaction_id_ ++;
        return {{std::string("ClientID"),std::to_string(client_id_)},{std::string("ClientTransactionID"),std::to_string(transaction_id_)}};
    }
    
} // ols
