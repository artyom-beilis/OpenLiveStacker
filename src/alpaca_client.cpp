#include "alpaca_client.h"
#include "httplib_for_ols.h"
//#include <booster/posix_time.h>
#include <stdexcept>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <booster/aio/endpoint.h>
#include <booster/aio/basic_socket.h>

#ifdef _WIN32
# include <winsock2.h>
# include <ws2tcpip.h>
# include <iphlpapi.h>
#else
# include <unistd.h>
# include <arpa/inet.h>
# include <sys/socket.h>
# include <ifaddrs.h>
# include <netinet/in.h>
# include <net/if.h>
#endif


namespace ols {
    AlpacaClient::AlpacaClient(std::string const &url,std::string const &type)
    {
        std::string addr;
        if(url == "auto")
            base_url_ = discover(type);
        else
            base_url_ = url;
        client_.reset(new httplib::Client(base_url_));
		client_->set_connection_timeout(2,0);
		client_->set_read_timeout(2,0);
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
        auto result = client_->Get(prefix_ + "/imagearray",std_params(),
            {
                {"Accept","application/imagebytes"},
                {"Accept-Encoding", "identity"}
                // ^^^ identity is not really needed 
                // just a workaround for ASCOM Remote 6.7 bug #71
            } 
        );
        if(!result)
            throw std::runtime_error("Failed connect to " + prefix_ + "/imagearray");
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
        if(!res)
            throw std::runtime_error("Failed connect to " + url);
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
        auto full_params = std_params();
        for(auto p: params)
            full_params.insert(p);
        auto res = client_->Get(url,full_params,httplib::Headers());
        if(!res)
            throw std::runtime_error("Failed connect to " + url);
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
    
    namespace {
#ifdef _WIN32
        std::vector<std::string> list_bcast_addresses()
        {
            IP_ADAPTER_ADDRESSES *addresses = nullptr, *addr = nullptr;
            ULONG outBufLen = 15000;
            addresses = (IP_ADAPTER_ADDRESSES *)malloc(outBufLen);
            if (GetAdaptersAddresses(AF_INET, 
                        GAA_FLAG_SKIP_ANYCAST |
                        GAA_FLAG_SKIP_MULTICAST |
                        GAA_FLAG_SKIP_DNS_SERVER,
                        nullptr,
                        addresses,
                        &outBufLen) != NO_ERROR) 
            {
                free(addresses);
                throw std::runtime_error("Failed to get adapter addresses");
            }
            std::vector<std::string> ips;

            for (addr = addresses; addr != nullptr ; addr = addr->Next) {
                if (addr->OperStatus != IfOperStatusUp) 
                    continue;
                IP_ADAPTER_UNICAST_ADDRESS *unicast = addr->FirstUnicastAddress;
                if (unicast && unicast->Address.lpSockaddr->sa_family == AF_INET) {
                    struct sockaddr_in *sa = (struct sockaddr_in*)unicast->Address.lpSockaddr;
                    DWORD mask;
                    ConvertLengthToIpv4Mask(unicast->OnLinkPrefixLength, &mask);
                    uint32_t ip = (sa->sin_addr.s_addr);
                    uint32_t broadcast = (ip & mask) | (~mask);
                    struct in_addr bcast_addr;
                    bcast_addr.s_addr = (broadcast);
                    char buf[INET_ADDRSTRLEN + 1] = {};
                    inet_ntop(AF_INET, &bcast_addr, buf, INET_ADDRSTRLEN);
                    fprintf(stderr,"Using %s\n",buf);
                    ips.push_back(buf);
                }
            }
            free(addresses);
            return ips;
        }
#else        
        std::vector<std::string> list_bcast_addresses()
        {
            struct ifaddrs *ifaddr = nullptr;
            if(getifaddrs(&ifaddr) != 0)  {
                throw std::runtime_error("Failed to get list of interfaces - autodetection failed");
            }
            std::vector<std::string> ips;

            for(ifaddrs *p = ifaddr; p ; p = p->ifa_next) {
                if(p->ifa_addr == nullptr || p->ifa_addr->sa_family != AF_INET || !(p->ifa_flags & IFF_UP))
                    continue;
                auto addr = (struct sockaddr_in *)p->ifa_addr;
                auto netmask = (struct sockaddr_in *)p->ifa_netmask;
                unsigned ip = addr->sin_addr.s_addr;
                unsigned mask = netmask->sin_addr.s_addr;
                unsigned broadcast = (ip & mask) | (~mask);
                
                struct in_addr bcast_addr;
                bcast_addr.s_addr = broadcast;
                char buf[INET_ADDRSTRLEN + 1] = {};
                inet_ntop(AF_INET, &bcast_addr, buf, INET_ADDRSTRLEN);
                fprintf(stderr,"Using %s\n",buf);
                ips.push_back(buf);
            }		
            freeifaddrs(ifaddr);
            ifaddr = nullptr;
            return ips;
        }
#endif        
        std::set<std::string> discover_ips(std::vector<std::string> const &baddr)
        {
            std::set<std::string> result;
            booster::aio::basic_socket s;
            s.open(booster::aio::pf_inet,booster::aio::sock_datagram);
            int enable = 1;
            if(setsockopt(s.native(),SOL_SOCKET,SO_BROADCAST,(char const *)&enable,sizeof(enable)) < 0)
                throw std::system_error(errno, std::generic_category()); 
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 500000;
            if(setsockopt(s.native(),SOL_SOCKET,SO_RCVTIMEO,(char const *)&tv,sizeof(tv)) < 0) {
                throw std::system_error(errno, std::generic_category()); 
            }
            std::string message = "alpacadiscovery1";
            for(std::string const &target_ip :baddr) {
                booster::aio::endpoint ep(target_ip,32227);
                auto addr = ep.raw();
                for(int attempts = 0; attempts < 2; attempts ++) {
                    if(sendto(s.native(),message.c_str(),message.size(),0,addr.first,addr.second) < 0) {
                        throw std::system_error(errno, std::generic_category()); 
                    }
                    char buffer[256] = {};
                    struct sockaddr_in sender;
                    socklen_t len = sizeof(sender);
                    if(recvfrom(s.native(),buffer,sizeof(buffer) - 1,0,(struct sockaddr *)&sender,&len) > 0) {
                        booster::aio::endpoint remote_ep;
                        remote_ep.raw((struct sockaddr *)&sender,len);
                        std::string remote_ip = remote_ep.ip();
                        std::istringstream ss(buffer);
                        cppcms::json::value v;
                        if(v.load(ss,true)) {
                            int port = v.get("AlpacaPort",-1);
                            if(port != -1) {
                                std::string addr = "http://" + remote_ip + ":" + std::to_string(port);
                                fprintf(stderr,"Found Alpaca at %s\n",addr.c_str());
                                result.insert(addr);
                            }
                        }
                    }
                }
            }
            return result;
        }
    }
    std::string AlpacaClient::discover(std::string const &type)
    {
        auto nets = list_bcast_addresses();
        auto ips = discover_ips(nets);
        for(auto const &ip: ips) {
            try {
                AlpacaClient cl(ip,type);
                auto res = cl.list_devices();
                if(!res.empty()) {
                    fprintf(stderr,"Found %s at %s done\n",type.c_str(),ip.c_str());
                    return ip;
                }
            }
            catch(std::exception const &) {
            }
        }
        throw std::runtime_error("Failed to discover Alpaca Server");
    }
    
} // ols
