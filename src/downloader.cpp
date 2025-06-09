#include "downloader.h"

#include <string.h>
#include "zlib.h"
#include <stdint.h>

#ifdef WITH_CURL            
#include <curl/curl.h>
#elif defined(ANDROID_SUPPORT)
extern "C" {
    typedef int (*ols_external_downloader_type)(char const *url,void *callback_cookie,char *error_message_buffer,int error_message_buffer_size);
    static ols_external_downloader_type ols_downloader = nullptr;
    void ols_set_external_downloader(ols_external_downloader_type downloader)
    {
        ols_downloader = downloader;
    }
}

#endif


namespace ols {

    class UnZipper {
    public:
        bool ok = true;

        enum State {
            reading_header,
            reading_fname,
            reading_body,
            reading_rest,
        }; 

        struct __attribute__((__packed__)) Header {
            uint32_t signature;
            uint16_t version;
            uint16_t flags;
            uint16_t compression;
            uint16_t ltime,ldate;
            uint32_t crc;
            uint32_t size_compressed;
            uint32_t size_uncompressed;
            uint16_t fname_length;
            uint16_t extra_length;
        };

        char const *get_error() noexcept
        {
            return error_;
        }

        UnZipper(std::string const &out_dir,std::function<bool(char const *)> const &new_file_callback) :
            output_dir_(out_dir),
            callback_(new_file_callback)
        {
        }

        ~UnZipper()
        {
            if(buffer_)
                free(buffer_);
        }
        
        bool consume(char *src,size_t n) noexcept
        {
            if(state_ == reading_rest) {
                return true;
            }
            while(n > 0) {
                char *tgt = state_ == reading_header ? (char*)&header_ : buffer_;
                if(tgt == nullptr) {
                    error_ = "INTERNAL ERROR";
                    return false;
                }
                tgt += read_;
                while(read_ < total_ && n > 0) {
                    *tgt++ = *src++;
                    n--;
                    read_ ++;
                }
                if(read_ == total_) {
                    switch(state_) {
                    case reading_header:    
                        if(!handle_header()) {
                            return false;
                        }
                        break;
                    case reading_fname:
                        if(!handle_file_name()){
                            return false;
                        }
                        break;
                    case reading_body:
                        if(!handle_body()) {
                            return false;
                        }
                        break;
                    case reading_rest:
                        return true;
                    }
                }
            }
            return true;
        }
        
        bool alloc(size_t n) noexcept
        {
            if(buffer_size_ >= n)
                return true;
            void *new_ptr = realloc(buffer_,n + 1);
            if(!new_ptr) {
                error_ = "allocation failed";
                return false;
            }
            buffer_ = static_cast<char *>(new_ptr);
            return true;
        }

        bool handle_header() noexcept
        {
            if(header_.signature == 0x02014b50 || header_.signature == 0x06054b50) {
                // directory
                state_ = reading_rest;
                return true;
            }
            if(header_.signature != 0x04034b50) { 
                error_ = "Invalid header";
                return false;
            }
            if(header_.flags & 1) {
                error_ = "Encryption unsuppored";
                return false;
            }
            if(header_.flags & 8) {
                error_ = "ZIP descriptor unsuppored";
                return false;
            }
            state_ = reading_fname;
            read_ = 0;
            total_ = header_.fname_length + header_.extra_length;
            if(!alloc(total_))
                return false;
            return true;
        }
        bool handle_file_name() noexcept
        {
            buffer_[header_.fname_length] = 0;
            try {
                if(!callback_(buffer_)) {
                    error_ = "canceled";
                    return false;
                }
            }
            catch(...) {
                error_ = "callback failed";
                return false;
            }
            state_ = reading_body;
            read_ = 0;
            total_ = header_.size_compressed;
            if(!alloc(header_.size_compressed))
                return false;
            snprintf(fname_,sizeof(fname_),"%s/%s",output_dir_.c_str(),buffer_);
            return true;
        }

        bool save_file() noexcept
        {
            header_.size_uncompressed = header_.size_compressed;
            if(header_.size_compressed != header_.size_uncompressed) {
                error_ = "Invalid stored size";
                return false;
            }
            FILE *f = fopen(fname_,"wb");
            if(!f) {
                error_ = "Failed to open file";
                return false;
            }
            if(fwrite(buffer_,1,header_.size_uncompressed,f) != header_.size_uncompressed) {
                error_ = "Failed to write file";
                fclose(f);
                return false;
            }
            bool close_ok = fclose(f) == 0;
            f=nullptr;
            if(close_ok)
                return true;
            error_ = "Failed to close file";
            return false;
        }

        bool inflate_file() noexcept
        {
            z_stream strm;
            memset(&strm,0,sizeof(strm));
            FILE *f = fopen(fname_,"wb");
            bool end_needed=false;
            int ret;
            size_t n;
            if(!f) {
                error_ = "Failed to open file";
                goto on_error;
            }
            end_needed = true;
            if(inflateInit2(&strm,-MAX_WBITS)!=Z_OK) {
                error_ = "Failed to init zlib";
                goto on_error;
            }

            strm.avail_in = header_.size_compressed;
            strm.next_in = (unsigned char *)(buffer_);
            do {
                strm.avail_out = sizeof(zbuf_);
                strm.next_out = zbuf_;
                ret = inflate(&strm, Z_NO_FLUSH); 
                n = sizeof(zbuf_) - strm.avail_out;
                switch(ret) {
                case Z_NEED_DICT:
                case Z_DATA_ERROR:
                case Z_MEM_ERROR:
                    error_ = "zlib Data Error";
                    goto on_error;
                };
                if(n > 0) {
                    if(fwrite(zbuf_,1,n,f) != n) {
                        error_ = "Failed write to file";
                        goto on_error;
                    }
                }
            } while(strm.avail_out == 0);
            if(ret != Z_STREAM_END) {
                error_ = "Decompression failed";
                goto on_error;
            }
            inflateEnd(&strm);
            end_needed = false;
            if(fclose(f) == 0) {
                return true;
            }
            f = nullptr;
            error_ = "Closing failed";
        on_error:
            if(f) {
                fclose(f);
            }
            if(end_needed)
                inflateEnd(&strm);
            return false;
        }

        bool handle_body() noexcept
        {
            bool status;
            if(header_.compression == 8) {
                status = inflate_file();
            }
            else if (header_.compression == 0) {
                status = save_file();
            }
            else {
                snprintf(fname_,sizeof(fname_),"Unsupported compression method %d\n",header_.compression);
                error_ = fname_;
                status = false;
            }
            state_ = reading_header;
            read_ = 0;
            total_ = sizeof(header_);
            return status;
        }
        size_t input(void *data,size_t size,size_t nmemb) noexcept
        {
            if(consume(static_cast<char *>(data),size*nmemb))
                return nmemb;
            ok = false;
            return 0;
        }

    private:
        State state_ = reading_header;
        Header header_;
        char *buffer_ = nullptr;
        size_t buffer_size_ = 0;
        char fname_[256];
        unsigned char zbuf_[16384];

        size_t read_ = 0,total_ = sizeof(Header);
        std::string output_dir_;
        std::function<bool(char const *)> callback_;
        char const *error_ = "Unknown error";
    };

    extern "C" {
        inline size_t ols_downloader_write_data(void *ptr, size_t size, size_t nmemb, void *stream) 
        {
            return static_cast<UnZipper*>(stream)->input(ptr,size,nmemb);
        }
#ifdef ANDROID_SUPPORT
        int ols_downloader_jna_write(void *cookie,char *buffer, int size) 
        {
            return ols_downloader_write_data(buffer,1,size,cookie);
        }
#endif        
                
    }
        

    bool zip_download(std::string const &url,std::string const &target_dir,std::string &error_message,std::function<bool(char const *)> new_file_callback)
    {
        UnZipper unzipper(target_dir,new_file_callback);
        int res = 0;
        if(url.substr(0,5)=="file:") {
            FILE *f=fopen(url.substr(5).c_str(),"rb");
            if(!f) {
                error_message = "Failed to open " + url;
                return false;
            }
            char buf[4096];
            size_t n;
            while((n=fread(buf,1,sizeof(buf),f))>0) {
                if(unzipper.input(buf,1,n)!=n) {
                    break;
                }
            }
            fclose(f);
            res = 0;
        }
        else {
            #ifdef WITH_CURL            
            CURL *curl = curl_easy_init();
            curl_easy_setopt(curl,CURLOPT_URL,url.c_str());
            curl_easy_setopt(curl,CURLOPT_WRITEDATA,&unzipper);
            curl_easy_setopt(curl,CURLOPT_WRITEFUNCTION,ols_downloader_write_data);
            curl_easy_setopt(curl,CURLOPT_SSL_VERIFYPEER,0l);
            curl_easy_setopt(curl,CURLOPT_FOLLOWLOCATION,1l);
            CURLcode res = curl_easy_perform(curl);
            if(res != 0) {
                error_message = curl_easy_strerror(res);
            }
            curl_easy_cleanup(curl);
            curl = 0;
            #elif defined(ANDROID_SUPPORT)
            if(ols_downloader == nullptr) {
                error_message = "Internal error: downloader was not configured properly";
                return false;    
            }
            char error_message_buffer[1024] ={};
            res = ols_downloader(url.c_str(),&unzipper,error_message_buffer,sizeof(error_message_buffer));
            if(res != 0) {
                error_message = error_message_buffer;
            }
            #else
            error_message = "OpenLiveStacker was build without download support, CURL needed";
            return false;
            #endif
            if(!unzipper.ok) {
                error_message = unzipper.get_error();
                return false;
            }
        }
        return res == 0;
    }
}


