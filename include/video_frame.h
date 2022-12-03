#pragma once
namespace ols {

    class VideoFrame {
    public:
        VideoFrame(void const *data,size_t len) 
        {
            buffer_ = malloc(len);
            if(!buffer_)
                throw std::bad_alloc();
            memcpy(buffer_,data,len);
            buffer_size_ = len;
        }
        
        VideoFrame(VideoFrame const &) = delete;
        VideoFrame(VideoFrame &&) = delete;
        
        void operator=(VideoFrame const &) = delete;
        void operator=(VideoFrame &&) = delete;

        void *data() const { return buffer_; }
        size_t size() const { return buffer_size_; }

        ~VideoFrame()
        {
            free(buffer_);
        }
    private:
        void *buffer_;
        size_t buffer_size_;
    };

    inline std::ostream &operator<<(std::ostream &out,VideoFrame const &frame)
    {
        out.write(static_cast<char *>(frame.data()),frame.size());
        return out;
    }

}
