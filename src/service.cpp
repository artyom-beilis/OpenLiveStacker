#include <cppcms/application.h>

#include "video_frame.h"

class VideoStream : public cppcms::application {
public:
    void update(std::shared_ptr<VideoFrame> frame)
    {
        {
            std::unique_lock<std::mutex> guard(frame_lock_);
            current_frame_ = frame;
        }
        service().post([]() { this->broadcast(); });
    }
    VideoStream(cppcms::service &srv) : cppcms::application(srv)
    {
        
    }
private:
    std::mutex frame_lock_;
    std::shared_ptr<VideoFrame> current_frame_;
    std::set<std::shared_ptr<cppcms::http::context> > streams_;

};


int main(int argc,char **argv)
{

}
