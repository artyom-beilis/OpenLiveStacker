#include "common_utils.h"
#include <opencv2/imgcodecs.hpp>
#include "data_items.h"

namespace ols {
    std::shared_ptr<CameraFrame> generate_dummy_frame(int w,int h,int c,int red,int green, int blue)
    {
        std::shared_ptr<CameraFrame> frame(new CameraFrame());
        frame->format.width = w;
        frame->format.height = h;
        frame->format.format = stream_mjpeg;
        cv::Mat img(h,w,c == 3 ? CV_8UC3: CV_8UC1);
        unsigned char *data = img.data;
        if(c == 3) {
            for(int r=0;r<h;r++) {
                for(int c=0;c<w;c++) {
                    int color = (((r / 8) ^ (c / 8)) & 1);
                    *data++ = color * red;
                    *data++ = color * green;
                    *data++ = color * blue;
                }
            }
        }
        else {
            for(int r=0;r<h;r++) {
                for(int c=0;c<w;c++) {
                    int color = (((r / 8) ^ (c / 8)) & 1);
                    *data++ = color * red;
                }
            }
        }
        std::vector<unsigned char> buf;
        cv::imencode(".jpeg",img,buf);
        frame->jpeg_frame = std::shared_ptr<VideoFrame>(new VideoFrame(buf.data(),buf.size()));
        frame->source_frame = frame->jpeg_frame;
        return frame;
    }
}
