#include "tiffmat.h"
#include "tiffio.h"
#include "tiff.h"
#include <opencv2/imgproc.hpp>

namespace ols {
    void save_tiff(cv::Mat data,std::string file,bool bgr2rgb)
    {
        if(bgr2rgb) {
            cv::Mat tmp;
            if(data.channels() == 3) {
                cv::cvtColor(data,tmp,cv::COLOR_BGR2RGB);
                data = tmp;
            }
            else if(data.channels() == 4) {
                cv::cvtColor(data,tmp,cv::COLOR_BGRA2RGBA);
                data = tmp;
            }
        }
            
        TIFF *out= TIFFOpen(file.c_str(), "w");
        if(!out)
            throw std::runtime_error("Failed to open file to write "+ file);
        try {
            TIFFSetField(out,TIFFTAG_IMAGEWIDTH,data.cols);
            TIFFSetField(out,TIFFTAG_IMAGELENGTH,data.rows);
            TIFFSetField(out,TIFFTAG_SAMPLESPERPIXEL,data.channels());
            TIFFSetField(out,TIFFTAG_BITSPERSAMPLE,data.elemSize1()*8);
            TIFFSetField(out,TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
            TIFFSetField(out,TIFFTAG_PLANARCONFIG,PLANARCONFIG_CONTIG);
            TIFFSetField(out,TIFFTAG_PHOTOMETRIC, data.channels()== 3 ? PHOTOMETRIC_RGB : PHOTOMETRIC_MINISBLACK);
            switch(data.type() & CV_MAT_DEPTH_MASK) {
            case CV_8U:
            case CV_16U:
                TIFFSetField(out,TIFFTAG_SAMPLEFORMAT,SAMPLEFORMAT_UINT);
                break;
            case CV_8S:
            case CV_16S:
            case CV_32S:
                TIFFSetField(out,TIFFTAG_SAMPLEFORMAT,SAMPLEFORMAT_INT);
                break;
            case CV_32F:
            case CV_64F:
                TIFFSetField(out,TIFFTAG_SAMPLEFORMAT,SAMPLEFORMAT_IEEEFP);
                break;
            default:
                throw std::runtime_error("Unsupported matrix format");
            }
            size_t n = TIFFScanlineSize(out);
            if(n != data.cols * data.channels() * data.elemSize1())
                throw std::runtime_error("Internal error scanline size is inconsistent");
            for(int i=0;i<data.rows;i++) { 
                if(TIFFWriteScanline(out,(char*)(data.data) + data.step[0]*i,i,0) < 0)
                    throw std::runtime_error("Failed to write tiff image");
            }
        }
        catch(...) {
            TIFFClose(out);
            throw;
        }
        TIFFClose(out);
    }

    cv::Mat load_tiff(std::string file,bool bgr2rgb)
    {
        TIFF *in = TIFFOpen(file.c_str(),"r");
        if(!in)
            throw std::runtime_error("Failed to open tiff file " + file);
        try {
            uint32_t width,height;
            uint16_t depth,spp,format = SAMPLEFORMAT_UINT;
            TIFFGetField(in,TIFFTAG_IMAGEWIDTH,&width);
            TIFFGetField(in,TIFFTAG_IMAGELENGTH,&height);
            TIFFGetField(in,TIFFTAG_SAMPLESPERPIXEL,&spp);
            TIFFGetField(in,TIFFTAG_BITSPERSAMPLE,&depth);
            TIFFGetField(in,TIFFTAG_SAMPLEFORMAT,&format);
            if(spp <= 0 || spp > 4)
                throw std::runtime_error("Invalid format for " + file);
            int type = -1;
            switch(depth) {
            case 8:
                switch(format) {
                case SAMPLEFORMAT_INT: type = CV_MAKETYPE(CV_8S,spp);  break;
                case SAMPLEFORMAT_UINT: type = CV_MAKETYPE(CV_8U,spp);  break;
                }
                break;
            case 16:
                switch(format) {
                case SAMPLEFORMAT_INT: type = CV_MAKETYPE(CV_16S,spp);  break;
                case SAMPLEFORMAT_UINT: type = CV_MAKETYPE(CV_16U,spp);  break;
                }
                break;
            case 32:
                switch(format) {
                case SAMPLEFORMAT_INT: type = CV_MAKETYPE(CV_32S,spp);  break;
                case SAMPLEFORMAT_IEEEFP: type = CV_MAKETYPE(CV_32F,spp);  break;
                }
                break;
            case 64:
                switch(format) {
                case SAMPLEFORMAT_IEEEFP: type = CV_MAKETYPE(CV_64F,spp);  break;
                }
                break;
            }
            if(type == -1)
                throw std::runtime_error("Unsupported tiff format " + file + ": spp=" + std::to_string(spp) + " depth=" + std::to_string(depth) + " format=" + std::to_string(format));
            cv::Mat res(height,width,type);
            if(TIFFScanlineSize(in)!=int(res.step[0])) {
                throw std::runtime_error("Internal error in stride/scanline size in " + file);
            }
            for(unsigned i=0;i<height;i++) {
                if(TIFFReadScanline(in,(char*)res.data + res.step[0]*i,i,0)<0)
                    throw std::runtime_error("Failed to read " + file);
            }
            TIFFClose(in);
            if(bgr2rgb) {
                cv::Mat tmp;
                if(res.channels() == 3) {
                    cv::cvtColor(res,tmp,cv::COLOR_RGB2BGR);
                    res = tmp;
                }
                else if(res.channels() == 4) {
                    cv::cvtColor(res,tmp,cv::COLOR_RGBA2BGRA);
                    res = tmp;
                }
            }
            return res;
        }
        catch(...) {
            TIFFClose(in);
            throw;
        }
    }
}
