#include "fitsmat.h"
#include <fitsio.h>
#include <iostream>

namespace ols {
    static void make_error(int code,char const *op,std::string const &file_name)
    {
        char buffer[256]={};
        fits_get_errstatus(code,buffer);
        throw CamError("Failed to read file " + file_name + " " + buffer + " doing " + op);
    }
    static void make_error(char const *error,std::string const &file_name)
    {
        throw CamError("Failed to read file " + file_name + ":" + error);
    }

    std::pair<cv::Mat,CamBayerType> load_fits(std::string file)
    {
        fitsfile *fptr = nullptr;
        try {
            CamBayerType type=bayer_na;
            int status = 0;
            if(fits_open_file(&fptr,file.c_str(),READONLY,&status))
                make_error(status,"open",file);

            int bpp = 0,naxes = 0;
            long dims[3]={};
            if(fits_get_img_param(fptr,3,&bpp,&naxes,dims,&status)) 
                make_error(status,"get img param",file);
            
            if(naxes != 2 && naxes !=3)
                make_error("Expecting 2/3 axis image",file);

            if(naxes == 2)
                dims[2] = 1;
            if(naxes == 3 && dims[2] != 3)
                make_error("Expecting 3 axis to have dimention == 3/RGB",file);

            
            int npixels = dims[0]*dims[1]*dims[2];
            static constexpr int bufsize = 256;
            float buf[bufsize];
            long fp = 1;
            float nullval=0;
            int anynull;
            cv::Mat m(dims[1],dims[0],
                (bpp == 8 ? 
                      (naxes == 3 ? CV_8UC3 : CV_8UC1) 
                    : (naxes == 3 ? CV_16UC3 : CV_16UC1))
            );
            int plane_size = dims[0]*dims[1];
            while(npixels > 0) {
                int count = std::min(npixels,bufsize);
                if(fits_read_img(fptr,TFLOAT,fp,count,&nullval,buf,&anynull,&status))
                    make_error(status,"read_img",file);
                int pos = fp - 1;
                if(bpp == 8) {
                    unsigned char *data = (unsigned char *)m.data;
                    if(dims[2] == 1) {
                        for(int i=0;i<count;i++)
                            data[i + pos] = buf[i];
                    }
                    else {
                        for(int i=0;i<count;i++) {
                            int color = (i + pos) / plane_size;
                            int pix   = (i + pos) % plane_size;
                            data[pix*3 + (2-color)] = buf[i];
                        }
                    }

                }
                else {
                    uint16_t *data = (uint16_t *)m.data;
                    if(dims[2] == 1) {
                        for(int i=0;i<count;i++)
                            data[i + pos] = buf[i];
                    }
                    else {
                        for(int i=0;i<count;i++) {
                            int color = (i + pos) / plane_size;
                            int pix   = (i + pos) % plane_size;
                            data[pix*3 + (2-color)] = buf[i];
                        }
                    }
                }
                fp+=count;
                npixels-=count;

            }

            // FIXME later - test keyword exist to prevent errors
            char bayer[64]={};
            int blen=0;
            if(0 == fits_read_string_key(fptr,"BAYERPAT",1,sizeof(bayer),bayer,&blen,nullptr,&status)) {
                if(blen > 0)
                    type = bayer_type_from_str(bayer);
            }
        
            // Close
            fits_close_file(fptr,&status);
            fptr = nullptr;
            return std::make_pair(m,type);
        }
        catch(...) {
            int status = 0;
            if(fptr)
                fits_close_file(fptr,&status);
            throw;
        }
    }
    
} // ols

#if 0

int main(int argc,char **argv)
{
   try {
       std::pair<cv::Mat,ols::CamBayerType> r=ols::load_fits(argv[1]);
       std::cerr << r.first.rows<<"x"<<r.first.cols << " for " << r.first.elemSize1() << " Bayer=" << bayer_type_to_str(r.second)<< std::endl;
   }
   catch(std::exception const &e) {
       std::cerr << "ERROR:!" << e.what() << std::endl;
   }
}

#endif
