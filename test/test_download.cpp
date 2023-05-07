#include "downloader.h"
#include <stdio.h>
int main(int argc,char **argv)
{
    if(argc!=3) {
        printf("Usage URL output_dir\n");
        return 1;
    }
    std::string error_message;
    bool r = ols::zip_download(argv[1],argv[2],error_message,[](char const *file)->bool {
        printf("Downloading %s\n",file);
        return true;
    });
    if(!r) {
        printf("Failed: %s\n",error_message.c_str());
        return 1;
    }
    else {
        printf("Ok\n");
    }
    return 0;
}
