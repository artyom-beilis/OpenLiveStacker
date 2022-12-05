#include "ols.h"
#include <iostream>
#include <fstream>
#include <unistd.h>

int main()
{   
    try {

        ols::OpenLiveStacker stacker;
        stacker.init("UVC",0);
        stacker.run();
        stacker.shutdown();
    }
    catch(std::exception const &e) {
        std::cerr << "failed:" << e.what() << std::endl;
        return 1;
    }
    return 0;
}
