#include "indigo_camera.h"
#include <sstream>
#include <iostream>

#define TEST(x) do { if(!(x)) { throw std::runtime_error("Failed :" #x); } } while(0)

bool same_fmt(ols::CamStreamFormat fmt,std::string const &f)
{
    std::cout << "Testing " << f << std::endl;
    std::ostringstream ss;
    ss<<fmt;
    if(ss.str() != f)
        std::cerr << "Invalid formats:" << ss.str() << "!="<<f;
    return ss.str() == f;
}

int main()
{
    try {
        using namespace ols;
        CamStreamFormat fmt;
        int bits=0;
        std::cout << "ToupTek" << std::endl;

        TEST(parse_indigo_format("MON08_1","MON 8 3040x2048",fmt,bits));
        TEST(same_fmt(fmt,"mono8:3040x2048"));
        TEST(bits==8);
        TEST(parse_indigo_format("MON14_1","MON 14 3040x2048",fmt,bits));
        TEST(same_fmt(fmt,"mono16:3040x2048"));
        TEST(parse_indigo_format("MON14_3","MON 14 1012x682",fmt,bits));
        TEST(same_fmt(fmt,"mono16:1012x682:bin3"));
        TEST(bits==14);

        std::cout << "ASI" << std::endl;

        TEST(parse_indigo_format("RAW 8 1x1","RAW 8 1304x976",fmt,bits));
        TEST(same_fmt(fmt,"raw8:1304x976"));
        TEST(parse_indigo_format("RGB 24 1x1","RGB 24 1304x976",fmt,bits));
        TEST(same_fmt(fmt,"rgb24:1304x976"));
        TEST(parse_indigo_format("RAW 16 1x1","RAW 16 1304x976",fmt,bits));
        TEST(same_fmt(fmt,"raw16:1304x976"));
        TEST(parse_indigo_format("Y 8 1x1","Y 8 1304x976",fmt,bits));
        TEST(same_fmt(fmt,"mono8:1304x976"));
        TEST(parse_indigo_format("RAW 8 2x2","RAW 8 652x488",fmt,bits));
        TEST(same_fmt(fmt,"raw8:652x488:bin2"));
        TEST(parse_indigo_format("RGB 24 2x2","RGB 24 652x488",fmt,bits));
        TEST(same_fmt(fmt,"rgb24:652x488:bin2"));
        TEST(parse_indigo_format("RAW 16 2x2","RAW 16 652x488",fmt,bits));
        TEST(same_fmt(fmt,"raw16:652x488:bin2"));
        TEST(parse_indigo_format("Y 8 2x2","Y 8 652x488",fmt,bits));
        TEST(same_fmt(fmt,"mono8:652x488:bin2"));

        std::cout << "SVBony/QHY" << std::endl;
        TEST(parse_indigo_format("RAW 8 1x1","RAW 8 1024x768",fmt,bits) && same_fmt(fmt,"raw8:1024x768"));
        TEST(parse_indigo_format("RAW 16 1x1","RAW 8 1024x768",fmt,bits) && same_fmt(fmt,"raw16:1024x768"));
        TEST(parse_indigo_format("RAW 16 2x2","RAW 8 1024x768",fmt,bits) && same_fmt(fmt,"raw16:1024x768:bin2"));
        TEST(parse_indigo_format("Y 8 1x1","Y 8 1024x768",fmt,bits) && same_fmt(fmt,"mono8:1024x768"));
        TEST(parse_indigo_format("Y 16 1x1","Y 8 1024x768",fmt,bits) && same_fmt(fmt,"mono16:1024x768"));
        TEST(parse_indigo_format("Y 16 2x2","Y 8 1024x768",fmt,bits) && same_fmt(fmt,"mono16:1024x768:bin2"));
        TEST(parse_indigo_format("RGB 24 1x1","RGB 8 1024x768",fmt,bits) && same_fmt(fmt,"rgb24:1024x768"));
        TEST(parse_indigo_format("RGB 24 2x2","RGB 24 1024x768",fmt,bits) && same_fmt(fmt,"rgb24:1024x768:bin2"));

        std::cout << "UVC" << std::endl;
        TEST(parse_indigo_format("0_1024x768","YUV 1024x768",fmt,bits) && same_fmt(fmt,"rgb24:1024x768"));

    }
    catch(std::exception const &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    std::cout << "Ok" << std::endl;
    return 0;
}
