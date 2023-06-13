#include "plate_solver.h"
#include "tiffmat.h"
#include <opencv2/imgcodecs.hpp>
#include <iostream>


double parse_rede(char const *msg)
{
    std::istringstream ss(msg);
    double d=0,m=0,s=0;
    std::string str;
    char del;
    ss >> d;
    ss>>del;
    if(ss && del==':') {
        ss>>m;
        ss >> del;
        if(ss && del==':')
            ss >> s;
    }
    return d + m / 60 + s / 3600;
}

int main(int argc,char **argv)
{
    std::string db;
    std::string astap;
    std::string output_jpeg = "res.jpeg";
    std::string img_path;
    double fov=0.524; // ASI224  / 400mm
    double radius = 5.0;
    double ra = 130.095833; // m44
    double de = 19.666666;

    for(;argc>=3;argc-=2,argv+=2) {
        std::string op = argv[1];
        char *val = argv[2];
        if(op == "--db")
            db = val;
        else if(op == "--astap")
            astap = val;
        else if(op =="--out")
            output_jpeg = val;
        else if(op == "--ra")
            ra = parse_rede(val) * 15;
        else if(op == "--de")
            de = parse_rede(val);
        else if(op == "--fov")
            fov = atof(val);
        else if(op == "-r")
            radius = atof(val); 
    }
    if(argc != 2) {
        std::cerr << "Usage [--db Path] [--astap Path] [--out output.jpeg] [--ra HH:MM:SS] [--de DD:MM:SS] [--fov FOV] [-r Radius] img" << std::endl;
        return 1;
    }

    img_path=argv[1];
    
    
    try {
        ols::PlateSolver ps(db,astap);
        cv::Mat img;
        if(img_path.find(".tiff")!=std::string::npos)
            img = ols::load_tiff(img_path);
        else
            img = cv::imread(img_path);
        auto r=ps.solve_and_mark(img,true,
                                 output_jpeg,
                                 fov,ra,de,radius,5.0);
        std::cout << "From " << r.center_col <<"x"<<r.center_row << " -> " << r.target_col << "x" << r.target_row << " total " << r.angle_to_target_deg << " deg" << std::endl;
    }
    catch(std::exception const &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    return 0;
}
