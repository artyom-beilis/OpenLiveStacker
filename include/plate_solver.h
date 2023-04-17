#pragma once
#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <memory>

namespace cv { class Mat; }
namespace ols {
    class PlateSolver {
    public:
        struct Result {
            int center_row;
            int center_col;
            int target_row;
            int target_col;
            double angle_to_target_deg;
        };

        static void init(std::string const &db_path,std::string const &path_to_astap_cli,double time_limit_sec = 5.0,std::string const &temp_dir=std::string());
        static void set_image(cv::Mat img,int dynamic_range);
        static Result solve_last_image( std::string const &jpeg_with_marks,
                                        double fov_deg,
                                        double target_ra_deg,
                                        double target_de_deg,
                                        double search_radius_deg);


        PlateSolver(std::string const &db_path,std::string const &path_to_astap_cli,double time_limit_sec = 5.0);

        void set_tempdir(std::string const &p);
        
        Result solve_and_mark(  cv::Mat &img,int dynamic_range,
                                std::string const &jpeg_with_marks,
                                double fov_deg,
                                double target_ra_deg,
                                double target_de_deg,
                                double search_radius_deg);
            

        Result solve(   std::string const &img_path,
                        double fov_deg,
                        double target_ra_deg,
                        double target_de_deg,
                        double search_radius_deg);
    private:
        std::string trim(std::string const &s);
        Result make_result(std::string const &path,double ra,double de);
        double get(std::map<std::string,double> const &vals,std::string const &name);
        std::map<std::string,double> parse_ini(std::string const &path,std::string &error);
        int run(std::vector<std::string> &opts,std::string ini_path);
        std::string db_,exe_;
        double timeout_;
        std::string temp_dir_;

        static std::mutex lock_;
        static std::mutex img_lock_;
        static std::unique_ptr<PlateSolver> instance_;
        static std::unique_ptr<cv::Mat> image_;
        static int image_dr_;
    };

} // namespace ols