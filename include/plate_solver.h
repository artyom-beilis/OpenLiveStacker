#pragma once
#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <memory>
#include "plate_solver_result.h"

namespace cv { class Mat; }
namespace ols {

    

    class PlateSolver {
    public:
        enum SolverFlag {
            solve_normal = 0,
            polar_start  = 1,
            polar_find_target = 2
        };
        struct Result {
            double center_ra_deg;
            double center_de_deg;
            int center_row;
            int center_col;
            int target_row;
            int target_col;
            double angle_to_target_deg;
            SolverFlag flag;
            PolarAlignResult polar_alignment;
        };

        static std::string db_path();
        static void init(std::string const &db_path,std::string const &path_to_astap_cli,std::string const &temp_dir=std::string());
        static void set_image(cv::Mat img,bool stretch);
        static Result solve_last_image( std::string const &jpeg_with_marks,
                                        double fov_deg,
                                        double target_ra_deg,
                                        double target_de_deg,
                                        double mount_ra_deg,
                                        double mount_de_deg,
                                        double search_radius_deg,
                                        double timeout,
                                        SolverFlag flag,
                                        double lat, double lon);


        PlateSolver(std::string const &db_path,std::string const &path_to_astap_cli);

        void set_tempdir(std::string const &p);
        
        Result solve_and_mark(  cv::Mat &img,bool do_stretch,
                                std::string const &jpeg_with_marks,
                                double fov_deg,
                                double target_ra_deg,
                                double target_de_deg,
                                double mount_ra_deg,
                                double mount_de_deg,
                                double search_radius_deg,
                                double timeout,
                                SolverFlag flag,
                                double lat, double lon);
            

        Result solve(   std::string const &img_path,
                        double fov_deg,
                        double target_ra_deg,
                        double target_de_deg,
                        double mount_ra_deg,
                        double mount_de_deg,
                        double search_radius_deg,
                        double timeout,
                        SolverFlag flag,
                        double lat, double lon);
    private:
        SolverResult extract_result_from_ini(std::string const &path);
        std::string trim(std::string const &s);
        Result make_result(SolverResult const &sr,double ra,double de);
        Result make_result(std::string const &path,double ra,double de);
        double get(std::map<std::string,double> const &vals,std::string const &name);
        std::map<std::string,double> parse_ini(std::string const &path,std::string &error);
        int run(std::vector<std::string> &opts,std::string ini_path,double timeout);
        std::string db_,exe_;
        std::string temp_dir_;
        SolverResult saved_result_;

        static std::mutex lock_;
        static std::mutex img_lock_;
        static std::unique_ptr<PlateSolver> instance_;
        static std::unique_ptr<cv::Mat> image_;
        static bool do_stretch_;
    };

} // namespace ols
