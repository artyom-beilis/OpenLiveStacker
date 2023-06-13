#include "plate_solver.h"
#include "tiffmat.h"
#include "util.h"
#include "live_stretch.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <booster/posix_time.h>
#include <booster/log.h>

namespace ols {
    PlateSolver::PlateSolver(std::string const &db_path,std::string const &astap) :
        db_(db_path),
        exe_(astap),
        temp_dir_("/tmp")
    {
        if(exe_.empty())
            exe_ = "astap_cli";
    }
    void PlateSolver::set_tempdir(std::string const &p) 
    {
        temp_dir_ = p;
    }
    PlateSolver::Result PlateSolver::solve(std::string const &img_path,double fov,double ra,double de,double rad,double timeout)
    {
        std::vector<std::string> cmd = {
            exe_,
           "-f",img_path,
           "-fov",std::to_string(fov),
            "-ra",std::to_string(ra/15.0),
            "-spd",std::to_string(de+90),
            "-r",std::to_string(rad),
            "-o",temp_dir_ + "/ols_astap_output"
        };
        if(!db_.empty()) {
            cmd.push_back("-d");
            cmd.push_back(db_);
        }
        std::string res_file = temp_dir_ + "/ols_astap_output.ini";
        std::remove(res_file.c_str());
        int status = run(cmd,res_file,timeout);
        switch(status) {
        case 0: break;
        case 1: 
            {
                std::string err;
                parse_ini(res_file,err);
                throw std::runtime_error("Failed to find solution " + err);
            }
        case 2: throw std::runtime_error("Not enough stars detected");
        case 16:throw std::runtime_error("Error reading image file: " + img_path);
        case 32:throw std::runtime_error("No star database found " + db_);
        case 33:throw std::runtime_error("Error reading star database " + db_);
        default:throw std::runtime_error("astap returned error code " + std::to_string(status));
        }
        return make_result(res_file,ra,de);
    }

    PlateSolver::Result PlateSolver::make_result(std::string const &path,double ra,double de)
    {
        std::string err;
        std::map<std::string,double> vals = parse_ini(path,err);
        if(!vals["PLTSOLVD"])
            throw std::runtime_error("Plate solving failed:" + err);
        double x0 = get(vals,"CRPIX1");
        double y0 = get(vals,"CRPIX2");
        double ra0 = get(vals,"CRVAL1");
        double de0 = get(vals,"CRVAL2");
        double delta_ra = (ra - ra0)*std::cos(de0/180*M_PI);
        double delta_de = de - de0;
        double c11 = get(vals,"CD1_1");
        double c12 = get(vals,"CD1_2");
        double c21 = get(vals,"CD2_1");
        double c22 = get(vals,"CD2_2");

        double D=1.0/(c11*c22 - c12*c21);
        double Mi[2][2] = {
            { c22*D, -c12*D},
            {-c21*D,  c11*D}
        };
        double xt = Mi[0][0] * delta_ra + Mi[0][1] * delta_de + x0;
        double yt = Mi[1][0] * delta_ra + Mi[1][1] * delta_de + y0;
        double diff_deg = std::sqrt(delta_ra*delta_ra + delta_de*delta_de);
        int rows = y0 * 2;
        Result r;
        r.center_col = x0;
        r.center_row = y0;
        r.target_col = xt;
        r.target_row = rows - yt;
        r.angle_to_target_deg = diff_deg;
        r.center_ra_deg = ra0;
        r.center_de_deg = de0;
        return r;
    }
    double PlateSolver::get(std::map<std::string,double> const &vals,std::string const &name)
    {
        auto p = vals.find(name);
        if(p==vals.end())
            throw std::runtime_error("Failed to get results key " + name);
        return p->second;
    }
    std::string PlateSolver::trim(std::string const &s)
    {
        size_t start = s.find_first_not_of(" \t\r\n");
        if(start == std::string::npos)
            return "";
        size_t end = s.find_last_not_of(" \t\r\n");
        if(end == std::string::npos)
            return s.substr(start);
        else
            return s.substr(start,end+1-start);
    }
    std::map<std::string,double> PlateSolver::parse_ini(std::string const &path,std::string &error)
    {
        std::map<std::string,double> vals;
        std::ifstream f(path);
        if(!f)
            throw std::runtime_error("Failed to open " + path);
        std::string key,value;
        while(std::getline(f,key,'=') && std::getline(f,value)) {
            double v;
            key = trim(key);
            value = trim(value);
            if(key == "PLTSOLVD") {
                v = value == "T";
            }
            else if(key == "ERROR") {
                error = value;
                continue;
            }
            else
                v = atof(value.c_str());
            vals[key]=v;
        }
        return vals;
    }

    PlateSolver::Result PlateSolver::solve_and_mark(
            cv::Mat &img,bool do_stretch,
            std::string const &jpeg_with_marks,
            double fov_deg,
            double target_ra_deg,
            double target_de_deg,
            double search_radius_deg,
            double timeout)
    {
        std::string tiff = temp_dir_ + "/ols_astap_input.tiff";
        save_tiff(img,tiff);
        auto r = solve(tiff,fov_deg,target_ra_deg,target_de_deg,search_radius_deg,timeout);
        if(img.channels() != 3) {
            cv::Mat tmp;
            cv::cvtColor(img,tmp,cv::COLOR_GRAY2BGR);
            img = tmp;
        }
        cv::Mat img8bit;
        if(do_stretch) {
            live_stretch(img,img8bit);
        }
        else {
            double factor = img.elemSize1() == 1 ? 1.0 :  (255.0 / 65535.0);
            img.convertTo(img8bit,CV_8UC3,factor);
        }
        cv::arrowedLine(img8bit,
                        cv::Point(r.center_col,r.center_row),
                        cv::Point(r.target_col,r.target_row),
                        cv::Scalar(0,0,255),
                        3);
        cv::imwrite(jpeg_with_marks,img8bit);
        return r;
    }


    int PlateSolver::run(std::vector<std::string> &opts,std::string ini_path,double timeout_sec)
    {
        std::vector<char *> args;
        std::ostringstream cmd;
        #if 0 //ifdef ANDROID_SUPPORT - old workaround
        // workaround of linker issue
        char exe[128];
        snprintf(exe,sizeof(exe),"/system/bin/linker%s",(sizeof(void*)==8 ? "64" : ""));
        args.push_back(exe);
        #else
        char const *exe=exe_.c_str();
        #endif
        for(auto &opt: opts) {
            args.push_back(&opt[0]);
        }
        for(char *arg : args) {
            cmd << arg << " ";
        }
        BOOSTER_INFO("ols") << "Running plate solver, exe=" << exe << " for architecture=" << OLS_ARCH <<  " with following command: "<<cmd.str();
        args.push_back(nullptr);
        #ifdef ANDROID_SUPPORT
        if(!exists(exe_)) {
            BOOSTER_ERROR("ols") << "There is no such executable file " << exe_ << ". Is it x86/x86_64 android?";
            throw std::runtime_error("The ASTAP isn't found, so such file " + exe_); 
        }
        #endif        
        int child_pid = fork();
        if(child_pid == 0) {
            int in_fd = open("/dev/null",O_RDONLY);
            int out_fd = open("/dev/null",O_WRONLY);
            dup2(in_fd,0);
            dup2(out_fd,1);
            dup2(out_fd,2);
            #ifdef ANDROID_SUPPORT
            execv(exe,args.data());
            #else
            execvp(exe,args.data());
            #endif
            int err = errno;
            std::ofstream f(ini_path);
            f<<"PLTSOLVD=F\nERROR=exec of " << exe_ << " failed with: " << strerror(err);
            f.close();
            _exit(1);
        }
        else if(child_pid < 0) {
            throw std::system_error(errno,std::generic_category(),"Failed to create process"); 
        }
        else {
            int N = int(timeout_sec * 10);
            int status = 0;
            bool wait_done = false;
            bool timeout = false;
            for(int i=0;i<N+1;i++) {
                booster::ptime::millisleep(100);
                int r,opt = WNOHANG;
                if(i >= N) {
                    timeout = true;
                    kill(child_pid,SIGKILL);
                    opt = 0;
                }
                r = waitpid(child_pid,&status,opt);
                int err = errno;
                if(r == 0)
                    continue;
                if(r < 0)
                    throw std::system_error(err,std::generic_category(),"wait failed"); 
                if(r != child_pid)
                    throw std::runtime_error("Unexpected wait returned!");
                wait_done = true;
                break;
            }
            if(!wait_done)
                throw std::runtime_error("Failed to wait for process");
            if(timeout)
                throw std::runtime_error("Execution took too much time, current limit is " + std::to_string(timeout_sec) +"s");
            if(!WIFEXITED(status))
                throw std::runtime_error("Execution of the astap process failed");
            return WEXITSTATUS(status);
        }
    }


    void PlateSolver::init(std::string const &db_path,std::string const &path_to_astap_cli,std::string const &temp_dir)
    {
        std::unique_lock<std::mutex> g(lock_);
        instance_.reset(new PlateSolver(db_path,path_to_astap_cli));
        if(!temp_dir.empty())
            instance_->set_tempdir(temp_dir);
    }

    std::string PlateSolver::db_path()
    {
        if(!instance_)
            throw std::runtime_error("PlateSolver is not configured");
        return instance_->db_;
    }
    
    void PlateSolver::set_image(cv::Mat img,bool do_stretch)
    {
        cv::Mat img_copy = img.clone();
        std::unique_lock<std::mutex> g(img_lock_);
        image_.reset(new cv::Mat(img_copy));
        do_stretch_ = do_stretch;
    }

    PlateSolver::Result PlateSolver::solve_last_image( std::string const &jpeg_with_marks,
                                        double fov,
                                        double ra,
                                        double de,
                                        double rad,
                                        double timeout)
    {
        cv::Mat img;
        bool do_stretch;
        {
            std::unique_lock<std::mutex> g(img_lock_);
            if(!image_)
                throw std::runtime_error("No image was set");
            img = *image_;
            do_stretch = do_stretch_;
        }
        {
            std::unique_lock<std::mutex> g(lock_);
            if(!instance_)
                throw std::runtime_error("plate solver is not ready");
            return instance_->solve_and_mark(img,do_stretch,jpeg_with_marks,fov,ra,de,rad,timeout);
        }
    }

    /// static stuff
    std::mutex PlateSolver::lock_;
    std::mutex PlateSolver::img_lock_;
    std::unique_ptr<PlateSolver> PlateSolver::instance_;
    std::unique_ptr<cv::Mat> PlateSolver::image_;
    bool PlateSolver::do_stretch_;


}
