#pragma once
#include "ctl_app.h"
#include "plate_solver.h"
#include <cppcms/application.h>
#include <cppcms/http_context.h>
#include <cppcms/http_response.h>
#include <cppcms/url_dispatcher.h>
#include <cppcms/service.h>
#include <booster/log.h>

#include "server_sent_events.h"
#include "util.h"
namespace ols {
    class PlateSolverControlApp : public ControlAppBase {
    public:
        PlateSolverControlApp(cppcms::service &srv,
                          std::string data_dir) :
            ControlAppBase(srv),
            data_dir_(data_dir)
        {
            dispatcher().map("POST","/?",&PlateSolverControlApp::solve,this);
        }
        void solve()
        {
            double fov = content_.get<double>("fov");
            double ra  = content_.get<double>("ra");
            double de  = content_.get<double>("de");
            double rad = content_.get<double>("rad");
            std::string img = "/plate_solving_solution.jpeg";
            std::string jpeg = data_dir_ + img;
            try {
                auto res = PlateSolver::solve_last_image(jpeg,fov,ra,de,rad);
                response_["solved"]=true;
                response_["distance_to_target"] = res.angle_to_target_deg;
                response_["result_image"] = "/data" + img;
            }
            catch(std::exception const &e) {
                response_["solved"]=false;
                response_["error"] = e.what();
            }
        }
    private:
        std::string data_dir_;
    };

};

