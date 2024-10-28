#pragma once
#include "ctl_app.h"
#include "plate_solver.h"
#include <cppcms/application.h>
#include <cppcms/http_context.h>
#include <cppcms/http_response.h>
#include <cppcms/url_dispatcher.h>
#include <cppcms/service.h>
#include <booster/log.h>

#include "rotation.h"
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
        struct DeltaAZ {
            double delta_alt;
            double delta_az;
        };

        DeltaAZ calc_delta_alt_az(double t_ra,double t_de,double c_ra,double c_de,double lat,double lon)
        {
            Derotator derot(lon,lat,c_ra,c_de);
            double time_now = time(nullptr); 
            auto src = derot.rayFromPos(c_ra,c_de,time_now);
            double src_E = src.x;
            double src_N = src.y;
            double src_U = src.z;
            auto tgt = derot.rayFromPos(t_ra,t_de,time_now);
            double tgt_E = tgt.x;
            double tgt_N = tgt.y;
            double tgt_U = tgt.z;

            double d_alt = (std::asin(tgt_U)  - std::asin(src_U)) * 180 / M_PI;
            double d_az = (std::atan2(tgt_E,tgt_N) - std::atan2(src_E,src_N)) * 180 / M_PI;
            if(d_az > 180)
                d_az -= 360;
            if(d_az < -180)
                d_az += 360;
            DeltaAZ res;
            res.delta_alt = d_alt;
            res.delta_az  = d_az;
            return res;
        }

        void solve()
        {
            double fov = content_.get<double>("fov");
            double ra  = content_.get<double>("ra");
            double de  = content_.get<double>("de");
            double mount_ra  = content_.get<double>("mount_ra");
            double mount_de  = content_.get<double>("mount_de");
            double rad = content_.get<double>("rad");
            double timeout = content_.get<double>("timeout");
            timeout = std::max(std::min(timeout,180.0),5.0);
            double constexpr invalid_geolocation = 1e6;
            double lat = content_.get<double>("lat",invalid_geolocation);
            double lon = content_.get<double>("lon",invalid_geolocation);
            std::string img = "/plate_solving_solution.jpeg";
            std::string jpeg = data_dir_ + img;
            try {
                auto res = PlateSolver::solve_last_image(
                            jpeg,fov,
                            ra,de,
                            mount_ra,mount_de,
                            rad,timeout);
                response_["solved"]=true;
                response_["distance_to_target"] = res.angle_to_target_deg;
                response_["result_image"] = "/data" + img;
                response_["center_ra"] = res.center_ra_deg;
                response_["center_de"] = res.center_de_deg;
                response_["delta_ra"] = ra - res.center_ra_deg;
                response_["delta_de"] = de - res.center_de_deg;
                if(lat != invalid_geolocation && lon != invalid_geolocation) {
                    auto delta = calc_delta_alt_az(ra,de,res.center_ra_deg, res.center_de_deg,lat,lon);
                    response_["delta_alt"] = delta.delta_alt;
                    response_["delta_az"]  = delta.delta_az;
                }
                else {
                    response_["delta_alt"] = cppcms::json::null();
                    response_["delta_az"]  = cppcms::json::null();
                }
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

