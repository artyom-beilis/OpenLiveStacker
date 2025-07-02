#pragma once
#include <cmath>
#include <iostream>
#include <tuple>
namespace ols {
    struct SolverResult {
        double x0,y0,ra0,de0;
        double c11,c12,c21,c22;
    };
    
    struct PolarAlignResult {
        float target_ra_deg = 0,target_de_deg = 0;
        float error_alt = 0,error_az = 0;
    };
    class Derotator {
    public:
        struct vec3d {
            float x,y,z;
            vec3d(float a=0, float b=0, float c=0) : x(a), y(b), z(c)
            {
            }
            vec3d operator-(vec3d other) const
            {
                return vec3d(x-other.x,y-other.y,z-other.z);
            }
            float normv() const
            {
                return std::sqrt(x*x+y*y+z*z);
            }
            vec3d norm() const
            {
                float nv = normv();
                return vec3d(x/nv,y/nv,z/nv);
            }
            float sprod(vec3d r) const
            {
                return x*r.x + y*r.y + z*r.z;
            }
            vec3d cross(vec3d r) const
            {
                float a[3] ={  x,  y,  z};
                float b[3] ={r.x,r.y,r.z};
                return vec3d(
                    a[1]*b[2] - a[2]*b[1],
                    a[2]*b[0] - a[0]*b[2],
                    a[0]*b[1] - a[1]*b[0]
                );
            }
        };

        Derotator(float lon_d,float lat_d,float RAd, float DEd) : 
            lon_d_(lon_d), lat_d_(lat_d),
            RAd_(RAd),DEd_(DEd)
        {
        }


        std::tuple<vec3d,vec3d,vec3d> getCameraRay(float RAd,float DEd,double time_s)
        {
            vec3d fwd = rayFromPos(RAd,DEd,time_s);

            float fwd_hlen = std::sqrt(fwd.x*fwd.x + fwd.y*fwd.y);
            vec3d fwd_hor(fwd.x/fwd_hlen,fwd.y/fwd_hlen,0.0);
            vec3d lft(-fwd_hor.y,fwd_hor.x,0.0);
            vec3d top = fwd.cross(lft);
            return std::make_tuple(top,lft,fwd);
        }

        vec3d cameraBearing(vec3d ray,std::tuple<vec3d,vec3d,vec3d> cameraRays)
        {
            auto top = std::get<0>(cameraRays);
            auto lft = std::get<1>(cameraRays);
            auto fwd = std::get<2>(cameraRays);
            auto x = -lft.sprod(ray);
            auto y = top.sprod(ray);
            auto z = fwd.sprod(ray);
            return vec3d(x,y,z);
        }

        float getAngleDeg(double time0_s,double time1_s)
        {
            double RAd = RAd_;
            double DEd = DEd_;
            auto camera0  = getCameraRay(RAd,DEd,time0_s);
            vec3d r0 = rayFromPos(RAd + 0.01,DEd,time0_s);
            vec3d c0 = cameraBearing(r0,camera0);

            auto camera1  = getCameraRay(RAd,DEd,time1_s);
            vec3d r1 = rayFromPos(RAd + 0.01,DEd,time1_s);
            vec3d c1 = cameraBearing(r1,camera1);

            return (std::atan2(c1.x,c1.y) - std::atan2(c0.x,c0.y)) * 180 / M_PI;
        }
        
        std::pair<float,float> ra_dec_from_alt_az(float alt_d,float az_d, double time_s)
        {
            double deg2rad = M_PI / 180;
            double lat = lat_d_ * deg2rad;
            double lon = lon_d_ * deg2rad;
            double alt = alt_d * deg2rad;
            double az  = az_d * deg2rad;
            double jd = time_s / 86400.0 + 2440587.5;

            double tu = jd - 2451545.0;
            double angle = M_PI * 2 * (0.7790572732640+1.00273781191135448 * tu);

            double DEC =  std::asin(std::sin(lat) * std::sin(alt) + std::cos(lat) * cos(alt) * cos(az));
            double H = std::atan2(-std::sin(az), (std::tan(alt) * std::cos(lat) - std::cos(az) * std::sin(lat) ));;
            double RA = angle - H + lon;
            double RAc = RA / (2*M_PI);
            double RAd = (RAc - floor(RAc))*360;
            double DEd = DEC / deg2rad;
            return std::pair<float,float>(RAd,DEd);

        }
        
        std::pair<float,float> alt_az_from_ra_dec(float RAd,float DECd, double time_s)
        {
            double rad2deg = 180 / M_PI;
            vec3d r = rayFromPos(RAd,DECd,time_s);
            double U = r.z;
            double E = r.x;
            double N = r.y;
            double alt = std::asin(U) * rad2deg;
            double az = std::atan2(E,N) * rad2deg;
            return std::pair<float,float>(alt,az);
        }

        vec3d rayFromPos(float RAd,float DEd, double time_s)
        {
            double deg2rad = M_PI / 180;
            double RA = RAd * deg2rad;
            double DE = DEd * deg2rad;
            double jd = time_s / 86400.0 + 2440587.5;
            double tu = jd - 2451545.0;
            double angle = M_PI * 2 * (0.7790572732640+1.00273781191135448 * tu);
            double q = angle + lon_d_*deg2rad;
            double H = q - RA;
            
            double f = deg2rad * lat_d_;

            double az_y = std::sin(H);
            double az_x = (std::cos(H) * std::sin(f) - std::tan(DE) * std::cos(f));
            double az = std::atan2(az_y,az_x);
            double sinH = std::sin(f) * std::sin(DE) + std::cos(f) * std::cos(DE) * std::cos(H);
            double hz = std::asin(sinH);
            double ray_n = -std::cos(az) * std::cos(hz);
            double ray_e = -std::sin(az) * std::cos(hz);
            double ray_u = std::sin(hz);
            return vec3d(ray_e, ray_n, ray_u);
        }
    private:
        float lon_d_,lat_d_;
        float RAd_,DEd_;
    };

    struct DeltaAZ {
        double delta_alt;
        double delta_az;
    };

    inline DeltaAZ calc_delta_alt_az(double t_ra,double t_de,double c_ra,double c_de,double lat,double lon)
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

}
