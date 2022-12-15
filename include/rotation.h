#pragma once
#include <cmath>
#include <iostream>
#include <tuple>
namespace old {
class Derotator {
public:
    static constexpr float PI = 3.14159265f;
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
        float RAd = RAd_;
        float DEd = DEd_;
        auto camera0  = getCameraRay(RAd,DEd,time0_s);
        vec3d r0 = rayFromPos(RAd + 0.01,DEd,time0_s);
        vec3d c0 = cameraBearing(r0,camera0);

        auto camera1  = getCameraRay(RAd,DEd,time1_s);
        vec3d r1 = rayFromPos(RAd + 0.01,DEd,time1_s);
        vec3d c1 = cameraBearing(r1,camera1);

        return (std::atan2(c1.x,c1.y) - std::atan2(c0.x,c0.y)) * 180 / PI;
    }

    vec3d rayFromPos(float RAd,float DEd, double time_s)
    {
        auto deg2rad = PI / 180;
        auto RA = RAd * deg2rad;
        auto DE = DEd * deg2rad;
        auto jd = time_s / 86400.0 + 2440587.5;
        auto tu = jd - 2451545.0;
        auto angle = PI * 2 * (0.7790572732640+1.00273781191135448 * tu);
        auto q = angle + lon_d_*deg2rad;
        auto H = q - RA;
        
        auto f = deg2rad * lat_d_;

        auto az_y = std::sin(H);
        auto az_x = (std::cos(H) * std::sin(f) - std::tan(DE) * std::cos(f));
        auto az = std::atan2(az_y,az_x);
        auto sinH = std::sin(f) * std::sin(DE) + std::cos(f) * std::cos(DE) * std::cos(H);
        auto hz = std::asin(sinH);
        auto ray_n = -std::cos(az) * std::cos(hz);
        auto ray_e = -std::sin(az) * std::cos(hz);
        auto ray_u = std::sin(hz);
        return vec3d(ray_e, ray_n, ray_u);
    }
private:
    float lon_d_,lat_d_;
    float RAd_,DEd_;
};
} // OLS

