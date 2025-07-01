#pragma once
#include "plate_solver_result.h"
#include <cmath>
#include <stdio.h>
namespace ols {
    namespace PA_utils {
        struct XY {
            double x,y;
        };

        inline XY radec2xy(double ra,double dec)
        {
            double ra_rad = ra/180 * M_PI;
            double dec_rad = (90 - dec) / 180 * M_PI;
            double R = std::sin(dec_rad);
            double X = std::sin(ra_rad) * R;
            double Y = std::cos(ra_rad) * R;
            return XY{X,Y};
        }
        inline std::pair<double,double> xy2radec(double X,double Y,bool is_north_pole)
        {
            double ra = std::atan2(X,Y) / M_PI * 180;
            while(ra < 0) {
                ra+=360;
            }
            double dec = std::asin(std::sqrt(X*X + Y*Y)) / M_PI * 180;
            dec = 90 - dec;
            if(!is_north_pole)
                dec = -dec;
            return std::make_pair(ra,dec);
        }

        inline std::pair<double,double> get_pole_ra_dec(bool is_north_pole)
        {
            double year = time(nullptr) / (24*3600*365.25) + 1970;
            double X = (-1.08646546412010002E-08 * year + 4.34586126467511366E-05) * year  - 4.34586065382615364E-02;
            double Y = (-1.70034584971379974E-10 * year + 9.78567600103283049E-05) * year  - 1.95033493591933837E-01;
            if(!is_north_pole) { // need to check if correct
                X = -X;
                Y = -Y;
            }
            return xy2radec(X,Y,is_north_pole);
        }

        inline XY get_point(SolverResult const &r,double dx,double dy)
        {
            double ra  = r.c11 * dx + r.c12 * dy;
            double dec = r.c21 * dx + r.c22 * dy;
            ra = ra / std::cos(r.de0 / 180 * M_PI);
            ra += r.ra0;
            dec += r.de0;
            return radec2xy(ra,dec);
        }

        struct M3x3 {
            double a11,a12,a13;
            double a21,a22,a23;
            double a31,a32,a33;
            double det() const
            {
                return a11*a22*a33 - a11*a23*a32 - a12*a21*a33 + a12*a23*a31 + a13*a21*a32 - a13*a22*a31;
            }
            void print(FILE *f) const
            {
                for(int i=0;i<3;i++) {
                    for(int j=0;j<3;j++)
                        fprintf(f,"%f ",(*this)[i][j]);
                    fprintf(f,"\n");
                }
            }
            M3x3 inv() const
            {
                double D1 = 1.0 / det();
                M3x3 iM;

                iM.a11 = D1 * (a22*a33 - a23*a32);
                iM.a12 = D1 * (a13*a32 - a12*a33);
                iM.a13 = D1 * (a12*a23 - a13*a22);

                iM.a21 = D1 * (a23*a31 - a21*a33);
                iM.a22 = D1 * (a11*a33 - a13*a31);
                iM.a23 = D1 * (a13*a21 - a11*a23);

                iM.a31 = D1 * (a21*a32 - a22*a31);
                iM.a32 = D1 * (a12*a31 - a11*a32);
                iM.a33 = D1 * (a11*a22 - a12*a21);
                return iM;
            }
            double const *operator[](int v) const
            {
                switch(v) {
                case 0: return &a11;
                case 1: return &a21;
                case 2: return &a31;
                default:
                    return nullptr;
                }
            }
            double *operator[](int v) 
            {
                switch(v) {
                case 0: return &a11;
                case 1: return &a21;
                case 2: return &a31;
                default:
                    return nullptr;
                }
            }
            M3x3 trans() const
            {
                M3x3 t;
                for(int i=0;i<3;i++)
                    for(int j=0;j<3;j++)
                        t[i][j]=(*this)[j][i];
                return t;
            }
            M3x3 matmul(M3x3 const &r) const
            {
                M3x3 res;
                for(int i=0;i<3;i++) {
                    for(int j=0;j<3;j++) {
                        double s = 0;
                        for(int k=0;k<3;k++) {
                            s+= (*this)[i][k]*r[k][j];
                        }
                        res[i][j] = s;
                    }
                }
                return res;
            }
        };
        
        struct M2x2 {
            double a11,a12;
            double a21,a22;
            double det() const
            {
                return a11*a22 - a12*a21;
            }
            M2x2 inv() const
            {
                double D = 1.0 / det();
                M2x2 iM = {
                     a22*D, -a12*D,
                    -a21*D,  a11*D
                };

                return iM;
            }
            double const *operator[](int v) const
            {
                switch(v) {
                case 0: return &a11;
                case 1: return &a21;
                default:
                    return nullptr;
                }
            }
            double *operator[](int v)
            {
                switch(v) {
                case 0: return &a11;
                case 1: return &a21;
                default:
                    return nullptr;
                }
            }
            void print(FILE *f) const
            {
                for(int i=0;i<2;i++) {
                    for(int j=0;j<2;j++)
                        fprintf(f,"%f ",(*this)[i][j]);
                    fprintf(f,"\n");
                }
            }
        };

        inline M3x3 get_trans_3points(XY a0,XY b0,XY c0,XY a1,XY b1,XY c1)
        {
            M3x3 A = {
                a0.x, a0.y, 1.0,
                b0.x, b0.y, 1.0,
                c0.x, c0.y, 1.0
            };
            M3x3 iA = A.inv();
            M3x3 p1 = {
                a1.x,b1.x,c1.x,
                a1.y,b1.y,c1.y,
                0,0,0
            };
            M3x3 res = iA.matmul(p1.trans()).trans();
            res[2][2] = 1;
            return res;
        }

        inline XY get_pivot_point(M3x3 const &T) 
        {
            double M11 = T[0][0], M12 = T[0][1], M13 = T[0][2];
            double M21 = T[1][0], M22 = T[1][1], M23 = T[1][2];
            M2x2 M = {
                M11-1,M12,
                M21,  M22-1
            };
            if(std::fabs(M.det()) < 1e-10) {
                return XY{NAN,NAN};
            }
            M2x2 iM = M.inv();
            XY xy;
            xy.x = iM.a11 * -M13 + iM.a12 * -M23;
            xy.y = iM.a21 * -M13 + iM.a22 * -M23;
            return xy;
        }
        inline XY get_pivot_point(SolverResult const &s1,SolverResult const &s2)
        {
            XY a0 = get_point(s1,0,0);
            XY b0 = get_point(s1,10,0);
            XY c0 = get_point(s1,0,10);

            XY a1 = get_point(s2,0,0);
            XY b1 = get_point(s2,10,0);
            XY c1 = get_point(s2,0,10);
            M3x3 trans =  get_trans_3points(a0,b0,c0,a1,b1,c1);
            XY pivot = get_pivot_point(trans);
            return pivot;
        }
        inline bool solve_polar_align(SolverResult const &s1,SolverResult const &s2,double lat,double lon,PolarAlignResult &r)
        {
            XY pivot = get_pivot_point(s1,s2);
            if(std::isnan(pivot.x) || std::isnan(pivot.y))
                return false;
            bool north_pole = lat >= 0;
            auto pivot_radec = xy2radec(pivot.x,pivot.y,north_pole);
            auto target_radec = get_pole_ra_dec(north_pole);
            DeltaAZ delta = calc_delta_alt_az(target_radec.first,target_radec.second,pivot_radec.first,pivot_radec.second,lat,lon);
            r.error_alt = delta.delta_alt;
            r.error_az = delta.delta_az;

            Derotator d(lon,lat,s2.ra0,s2.de0);
            double time_s = time(nullptr);
            auto alt_az_s2 = d.alt_az_from_ra_dec(s2.ra0,s2.de0,time_s);
            alt_az_s2.first += delta.delta_alt;
            alt_az_s2.second += delta.delta_az;
            auto ra_dec_tgt = d.ra_dec_from_alt_az(alt_az_s2.first,alt_az_s2.second,time_s);
            r.target_ra_deg = ra_dec_tgt.first;
            r.target_de_deg = ra_dec_tgt.second;
            return true;
        }
    }// PA_utils
}
