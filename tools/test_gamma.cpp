#include <opencv2/core.hpp>
#include <chrono>
#include <stdio.h>
#include "simd_ops.h"
static constexpr int gamma_table_size = 128;
float gamma_table_current_gamma_ = -1;
float gamma_ = 1.3;
float gamma_table_[gamma_table_size+1];

void apply_gamma(cv::Mat &frame)
{
    float t_factor = 1.0f/(gamma_table_size-1);
    if(gamma_table_current_gamma_ == -1.0f || gamma_table_current_gamma_ != gamma_) {
        for(int i=0;i<gamma_table_size;i++) {
            gamma_table_[i] = i * t_factor;
        }
        gamma_table_[gamma_table_size] = 1.0f;
        cv::Mat tmp(1,gamma_table_size,CV_32FC1,gamma_table_);
        cv::pow(tmp,gamma_,tmp);
        gamma_table_current_gamma_ = gamma_;
    }

    float *p = (float*)frame.data;
    int N = frame.rows*frame.cols*3;
    int i=0;
#ifdef V_SIMD
    using namespace ols;
    v_f32 one  = v_setall_f32(1.0f);
    v_f32 mmin = v_setall_f32(gamma_table_size-1.0f);

    int limit = N / 4 * 4;

    for(i=0;i<limit;i+=4,p+=4) {
        v_f32 v = v_load_f32(p);
        v_f32 vf = v_mul_f32(v,mmin);
        v_f32 findx = v_floor_f32(vf);
        v_s32 indx = v_min_s32(v_setall_s32(gamma_table_size-1),v_max_s32(v_setall_s32(0),v_cvt_f32_s32(findx)));
        v_f32 w1 = v_sub_f32(vf,findx);
        v_f32 w0 = v_sub_f32(one,w1);
#if 1        
        int indexes[4];
        float p0[4],p1[4];
        v_store_s32(indexes,indx);

        p0[0] = gamma_table_[indexes[0]];
        p1[0] = gamma_table_[indexes[0]+1];
        p0[1] = gamma_table_[indexes[1]];
        p1[1] = gamma_table_[indexes[1]+1];
        p0[2] = gamma_table_[indexes[2]];
        p1[2] = gamma_table_[indexes[2]+1];
        p0[3] = gamma_table_[indexes[3]];
        p1[3] = gamma_table_[indexes[3]+1];

        v = v_add_f32(v_mul_f32(w0,v_load_f32(p0)),v_mul_f32(w1,v_load_f32(p1)));
#else
        v_f32 p0 = _mm_i32gather_ps(gamma_table_,indx,4);
        v_f32 p1 = _mm_i32gather_ps(gamma_table_,_mm_add_epi32(indx,v_setall_s32(1)),4);
        v = v_add_f32(v_mul_f32(w0,p0),v_mul_f32(w1,p1));
#endif                

        v_store_f32(p,v);
    }
#endif

    for(;i<N;i++,p++) {
        float v = *p;
        float vf = v*(gamma_table_size-1);
        int indx = std::max(0,std::min(gamma_table_size-1,int(vf)));
        float w1 = vf-indx;
        float w0 = 1.0f - w1;
        v = gamma_table_[indx]*w0 + gamma_table_[indx+1]*w1;
        *p = v;
    }
}

int main()
{
    cv::Mat v(768,1024,CV_32FC3);
    float *p=(float *)(v.data);
    for(int round = 1;round >=0 ;round--) {
        int N=100;
        double total = 0;
        for(int samples=-5;samples<N;samples++) {
            srand(0);
            for(int i=0;i<v.rows*v.cols*3;i++)
                p[i] = (float)rand()/RAND_MAX;
            auto start = std::chrono::high_resolution_clock::now();
            if(round == 0)
                cv::pow(v,gamma_,v);
            else
                apply_gamma(v);
            auto end = std::chrono::high_resolution_clock::now();
            double time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1> > >(end-start).count();
            if(samples>=0)
                total +=time;
            else if(samples == -1) {
                srand(0);
                float md=0;
                for(int i=0;i<v.rows*v.cols*3;i++) {
                    float diff = p[i] - std::pow((float)rand()/RAND_MAX,gamma_);
                    md = std::max(md,std::abs(diff));
                }
                printf("max diff = %5.4f\n",md);
            }
        }
        printf("%s: %5.3fms\n",(round == 0 ? "pow" : "lut"),total / N * 1000);
    }
}

