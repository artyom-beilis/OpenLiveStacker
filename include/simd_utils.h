#pragma once

namespace ols  {
#ifdef USE_CV_SIMD

/// apply plicewise linear interpoalation, v in range [0,1], table of size [size+1]

__attribute__((always_inline)) inline void curve_simd(cv::v_float32x4 &v,int size,float *table)
{ 
    cv::v_float32x4 vf = v * cv::v_setall_f32(size - 1.0f); 
    cv::v_int32x4 indx = cv::v_floor(vf); 
    indx = cv::v_min(cv::v_setall_s32(size-1),cv::v_max(cv::v_setzero_s32(),indx)); 
    cv::v_float32x4 w1 = vf - cv::v_cvt_f32(indx); 
    cv::v_float32x4 w0 = cv::v_setall_f32(1.0f) - w1; 

    int indexes[4]; 
    float p0[4],p1[4]; 
    cv::v_store(indexes,indx); 

    p0[0] = table[indexes[0]]; 
    p1[0] = table[indexes[0]+1]; 
    p0[1] = table[indexes[1]]; 
    p1[1] = table[indexes[1]+1]; 
    p0[2] = table[indexes[2]]; 
    p1[2] = table[indexes[2]+1]; 
    p0[3] = table[indexes[3]]; 
    p1[3] = table[indexes[3]+1]; 
    v = w0 * cv::v_load(p0) + w1 * cv::v_load(p1); 
}

#endif

/// table of size M+1
__attribute__((always_inline)) inline float curve_one(float v,int M,float *table)
{
    float vf = v*(M-1);
    int indx = std::max(0,std::min(M-1,int(vf)));
    float w1 = vf-indx;
    float w0 = 1.0f - w1;
    v = table[indx]*w0 + table[indx+1]*w1;
    return v;
}

// table of size M+1
inline void prepare_power_curve(int gamma_table_size,float *table,float pw)
{
    float t_factor = 1.0f / (gamma_table_size - 1);
    for(int i=0;i<gamma_table_size;i++) {
        table[i] = i * t_factor;
    }
    table[gamma_table_size] = 1.0f;
    cv::Mat tmp(1,gamma_table_size,CV_32FC1,table);
    cv::pow(tmp,pw,tmp);
}

inline void prepare_asinh_curve(int gamma_table_size,float *table,float pw)
{
    float half_point = std::pow(0.5f,1.0f/pw);
    float a = 1.0f/half_point * std::sinh(std::acosh(1.0f/(2.0f*half_point)));
    float t_factor = 1.0f / (gamma_table_size - 1);
    for(int i=0;i<gamma_table_size;i++) {
        table[i] = i * t_factor;
    }
    table[gamma_table_size] = 1.0f;
    if(a == 1)
        return;
    float factor = 1.0f / std::asinh(a);
    for(int i=1;i<gamma_table_size;i++) {
        table[i] = factor * std::asinh(table[i] * a);
    }
}




}
