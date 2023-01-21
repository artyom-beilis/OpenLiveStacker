#include "cv_util.h"
#include <immintrin.h>
#include <chrono>

namespace ols {
    Gamma::Gamma(float value)
    {
        static constexpr int Np = 256;
        float x[Np];
        float y[Np];
        gamma = value;
        roots = value < 1.0f ? 2 : ( value  <= 1.5 ? 1 : 0 );
        for(int i=0;i<Np;i++) {
            x[i] = i / (float(Np-1));
            y[i] = std::pow(x[i],value);
            if(roots>=1)
                x[i] = std::sqrt(x[i]);
            if(roots>=2)
                x[i] = std::sqrt(x[i]);
        }
        cv::Mat M(Np,N,CV_32FC1);
        for(int i=0;i<Np;i++)
            M.at<float>(i,0)=1.0f;
        for(int r=1;r<N;r++) {
            for(int c=0;c<Np;c++)
                M.at<float>(c,r) = M.at<float>(c,r-1)*x[c];
        }
        cv::Mat Y(Np,1,CV_32FC1,y);
        cv::Mat P(N,1,CV_32FC1,p);
        cv::solve(M,Y,P,cv::DECOMP_NORMAL);
    }

    template<int G>
    void Gamma::apply_v(float *v,int total) const
    {
        int i=0;
        if(__builtin_cpu_supports("avx")) {
            for(;i+7<total;i+=8) {
                __m256 x = _mm256_loadu_ps(v+i);
                if(G>=2)
                    x=_mm256_sqrt_ps(x);
                if(G>=1)
                    x=_mm256_sqrt_ps(x);
                __m256 r;
                r = _mm256_mul_ps(x,_mm256_set1_ps(p[5]));
                r = _mm256_add_ps(r,_mm256_set1_ps(p[4]));
                r = _mm256_mul_ps(x,r);
                r = _mm256_add_ps(r,_mm256_set1_ps(p[3]));
                r = _mm256_mul_ps(x,r);
                r = _mm256_add_ps(r,_mm256_set1_ps(p[2]));
                r = _mm256_mul_ps(x,r);
                r = _mm256_add_ps(r,_mm256_set1_ps(p[1]));
                r = _mm256_mul_ps(x,r);
                r = _mm256_add_ps(r,_mm256_set1_ps(p[0]));
                _mm256_storeu_ps(v+i,r);
            }
        }
        else if(__builtin_cpu_supports("sse4.2")) {
            for(;i+3<total;i+=4) {
                __m128 x = _mm_loadu_ps(v+i);
                if(G>=2)
                    x=_mm_sqrt_ps(x);
                if(G>=1)
                    x=_mm_sqrt_ps(x);
                __m128 r;
                r = _mm_mul_ps(x,_mm_set1_ps(p[5]));
                r = _mm_add_ps(r,_mm_set1_ps(p[4]));
                r = _mm_mul_ps(x,r);
                r = _mm_add_ps(r,_mm_set1_ps(p[3]));
                r = _mm_mul_ps(x,r);
                r = _mm_add_ps(r,_mm_set1_ps(p[2]));
                r = _mm_mul_ps(x,r);
                r = _mm_add_ps(r,_mm_set1_ps(p[1]));
                r = _mm_mul_ps(x,r);
                r = _mm_add_ps(r,_mm_set1_ps(p[0]));
                _mm_storeu_ps(v+i,r);
            }
        }
        else {
            // no vector extension for sqrt
            cv::Mat tmp(1,total/4*4,CV_32FC1,v);
            if(G>=2)
                cv::sqrt(tmp,tmp);
            if(G>=1)
                cv::sqrt(tmp,tmp);

            for(;i+3<total;i+=4) {
                typedef float float4 __attribute__ ((vector_size(16)));
                typedef float float4u __attribute__ ((vector_size(16),aligned(4)));
                float4u *ptr = (float4u*)(v+i);
                float4 x = *ptr;
                x = ((((p[5]*x+p[4])*x+p[3])*x+p[2])*x+p[1])*x+p[0];
                *ptr = x;
            }
        }

        if(total > 0) {
            cv::Mat tmp(1,total-i,CV_32FC1,v+i);
            if(G>=2)
                cv::sqrt(tmp,tmp);
            if(G>=1)
                cv::sqrt(tmp,tmp);
            for(;i<total;i++) {
                float x = v[i];
                v[i] = ((((p[5]*x+p[4])*x+p[3])*x+p[2])*x+p[1])*x+p[0];
            }
        }
    }
    
    void Gamma::apply(cv::Mat &m) const
    {

        if((m.type() & CV_MAT_DEPTH_MASK) == CV_32F && m.isContinuous()) {
            int total = m.rows*m.cols*m.channels();
            float *v = reinterpret_cast<float*>(m.data);
            if(roots==2)
                apply_v<2>(v,total);
            else if(roots==1)
                apply_v<1>(v,total);
            else 
                apply_v<0>(v,total);
        }
        else {
            cv::pow(m,gamma,m);
        }
    }
}

#ifdef GAMMA_TEST

int main()
{
    double gamma[]={1/3.0,1/2.4,1/2.2,1/1.7,1/1.5,1/1.3,1.0,1.3,1.5,1.7,2.2,2.4,3.0};

    for(auto g: gamma) {
        ols::Gamma c(g);
        cv::Mat a(19,17,CV_32FC3);
        cv::Mat ref;
        float *data=(float*)(a.data);
        int N=a.rows*a.cols * 3;
        for(int i=0;i<N;i++) {
            data[i] = (float)(i)/(N-1);
            cv::pow(a,g,ref);
        }
        float *rdata = (float*)(ref.data);
        c.apply(a);
        float diff = 0;
        for(int i=0;i<N;i++) {
            diff = std::max(diff,std::abs(data[i]-rdata[i]));
        }
        printf("g=%f d=%f\n",g,diff);
    }

    for(int test_case=0;test_case<2;test_case++) {

        float test_gamma = test_case== 0 ? 1/2.2 : 2.2;

        ols::Gamma gm(test_gamma);

        cv::Mat img(1080,1920,CV_32FC3);
        for(int i=0;i<img.rows*img.cols*3;i++) {
            ((float*)(img.data))[i] = (float)rand() / RAND_MAX;
        }
        
        auto start_ts = std::chrono::high_resolution_clock::now();
        int times=100;
        for(int i=-times/10;i<times;i++) {
            if(i==0)
                start_ts = std::chrono::high_resolution_clock::now();
            cv::Mat tmp = img.clone();
            gm.apply(tmp);
        }
        auto end_ts = std::chrono::high_resolution_clock::now();
        float time = std::chrono::duration_cast<std::chrono::duration<float, std::ratio<1> > >(end_ts-start_ts).count() / times * 1e3;
        printf("Updated gamma=%5.3f ms\n",time);
        start_ts = std::chrono::high_resolution_clock::now();
        for(int i=-times/10;i<times;i++) {
            if(i==0)
                start_ts = std::chrono::high_resolution_clock::now();
            cv::Mat tmp = img.clone();
            cv::pow(tmp,test_gamma,tmp);
        }
        end_ts = std::chrono::high_resolution_clock::now();
        time = std::chrono::duration_cast<std::chrono::duration<float, std::ratio<1> > >(end_ts-start_ts).count() / times * 1e3;
        printf("cv::pow gamma=%5.3f ms\n",time);
    }
}


#endif
