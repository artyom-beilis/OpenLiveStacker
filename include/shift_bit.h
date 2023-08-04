#pragma once
#ifdef USE_CV_SIMD
#ifdef USE_ARM_SIMD
#include <arm_neon.h>
#else
#include <immintrin.h>
#include <emmintrin.h>
#endif
#endif


namespace ols {

#ifdef USE_CV_SIMD
    #ifdef USE_ARM_SIMD
    template<int N>
    inline int shift_to16_fixed_arm(unsigned short *data,int lim)
    {
        int i=0;
        for(;i<lim;i+=8) {
            uint16x8_t v = vld1q_u16(data+i);
            v= vshlq_n_u16(v,N);
            vst1q_u16(data+i,v);
        }
        return i;
    }
    #endif
#endif        

    inline void shift_to16(int bits,unsigned short *data,int n) 
    {
        int shift = 16 - bits;
        int i=0;
#ifdef USE_CV_SIMD
        int lim = n/8*8;
        #ifdef USE_ARM_SIMD
        switch(n) {
        case 1: i = shift_to16_fixed_arm<1>(data,lim); break;
        case 2: i = shift_to16_fixed_arm<2>(data,lim); break;
        case 3: i = shift_to16_fixed_arm<3>(data,lim); break;
        case 4: i = shift_to16_fixed_arm<4>(data,lim); break;
        case 5: i = shift_to16_fixed_arm<5>(data,lim); break;
        case 6: i = shift_to16_fixed_arm<6>(data,lim); break;
        case 7: i = shift_to16_fixed_arm<7>(data,lim); break;
        }
        #else            
        __m128i vs = _mm_set1_epi64x(shift);
        for(;i<lim;i+=8) {
            auto v = _mm_loadu_si128((__m128i*)(data+i));
            v = _mm_sll_epi16(v,vs);
            _mm_storeu_si128((__m128i*)(data+i),v);
        }
        #endif
#endif        
        for(;i<n;i++) {
            data[i]<<=shift;
        }
    }
}

