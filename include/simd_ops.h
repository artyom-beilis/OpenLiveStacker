#pragma once 

/*
MIT License

Copyright (c) 2020 MathLib-ARM, (c) 2023 Artyom Beilis

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
         

#ifdef V_HAVE_SSE2
#include <xmmintrin.h>
#include <smmintrin.h>
#define V_SIMD
#elif defined(V_HAVE_NEON)
#include <arm_neon.h>
#define V_SIMD
#endif

namespace ols {
#ifdef V_HAVE_SSE2

    typedef __m128  v_f32;
    typedef __m128i v_s32;

    #define v_setall_f32(VAL) _mm_set1_ps(VAL)
    #define v_setall_s32(VAL) _mm_set1_epi32(VAL)

    #define v_load_f32 _mm_loadu_ps
    #define v_store_f32 _mm_storeu_ps
    inline void v_store_s32(int* p,v_s32 v) { _mm_storeu_si128((v_s32*)(p),v); }

    #define v_max_f32 _mm_max_ps
    #define v_min_f32 _mm_min_ps
    #define v_max_s32 _mm_max_epi32
    #define v_min_s32 _mm_min_epi32
    
     
    #define v_mul_f32 _mm_mul_ps
    #define v_add_f32 _mm_add_ps
    #define v_sub_f32 _mm_sub_ps

    #define v_floor_f32 _mm_floor_ps
    #define v_cvt_f32_s32 _mm_cvtps_epi32

#elif defined V_HAVE_NEON

    typedef float32x4_t v_f32;
    typedef int32x4_t   v_s32;

    #define v_setall_f32    vdupq_n_f32
    #define v_setall_s32    vdupq_n_s32

    #define v_load_f32 vld1q_f32 
    #define v_store_f32 vst1q_f32
    #define v_store_s32 vst1q_s32

    #define v_max_f32 vmaxq_f32
    #define v_min_f32 vminq_f32
    #define v_max_s32 vmaxq_s32
    #define v_min_s32 vminq_s32
    
     
    #define v_mul_f32  vmulq_f32
    #define v_add_f32  vaddq_f32
    #define v_sub_f32  vsubq_f32

    #define v_floor_f32 vrndmq_f32
    #define v_cvt_f32_s32 vcvtq_s32_f32

#endif
} // OLS


