/**
 * SIMD math accelerated functions.
 * @author trantuan-20048607
 * @date 2022.2.28
 * @note Only supports ARM NEON and intel SSE2.
 */

#ifndef SIMD_H_
#define SIMD_H_

#include <cmath>
#include <opencv2/core/types.hpp>

#if defined(__x86_64__)

#include <emmintrin.h>  // SSE 2.

#endif

#if defined(__aarch64__)

#include "sse2neon/sse2neon.h"  // Translate SSE to NEON.

#endif

#if defined(__x86_64__) | defined(__aarch64__)
#define USE_SSE2

#include "sse-math/sse_math_extension.h"

#endif

namespace simd {
    /**
     * @brief Calculate both sine and cosine for 4 floats at the same time.
     * @param [in] x Input 4 floats.
     * @param [out] s Output sine value.
     * @param [out] c Output cosine value.
     */
    [[maybe_unused]] inline void SinCosFloatX4(const float x[4], float s[4], float c[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
        v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, s_v4sf, c_v4sf;
        sincos_ps(x_v4sf, &s_v4sf, &c_v4sf);
        auto s_ptr = (float *) &s_v4sf, c_ptr = (float *) &c_v4sf;
        s[0] = *s_ptr++;
        c[0] = *c_ptr++;
        s[1] = *s_ptr++;
        c[1] = *c_ptr++;
        s[2] = *s_ptr++;
        c[2] = *c_ptr++;
        s[3] = *s_ptr;
        c[3] = *c_ptr;
#else
        for (auto i = 0; i < 4; i++) {
            s[i] = std::sin(x[i]);
            c[i] = std::cos(x[i]);
        }
#endif
    }

    /**
     * @brief Calculate sine for 4 floats at the same time.
     * @param [in,out] x Input 4 floats and will be output.
     */
    [[maybe_unused]] inline void SinFloatX4(float x[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
        v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, s_v4sf = sin_ps(x_v4sf);
        auto s_ptr = (float *) &s_v4sf;
        x[0] = *s_ptr++;
        x[1] = *s_ptr++;
        x[2] = *s_ptr++;
        x[3] = *s_ptr;
#else
        for (auto i = 0; i < 4; i++)
            x[i] = std::sin(x[i]);
#endif
    }

    /**
     * @brief Calculate sine value.
     * @param [in] x Input x.
     * @return Output sin(x).
     */
    [[maybe_unused]] inline float SinFloat(float x) {
#if defined(__x86_64__) | defined(__aarch64__)
        v4sf x_v4sf = {x}, s_v4sf = sin_ps(x_v4sf);
        return *(float *) &s_v4sf;
#else
        return std::sin(x);
#endif
    }

    /**
     * @brief Calculate cosine for 4 floats at the same time.
     * @param [in,out] x Input 4 floats and will be output.
     */
    [[maybe_unused]] inline void CosFloatX4(float x[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
        v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, c_v4sf = cos_ps(x_v4sf);
        auto c_ptr = (float *) &c_v4sf;
        x[0] = *c_ptr++;
        x[1] = *c_ptr++;
        x[2] = *c_ptr++;
        x[3] = *c_ptr;
#else
        for (auto i = 0; i < 4; i++)
            x[i] = std::cos(x[i]);
#endif
    }

    /**
     * @brief Calculate cosine value.
     * @param [in] x Input x.
     * @return Output cos(x).
     */
    [[maybe_unused]] inline float CosFloat(float x) {
#if defined(__x86_64__) | defined(__aarch64__)
        v4sf x_v4sf = {x}, c_v4sf = cos_ps(x_v4sf);
        return *(float *) &c_v4sf;
#else
        return std::cos(x);
#endif
    }

    /**
     * @brief Calculate tangent for 4 floats at the same time.
     * @param [in,out] x Input 4 floats and will be output.
     */
    [[maybe_unused]] inline void TanFloatX4(float x[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
        v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, t_v4sf = tan_ps(x_v4sf);
        auto t_ptr = (float *) &t_v4sf;
        x[0] = *t_ptr++;
        x[1] = *t_ptr++;
        x[2] = *t_ptr++;
        x[3] = *t_ptr;
#else
        for (auto i = 0; i < 4; i++)
            x[i] = std::tan(x[i]);
#endif
    }

    /**
     * @brief Calculate cotangent for 4 floats at the same time.
     * @param [in,out] x Input 4 floats and will be output.
     */
    [[maybe_unused]] inline void CotFloatX4(float x[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
        v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, t_v4sf = cot_ps(x_v4sf);
        auto t_ptr = (float *) &t_v4sf;
        x[0] = *t_ptr++;
        x[1] = *t_ptr++;
        x[2] = *t_ptr++;
        x[3] = *t_ptr;
#else
        for (auto i = 0; i < 4; i++)
            x[i] = 1.f / std::tan(x[i]);
#endif
    }

    /**
     * @brief Calculate arc tangent for 4 floats at the same time.
     * @param [in,out] x Input 4 floats and will be output.
     */
    [[maybe_unused]] inline void AtanFloatX4(float x[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
        v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, t_v4sf = atan_ps(x_v4sf);
        auto t_ptr = (float *) &t_v4sf;
        x[0] = *t_ptr++;
        x[1] = *t_ptr++;
        x[2] = *t_ptr++;
        x[3] = *t_ptr;
#else
        for (auto i = 0; i < 4; i++)
            x[i] = std::atan(x[i]);
#endif
    }

    /**
     * @brief Calculate arc tangent II for 4 floats at the same time.
     * @param [in] y Input 4 ys.
     * @param [in] x Input 4 xs.
     * @param [out] res Output 4 results.
     */
    [[maybe_unused]] inline void Atan2FloatX4(const float y[4], const float x[4], float res[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
        v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, y_v4sf = {y[0], y[1], y[2], y[3]};
        v4sf res_v4sf = atan2_ps(y_v4sf, x_v4sf);
        auto res_ptr = (float *) &res_v4sf;
        res[0] = *res_ptr++;
        res[1] = *res_ptr++;
        res[2] = *res_ptr++;
        res[3] = *res_ptr;
#else
        for (auto i = 0; i < 4; i++)
            res[i] = std::atan2(y[i], x[i]);
#endif
    }

    /**
     * @brief Calculate arc tangent II for single float at the same time.
     * @param [in] y Input y.
     * @param [in] x Input x.
     * @return atan2(y, x).
     */
    [[maybe_unused]] inline float Atan2Float(float y, float x) {
#if defined(__x86_64__) | defined(__aarch64__)
        return atan2_ref(y, x);
#else
        return std::atan2(y, x);
#endif
    }

    /**
     * @brief Calculate square root of single float value.
     * @param [in] x Input x.
     * @return sqrt(x).
     */
    [[maybe_unused]] inline float SqrtFloat(float x) {
#if defined(__x86_64__) | defined(__aarch64__)
        return sqrt_ps(x);
#else
        return std::sqrt(x);
#endif
    }

    /**
     * @brief Calculate reversed square root of single float value, note that it's faster than 0x5f3759df method.
     * @param [in] x Input x.
     * @return 1/sqrt(x).
     */
    [[maybe_unused]] inline float RsqrtFloat(float x) {
#if defined(__x86_64__) | defined(__aarch64__)
        return rsqrt_ps(x);
#else
        return 1 / std::sqrt(x);
#endif
    }
}

#endif  // SIMD_H_
