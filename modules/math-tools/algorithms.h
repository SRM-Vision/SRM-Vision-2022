/**
 * Algorithms library header.
 * @author trantuan-20048607
 * @date 2022.2.28
 */

#ifndef ALGORITHMS_H_
#define ALGORITHMS_H_

#include <cmath>
#include <opencv2/core/types.hpp>
#include "simd.h"

namespace algorithm {
    using namespace simd;

    /**
     * @brief Calculate area of N-side polygon.
     * @param [in] points Input vector, should in clockwise order.
     * @return Area value.
     */
    [[maybe_unused]] inline double PolygonArea(const std::vector<cv::Point2f> &points) {
        auto n = points.size();
        double area = 0;
        for (auto i = 0; i < n; ++i) {
            auto j = (i + 1) % n;
            area += points[i].x * points[j].y;
            area -= points[i].y * points[j].x;
        }
        area *= 0.5;
        return (area < 0 ? -area : area);
    }

    /**
     * @brief Calculate area of N-side polygon.
     * @tparam N Size of input array.
     * @param [in] points Input array, should in clockwise order.
     * @return Area value.
     */
    template<unsigned int N>
    [[maybe_unused]] double PolygonArea(const cv::Point2f points[N]) {
        double area = 0;
        for (auto i = 0; i < N; ++i) {
            auto j = (i + 1) % N;
            area += points[i].x * points[j].y;
            area -= points[i].y * points[j].x;
        }
        area *= 0.5;
        return (area < 0 ? -area : area);
    }

    /**
     * @brief Calculate angle between two 2D vectors.
     * @param [in] vector_a Vector A.
     * @param [in] vector_b Vector B.
     * @return Angle of two vectors, in DEGREE.
     */
    [[maybe_unused]] inline float VectorAngle(const cv::Point2f &vector_a, const cv::Point2f &vector_b) {
        constexpr float rad2deg = 180.0 / CV_PI;
        return std::acos(std::max(-1.f, std::min(
                1.f, float(vector_a.dot(vector_b))
                     * RsqrtFloat(vector_a.x * vector_a.x + vector_a.y * vector_a.y)
                     * RsqrtFloat(vector_b.x * vector_b.x + vector_b.y * vector_b.y))))
               * rad2deg;
    }

    /**
     * @brief Convert time from nanoseconds to seconds.
     * @param time Time in nanoseconds.
     * @return Time in seconds ind double.
     */
    inline double NanoSecondsToSeconds(const uint64_t &time) {
        return static_cast<double>(time) * 1e-9;
    }

    /**
     * @brief Calculate time from start to end (ns), return the time in seconds.
     * @param start Start time, ns.
     * @param end End time, ns.
     * @return Duration in seconds.
     */
    inline double NanoSecondsToSeconds(const uint64_t &start, const uint64_t &end) {
        return static_cast<double>(end - start) * 1e-9;
    }

    /**
     * @brief Calculate the shortest angular distance in [-pi, pi].
     * @param from Start angle, RAD.
     * @param to End angle, RAD.
     * @return The shortest angular distance, RAD.
     */
    inline double ShortestAngularDistance(double from, double to) {
        const double result = fmod(to - from + M_PI, 2 * M_PI);
        return result <= 0 ? result + M_PI : result - M_PI;
    }
}

#endif  // ALGORITHMS_H_
