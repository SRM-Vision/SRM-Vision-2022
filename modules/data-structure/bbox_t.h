/**
 * 4-point data model header.
 * \author anonymity
 * \date 2022.1.28
 */

#ifndef BBOX_T_H_
#define BBOX_T_H_

#include <opencv2/core/types.hpp>

/// \brief 4-point model structure for detection result.
struct alignas(4) bbox_t {
    cv::Point2f points[4];
    float confidence;
    int color;  ///< 0: blue, 1: red, 2: grey, 3: purple.
    int id;  ///< 0: sentry, 1-5: cars, 6: base.

    inline bool operator==(const bbox_t &bbox) const {
        return (
                points[0] == bbox.points[0] && points[1] == bbox.points[1]
                && points[2] == bbox.points[2] && points[3] == bbox.points[3]
                && confidence == bbox.confidence
                && color == bbox.color
                && id == bbox.id);
    }

    inline bool operator!=(const bbox_t &bbox) const {
        return (
                points[0] != bbox.points[0] || points[1] != bbox.points[1]
                || points[2] != bbox.points[2] || points[3] != bbox.points[3]
                || confidence != bbox.confidence
                || color != bbox.color
                || id != bbox.id);
    }
};

#endif  // BBOX_T_H_
