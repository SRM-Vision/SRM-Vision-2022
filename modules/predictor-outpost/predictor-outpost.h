/**
 * Outpost predictor definition header.
 * \author LIYunzhe1408
 * \date 2022.5.2
 */
#ifndef PREDICTOR_OUTPOST_H_
#define PREDICTOR_OUTPOST_H_
#include "data-structure/buffer.h"
#include "data-structure/communication.h"
#include "digital-twin/facilities/outpost.h"
#include "detector-outpost/detector_outpost.h"

#include <utility>

struct OutputData {
    void Update(const coordinate::TranslationVector& shoot_point);

    float yaw;
    float pitch;
    float delay;
    int fire;
};

class OutpostPredictor
{
public:
    explicit OutpostPredictor(SendToOutpostPredictor send_to_outpost_predictor):
                                                clockwise_(send_to_outpost_predictor.is_clockwise),
                                                outpost_center_(std::move(send_to_outpost_predictor.outpost_center)),
                                                going_center_(std::move(send_to_outpost_predictor.going_center_point)),
                                                coming_center_(std::move(send_to_outpost_predictor.coming_center_point)),
                                                center_distance_(send_to_outpost_predictor.center_distance),
                                                bullet_speed_(send_to_outpost_predictor.bullet_speed),
                                                shoot_point(send_to_outpost_predictor.shoot_point){}
    ~OutpostPredictor() = default;
    SendPacket Run();
private:
    const double kRotateSpeed_ = 0.4;  ///< round/s
    const double kCommunicationTime_ = 0.03; ///< m/s

    double bullet_speed_;   ///< m/s
    int clockwise_;
    OutputData output_data_{};
    double center_distance_;
    cv::Point2f outpost_center_;
    cv::Point2f going_center_;
    cv::Point2f coming_center_;
    coordinate::TranslationVector shoot_point;
};

#endif  // PREDICTOR_OUTPOST_H_
