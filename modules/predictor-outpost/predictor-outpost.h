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
#include "debug-tools/trackbar.h"
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
    explicit OutpostPredictor()
    {
        debug::Trackbar<double>::Instance().AddTrackbar("outpost pitch:",
                                                        "outpost",
                                                        delta_pitch_,
                                                        1);
    }
    ~OutpostPredictor() = default;
    void GetFromDetector(SendToOutpostPredictor send_to_outpost_predictor);
    SendPacket Run();
private:
    const double kRotateSpeed_ = 0.4;  ///< round/s
    const double kCommunicationTime_ = 0.03; ///< m/s

    double bullet_speed_ = 0;   ///< m/s
    double delta_pitch_ = 0;
    int clockwise_ = 0;
    OutputData output_data_{};
    double center_distance_ = 0;
    cv::Point2f outpost_center_ = cv::Point2f (0.0, 0.0);
    cv::Point2f going_center_ = cv::Point2f (0.0, 0.0);
    cv::Point2f coming_center_ = cv::Point2f (0.0, 0.0);
    coordinate::TranslationVector shoot_point;
};

#endif  // PREDICTOR_OUTPOST_H_
