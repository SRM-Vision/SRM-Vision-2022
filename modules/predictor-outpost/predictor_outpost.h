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
#include "predictor-armor-debug/predictor_armor_debug.h"

#include <utility>
#define PREDICTOR_OUTPOST_H_
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
        debug::Trackbar<double>::Instance().AddTrackbar("shoot_distance:",
                                                        "outpost",
                                                        advanced_distance,
                                                        150);
        debug::Trackbar<double>::Instance().AddTrackbar("delay_time:",
                                                        "outpost",
                                                        delay_time_,
                                                        1);
    }
    ~OutpostPredictor() = default;
    SendPacket Run(DetectedData detected_data, float bullet_speed = 16);
    void GetROI(cv::Rect &roi_rect, const cv::Mat &src_image);
private:

    const double kRotateSpeed_ = 0.4;  ///< round/s
    const double kCommunicationTime_ = 0.03; ///< m/s


    //ROI
    cv::Point2f roi_corners_[4]{};
    int armor_num = 0;
    int buff = 0;


    //
    double delta_pitch_ = 0;
    double advanced_distance = 100;
    OutputData output_data_{};

    bool ready_ = false;
    std::chrono::high_resolution_clock::time_point ready_time_{};
    double delay_time_=0.0;
};

#endif  // PREDICTOR_OUTPOST_H_
