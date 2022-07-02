//
// Created by lzy on 2022/6/30.
//

#ifndef NEW_PREDICTOR_OUTPOST_H_
#define NEW_PREDICTOR_OUTPOST_H_

#include "data-structure/buffer.h"
#include "data-structure/communication.h"
#include "digital-twin/facilities/outpost.h"
#include "detector-outpost/detector_outpost.h"
#include <cmath>
#include <utility>
#include "debug-tools/trackbar.h"
struct NewOutputData {
    void Update(const coordinate::TranslationVector& shoot_point);

    float yaw;
    float pitch;
    float delay;
    int fire;
};

class NewOutpostPredictor {
public:

    explicit NewOutpostPredictor():going_center3d_(0,0,0),coming_center3d_(0,0,0),center_3d_(0,0,0)
    {
        debug::Trackbar<double>::Instance().AddTrackbar("outpost pitch:",
                                                        "outpost",
                                                        delta_pitch_,
                                                        1);
        debug::Trackbar<double>::Instance().AddTrackbar("shoot_distance:",
                                                        "outpost",
                                                        advanced_distance,
                                                        150);
    }
    ~NewOutpostPredictor() = default;
    void GetFromDetector(SendToOutpostPredictor send_to_outpost_predictor);
    SendPacket Run();
private:
    const double kRotateSpeed_ = 0.4;  ///< round/s
    const double kCommunicationTime_ = 0.03; ///< m/s

    double bullet_speed_ = 0;   ///< m/s
    double delta_pitch_ = 0;
    double advanced_distance = 100;
    int clockwise_ = 0;
    NewOutputData output_data_{};
    double center_distance_ = 0;
    cv::Point2f outpost_center_ = cv::Point2f (0.0, 0.0);   // 图中
    coordinate::TranslationVector shoot_point_;                     // 相机坐标系下
    coordinate::TranslationVector center_3d_;                       // 世界坐标系下

    cv::Point2f going_center2d_ = cv::Point2f (0.0, 0.0);
    coordinate::TranslationVector going_center3d_;              // 世界坐标系下

    cv::Point2f coming_center2d_ = cv::Point2f (0.0, 0.0);
    coordinate::TranslationVector coming_center3d_;             // 世界坐标系下


};


#endif //NEW_PREDICTOR_OUTPOST_H_
