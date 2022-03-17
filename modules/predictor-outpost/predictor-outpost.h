/**
 * Outpost predictor definition header.
 * \author LIYunzhe1408
 * \date 2022.3.14
 */
#ifndef SRM_VISION_2022_PREDICTOR_OUTPOST_H
#define SRM_VISION_2022_PREDICTOR_OUTPOST_H

#include "data-structure/serial_data.h"
#include "digital-twin/battlefield.h"
#include "digital-twin/components/armor.h"
#include "math-tools/algorithms.h"
#include "digital-twin/facility.h"

class OutpostPredictor
{
public:
    ATTR_READER_REF(predict_shoot_center_, TranslationVectorCamPredict);
    OutpostPredictor();

    SendPacket Run(const Battlefield& battlefield);
private:
    std::vector<Armor> TargetArmorFilter(const std::vector<Armor> &potential_target);
    coordinate::TranslationVector CalculateMovingTargetCenter(const coordinate::TranslationVector & moving_center, int detect_num);
    coordinate::TranslationVector CalculatePredictShootCenter(const coordinate::TranslationVector & target_moving_center
                                                              , float bullet_speed, const Eigen::Quaternionf &quaternion
                                                              , float distance);
    void DecideGoingAndComing(const std::vector<Armor> & target);

    // Control and Robot properties
    const float kControl_delay_ = 1;             // TODO
    float shoot_delay_ = 0;

    //  Outpost properties
    Outpost::Colors color_;
    const Facility::FacilityTypes kType_ =  Facility::kOutpost;
    const double kArmor_included_rad_ = CV_2PI / 3;
    const float kRotate_angular_speed_ = 0.4 * CV_2PI;
    int is_clockwise = 0;                        // anti_clockwise = -1, clockwise = 1
    double clockwise_assist = 0;
    const int kCollectNum = 10;


    // Send
    SendPacket send_packet;

    // Actual
    coordinate::TranslationVector target_moving_center_;

    // Predict
    coordinate::TranslationVector predict_shoot_center_;


    // Detect 1
    std::vector<Armor> target_;

    // Detect 2
    Armor going_armor_;
    Armor coming_armor_;
};

#endif //SRM_VISION_2022_PREDICTOR_OUTPOST_H
