/**
 * Outpost predictor class header.
 * \author Lzy20020320 LIYunzhe1408
 * \date 2022.7.13
 */


#ifndef PREDICTOR_OUTPOST_NEW_H_
#define PREDICTOR_OUTPOST_NEW_H_
#include "math-tools/algorithms.h"
#include <data-structure/communication.h>
#include "../digital-twin/battlefield.h"
#include "debug-tools/painter.h"
#include "predictor-spin/spin_predictor.h"
#include "predictor-armor/predictor_armor_debug.h"
#include "predictor_outpost_debug.h"
#include "lang-feature-extension/attr-reader.h"
#include "trajectory-solver/trajectory-solver.h"

/**
 * \Brief Find the center shooting point of outpost, auto-shoot after a time_delay.
 * @Details Set color first, then run predictor to auto-shoot and clear all parameters in controller if the mode changes.
 */
class OutpostPredictor {
public:
    OutpostPredictor() = default;

    ~OutpostPredictor() = default;

    /**
     * \Brief Load params and choice debug or not.
     * \param config_path the config path of outpost params data file.
     * \param debug Choice debug or not.
     * \return Whether initialized successfully.
     */
    bool Initialize();

    /**
     * \Brief Collect armors, get center points and decide auto-shoot signal.
     * \param battlefield
     * \param bullet_speed
     * \return 'SendPacket' to send information to EC.
     */
    SendPacket OldRun(Battlefield battlefield);
    SendPacket NewRun(Battlefield battlefield, const float& bullet_speed,  int width,const std::chrono::steady_clock::time_point& time);
    SendPacket Run(Battlefield battlefield, const float& bullet_speed, cv::MatSize frame_size,const std::chrono::steady_clock::time_point& time);

    /**
    * \Brief Set the color of outpost.
    * \param enemy_color Enemy color.
    */
    inline void SetColor(const Entity::Colors &enemy_color) { enemy_color_ = enemy_color; }

    /**
    * \Brief Clear the information in OutpostPredictor.
    */
    void Clear(){
        checked_clockwise_ = false;
        clockwise_ = 0;  ///< 1 (rotate left) or -1 (rotate right)
        last_armor_x_ = 0 ;
        ready_fire_ = false;
        prepared_ = false;
        need_init_ = true;
        fire_ = false;  ///< only used to show the image in the image when debug
        biggest_area_ = 0;
        shoot_delay_time_ = 0;
        roi_buff_ = 0;
        aim_buff_ = 0;
    };

    /**
    * \Brief get the roi.
    * \param [out]roi_rect.
    * \param src_image the image;
    */
    cv::Rect GetROI(const cv::Mat &src_image);

    ATTR_READER(outpost_center_, OutpostCenter);

    ATTR_READER(fire_, Fire);

private:

    /**
     * \Brief Find the armor with the biggest area.
     * \param [in] armors. All detected armors.
     * \return index of the armor with biggest area.
     */
    int FindBiggestArmor(const std::vector<Armor> &armors);

    /**
     * \Brief Decide the coming/going armor in different rotating cases.
     * \Details In one or two armors cases, compare 'armor center x' with 'outpost center x' to decide coming/going.
     */
    void DecideComingGoing();

    /**
     * \Brief Judge rotate direction.
     * \Details Calculate difference value of contiguous armor centers' x.
     * \Note Variable 'clockwise' is ought to be valued as 1 (rotate left) or -1 (rotate right).
     */
    void IsClockwise();


    /**
     * \Brief get the center of ROI.
     * \param armor the main armor.
     */
    void UpdateROICorners(const Armor& armor);

    /**
     * \Brief get flying time of bullet.
     * \param bullet_speed bullet speed.
     * \param armor the target.
     */
    double GetBulletFlyTime(const float& bullet_speed, const Armor& armor);

private:
    const double kFindBiggestArmorTime = 4;  ///< during this time try to find the the front of the target.
    const double kAreaThreshold = 0.93;  ///< when area is biggest than area threshold * biggest armor it is the front of the target.
    const double kAreaThresholdLow = 0.9;  ///< lower threshold to avoid can't find the front of the target.
    const double kAimBuff = 20;  ///< The num frame number to ensure the result.

    Outpost outpost_{};
    cv::Point2f outpost_center_{};  ///< only used to show the point in the image when debug
    std::chrono::high_resolution_clock::time_point start_time_{};
    std::chrono::high_resolution_clock::time_point ready_time_{};

    Entity::Colors enemy_color_{};
    bool checked_clockwise_{false};
    int clockwise_{0};  ///< 1 (rotate left) or -1 (rotate right)

    double last_armor_x_{0};

    bool ready_fire_{false};
    bool prepared_{false};
    bool need_init_{true};
    bool fire_{false};  ///< only used to show the image in the image when debug

    double biggest_area_{0};
    double shoot_delay_time_{0.52};

    int roi_buff_;
    cv::Point2f roi_corners_[4];

    int aim_buff_{0};
};

#endif //PREDICTOR_OUTPOST_H_
