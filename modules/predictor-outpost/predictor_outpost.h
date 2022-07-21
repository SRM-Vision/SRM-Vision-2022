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
#include "compensator/compensator.h"
#include "predictor-armor/filter.h"

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
     * \param yaw_pitch_roll data form gyroscope.
     * \return 'SendPacket' to send information to EC.
     */
    SendPacket StaticOutpostRun(const Battlefield &battlefield, std::array<float, 3> yaw_pitch_roll);


    /**
     * \Brief Control the pitch of Hero and auto fire in appropriate time;
     * \param battlefield
     * \param bullet_speed
     * \param width the width of the image.
     * \param yaw_pitch_roll data form gyroscope.
     * \param time used to calculate the cost time of calculating.
     * \return 'SendPacket' to send information to EC.
     */
    SendPacket SpinningOutpostRun(const Battlefield &battlefield, const float &bullet_speed, int width,
                                  const std::array<float, 3> &yaw_pitch_roll,
                                  const std::chrono::steady_clock::time_point &time);

//    SendPacket Run(Battlefield battlefield, const float &bullet_speed, cv::MatSize frame_size,
//                   const float &real_pitch,
//                   const std::chrono::steady_clock::time_point &time);


    /**
    * \Brief Set the color of outpost.
    * \param enemy_color Enemy color.
    */
    inline void SetColor(const Entity::Colors &enemy_color) { enemy_color_ = enemy_color; }


    /**
    * \Brief Clear the information in OutpostPredictor.
    */
    void Clear() {
        ready_fire_ = false;
        fire_ = false;  ///< only used to show the image in the image when debug
        shoot_delay_time_ = 0;
        roi_buff_ = 0;
    };


    /**
    * \Brief get the roi.
    * \param src_image the image;
    * \return roi.
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
    static int FindBiggestArmor(const std::vector<Armor> &armors);

    /**
     * \Brief Get the height of outpost.
     * \param armor the main armor of outpost.
     * \param pitch the pitch data from gyroscope.
     * \return the height of outpost.
     */
    static double GetOutpostHeight(const Armor &armor, const float &pitch);

    /**
     * \Brief Use plane distance to ensure the shoot delay.
     * \param plane_distance the plane distance of outpost.
     * \return the shoot delay.
     */
    static double GetShootDelay(const double &plane_distance);

    /**
     * \Brief Choice the roi.
     * \param armor the main armor of outpost.
     * \param plane_distance the plane distance of outpost.
     * \return roi
     */
    void GetROI(const Armor &armor, const double &plane_distance);

    void Offset(float &pitch,double distance);


private:
    const double  kHeightThreshold = 0.4;  ///< the lowest height of outpost

    Entity::Colors enemy_color_{};
    Outpost outpost_{};
    cv::Rect roi_rect = {};
    cv::Point2f outpost_center_{};  ///< only used to show the point in the image when debug

    std::chrono::high_resolution_clock::time_point ready_time_{};
    FilterDTMean<double, 10> distance_filter_;

    bool ready_fire_{false};
    bool fire_{false};  ///< only used to show the image in the image when debug

    double shoot_delay_time_{0.125};
    int roi_buff_{15};

};

#endif //PREDICTOR_OUTPOST_NEW_H_
