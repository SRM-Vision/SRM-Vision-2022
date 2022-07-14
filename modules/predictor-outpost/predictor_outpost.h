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
#include "predictor-armor/spin_detector.h"
#include "predictor-armor/predictor_armor_debug.h"
#include "predictor_outpost_debug.h"


/**
 * \Brief Find the center shooting point of outpost, auto-shoot after a time_delay.
 * @Details Set color first, then run predictor to auto-shoot and clear all parameters in controller if the mode changes.
 */
class OutpostPredictor{
public:
    OutpostPredictor() = default;
    ~OutpostPredictor() = default;

    /**
     * \Brief Load params and choice debug or not.
     * @param config_path the config path of outpost params data file.
     * @param debug Choice debug or not.
     * @return Whether initialized successfully.
     */
    bool Initialize();

    /**
     * \Brief Collect armors, get center points and decide auto-shoot signal.
     * @param battlefield
     * @param bullet_speed
     * @return 'SendPacket' to send information to EC.
     */
    SendPacket Run(Battlefield battlefield, float bullet_speed = 16);

    /**
    * \Brief Set the color of outpost.
    * \param enemy_color Enemy color.
    */
    void SetColor(const Entity::Colors &enemy_color) { enemy_color_ = enemy_color; }

    /**
    * \Brief Clear the information in OutpostPredictor.
    */
    void Clear();

private:

    /**
     * \Brief Find the armor with the biggest area.
     * @param [in] armors. All detected armors.
     * @return index of the armor with biggest area.
     */
    int FindBiggestArmor(const std::vector<Armor> &armors);

    /**
     * \Brief Decide the coming/going armor in different rotating cases.
     * @Details In one or two armors cases, compare 'armor center x' with 'outpost center x' to decide coming/going.
     */
    void DecideComingGoing();

    /**
     * \Brief Judge rotate direction.
     * @Details Calculate difference value of contiguous armor centers' x.
     * @Note Variable 'clockwise' is ought to be valued as 1 (rotate left) or -1 (rotate right).
     */
    void IsClockwise();

private:
    const double kFindBiggestArmorTime = 4;  ///< during this time try to find the the front of the target.
    const double kAreaThreshold = 0.93;  ///< when area is biggest than area threshold * biggest armor it is the front of the target.
    const double kAreaThresholdLow = 0.9;  ///< lower threshold to avoid can't find the front of the target.
    const double kAimBuff = 20;  ///< The num frame number to ensure the result.

    Outpost outpost_{};

    std::chrono::high_resolution_clock::time_point start_time_{};
    std::chrono::high_resolution_clock::time_point ready_time_{};

    Entity::Colors enemy_color_{};
    bool checked_clockwise_{false};
    int clockwise_{0};  ///< 1 (rotate left) or -1 (rotate right)

    double last_armor_x_{0};

    bool ready_fire_{false};
    bool prepared_{false};
    bool need_init_{true};

    double biggest_area_{0};
    double shoot_delay_time_{0};

    int aim_buff_{0};
};

#endif //PREDICTOR_OUTPOST_H_
