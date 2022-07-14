/**
 * Outpost predictor class header.
 * \author Lzy20020320
 * \date 2022.7.13
 */


#ifndef PREDICTOR_OUTPOST_NEW_H_
#define PREDICTOR_OUTPOST_NEW_H_


#include <data-structure/communication.h>
#include "../digital-twin/battlefield.h"
#include "debug-tools/painter.h"
#include "predictor-armor/spin_detector.h"
#include "predictor-armor-debug/predictor_armor_debug.h"
#include "predictor-outpost-debug/predictor_outpost_debug.h"

class OutpostPredictor{
public:
    OutpostPredictor() = default;
    ~OutpostPredictor() = default;

    SendPacket Run(Battlefield battlefield, float bullet_speed = 16);

    /**
    * \brief Set the color of outpost.
    * \param enemy_color Enemy color.
    */
    void SetColor(const Entity::Colors &enemy_color) { enemy_color_ = enemy_color; }

    /**
    * \brief Clear the information in OutpostPredictor.
    */
    void Clear();

private:
    /**
     * \brief Find the armor with the biggest area.
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

    Outpost outpost_{};

    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point ready_time_{};

    Entity::Colors enemy_color_;
    bool checked_clockwise_ = false;
    int clockwise_          = 0;       ///< 1 (rotate left) or -1 (rotate right)

    double last_armor_x_{};

    bool ready_fire_ = false;
    bool prepared_ = false;
    bool need_init_ = true;

    double biggest_area_ = 0;
    double shoot_delay_time_   = 0;

    int buff{};
};

#endif //PREDICTOR_OUTPOST_H_
