//
// Created by lzy on 2022/7/13.
//

#ifndef PREDICTOR_OUTPOST_NEW_H_
#define PREDICTOR_OUTPOST_NEW_H_

#endif //PREDICTOR_OUTPOST_NEW_H_

#include <data-structure/communication.h>
#include "../digital-twin/battlefield.h"
#include "debug-tools/painter.h"
#include "predictor-armor/spin_detector.h"
#include "predictor-armor-debug/predictor_armor_debug.h"

class OutpostPredictor{
public:
    OutpostPredictor() = default;
    ~OutpostPredictor() = default;
    SendPacket Run(Battlefield battlefield, float bullet_speed = 16);
    void SetColor(const Entity::Colors& enemy_color)    {   enemy_color_ = enemy_color;}

private:
    int FindBiggestArmor(const std::vector<Armor> &armors);
    void DecideComingGoing();
    void IsClockwise();

    Outpost outpost_{};

    std::chrono::high_resolution_clock::time_point start_time_{};
    std::chrono::high_resolution_clock::time_point ready_time_{};

    Entity::Colors enemy_color_{};
    bool checked_clockwise_{};
    int clockwise_{};

    double last_armor_x_{};

    bool ready_fire_ = false;
    bool prepared_ = false;
    bool need_init_ = true;

    double biggest_area_{};
    double shoot_delay_time_=0.1;

    int buff{};
};

