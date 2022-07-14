/**
 * Hero controller header.
 * \author LIYunzhe1408
 * \date 2022.2.11
 * \warning NEVER include this file except in ./controller_hero.cpp.
 */

#ifndef CONTROLLER_HERO_H_
#define CONTROLLER_HERO_H_


#include <predictor-outpost/predictor_outpost_new.h>

class [[maybe_unused]] HeroController final : public Controller {
public:
    bool Initialize() final;

    void Run() final;

private:
    /// Own registry in controller factory.
    void CheckSum(){
        send_packet_.check_sum = send_packet_.yaw + send_packet_.pitch + send_packet_.delay +
                                float(send_packet_.fire) + float (send_packet_.distance_mode) +
                                float(send_packet_.point1_x) + float(send_packet_.point1_y) +
                                float(send_packet_.point2_x) + float(send_packet_.point2_y) +
                                float(send_packet_.point3_x) + float(send_packet_.point3_y) +
                                float(send_packet_.point4_x) + float(send_packet_.point4_y);
    };
    [[maybe_unused]] static ControllerRegistry<HeroController> hero_controller_registry_;

//    OutpostDataDetector outpost_detector_;
//    OutpostPredictor outpost_predictor_;
    OutpostPredictor outpost_predictor_new_;


};

#endif  // CONTROLLER_HERO_H_
