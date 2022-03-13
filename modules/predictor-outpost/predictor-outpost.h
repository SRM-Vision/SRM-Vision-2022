//
// Created by screw on 2022/3/13.
//

#ifndef SRM_VISION_2022_PREDICTOR_OUTPOST_H
#define SRM_VISION_2022_PREDICTOR_OUTPOST_H

#include "data-structure/serial_data.h"
#include "digital-twin/battlefield.h"



class OutpostPredictor
{
public:
    OutpostPredictor() = default;

    SendPacket Run(const Battlefield& battlefield);
private:
    SendPacket send_packet;
    std::vector<Outpost> outpost;
    Armor going_armor;
    Armor coming_armor;

};

#endif //SRM_VISION_2022_PREDICTOR_OUTPOST_H
