/**
 * Serial communication packet models header.
 * \author anonymity, screw-44
 * \date 2022.1.28
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include <Eigen/Eigen>
#include "serial/packet.h"
#include "digital-twin/entity.h"

/// \brief Packet of auto aiming mode.
enum AimModes {
    kNormal = 10,
    kSmallRune = 30,
    kBigRune = 40,
    kOutPost = 50,
    SIZE = 6
};

/// \brief Packet format to send.
using SendPacket = SerialSendPacket;

/// \brief Packet format received.
struct ReceivePacket {
    AimModes mode;
    int armor_kind;
    int prior_enemy;
    Entity::Colors color;
    float bullet_speed;
    std::array<float, 3> yaw_pitch_roll;
    Eigen::Vector3f self_speed;

    ReceivePacket() :
            mode(kNormal),
            armor_kind(0),
            prior_enemy(0),
            color(Entity::kBlue),
            bullet_speed(15),
            yaw_pitch_roll{0, 0, 0},
            self_speed{2, 0, 0} {}

    explicit ReceivePacket(const SerialReceivePacket &serial_receive_packet) :
            mode(static_cast<AimModes>(serial_receive_packet.mode)),
            armor_kind(serial_receive_packet.armor_kind),
            prior_enemy(serial_receive_packet.prior_enemy),
            color(Entity::kBlue),
            bullet_speed(serial_receive_packet.bullet_speed),
            yaw_pitch_roll{serial_receive_packet.yaw, serial_receive_packet.pitch, serial_receive_packet.roll},
            self_speed{serial_receive_packet.chassis_speed, 0, 0} {
        if (serial_receive_packet.color == 23)
            color = Entity::Colors::kRed;
        else if (serial_receive_packet.color == 13)
            color = Entity::Colors::kBlue;
    }
};

#endif  // COMMUNICATION_H_
