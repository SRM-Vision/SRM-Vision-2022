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
enum AimModes{
    kNormal = 10,
    kAntiTop = 20,
    kSmallRune = 30,
    kBigRune = 40,
    kOutPost = 50,
    kAutoAntiTop = 1,   //  use for sentry manually
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
    std::array<float,3> yaw_pitch_roll;

    ReceivePacket() :
            mode(kNormal),
            armor_kind(0),
            prior_enemy(0),
            color(Entity::Colors::kBlue),
            bullet_speed(0),
            yaw_pitch_roll{0,0,0} {}

    explicit ReceivePacket(const SerialReceivePacket &serial_receive_packet) :
            mode(static_cast<AimModes>(serial_receive_packet.mode)),
            armor_kind(serial_receive_packet.armor_kind),
            prior_enemy(serial_receive_packet.prior_enemy),
            color(Entity::Colors::kBlue),
            bullet_speed(serial_receive_packet.bullet_speed),
            yaw_pitch_roll{serial_receive_packet.yaw,serial_receive_packet.pitch,serial_receive_packet.roll} {
        if(serial_receive_packet.color == 23)   /// TODO checkout color code with controller
            color = Entity::Colors::kBlue;
        else if(serial_receive_packet.color == 33)
            color = Entity::Colors::kRed;
    }
};

#endif  // COMMUNICATION_H_
