/**
 * Serial communication packet models header.
 * \author anonymity, screw-44
 * \date 2022.1.28
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include <Eigen/Eigen>
#include "serial/packet.h"

/// \brief Packet format to send.
using SendPacket = SerialSendPacket;

/// \brief Packet format received.
struct ReceivePacket {
    int mode;
    int armor_kind;
    int prior_enemy;
    int color;
    float bullet_speed;
    Eigen::Quaternionf quaternion;

    ReceivePacket() :
            mode(0),
            armor_kind(0),
            prior_enemy(0),
            color(0),
            bullet_speed(0),
            quaternion({1, 0, 0, 0}) {}

    explicit ReceivePacket(const SerialReceivePacket &serial_receive_packet) :
            mode(serial_receive_packet.mode),
            armor_kind(serial_receive_packet.armor_kind),
            prior_enemy(serial_receive_packet.prior_enemy),
            color(serial_receive_packet.color),
            bullet_speed(serial_receive_packet.bullet_speed),
            quaternion(serial_receive_packet.quaternion) {}
};

#endif  // COMMUNICATION_H_
