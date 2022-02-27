/**
 * Serial communication packet models header.
 * \author anonymity, screw-44
 * \date 2022.1.28
 */

#ifndef SERIAL_DATA_H_
#define SERIAL_DATA_H_

#include <Eigen/Eigen>

// TODO Simplify these data models.

/// \brief Packet format to send.
struct SendPacket {
    float yaw;
    float pitch;
    float delay;
    float check_sum;
};

/// \brief Packet format received.
/// \warning Do NOT use this data type except in Serial class.
struct SerialReceivePacket {
    int mode;
    int armor_kind;
    int prior_enemy;
    int color;
    float bullet_speed;
    float quaternion_0;
    float quaternion_1;
    float quaternion_2;
    float quaternion_3;
};

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

    explicit ReceivePacket(const SerialReceivePacket &serial_receive_packet) {
        mode = serial_receive_packet.mode;
        armor_kind = serial_receive_packet.armor_kind;
        prior_enemy = serial_receive_packet.prior_enemy;
        color = serial_receive_packet.color;
        bullet_speed = serial_receive_packet.bullet_speed;
        quaternion = {serial_receive_packet.quaternion_0,
                      serial_receive_packet.quaternion_1,
                      serial_receive_packet.quaternion_2,
                      serial_receive_packet.quaternion_3};
    }
};

#endif  // SERIAL_DATA_H_
