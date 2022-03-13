/**
 * Serial packet structure definition header.
 * \author trantuan-20048607
 * \date 2022.2.24
 */

#ifndef PACKET_H_
#define PACKET_H_

/// \brief Packet format received.
/// \warning Do NOT use this data type except in Serial class.
struct SerialReceivePacket {
    int mode;
    int armor_kind;
    int prior_enemy;
    int color;
    float bullet_speed;
    float quaternion[4];
};

inline std::ostream &operator<<(std::ostream &str, const SerialReceivePacket &receive_packet) {
    str << receive_packet.mode << " | " << receive_packet.armor_kind << " | " << receive_packet.prior_enemy
        << " | " << receive_packet.color << " | " << receive_packet.bullet_speed;
    for (float i: receive_packet.quaternion)
        str << " | " << i;
    return str;
}

/// \brief Packet format received.
/// \warning Do NOT use this data type except in Serial class.
struct SerialSendPacket {
    float yaw;
    float pitch;
    float delay;
    bool fire;
    float check_sum;
};

inline std::ostream &operator<<(std::ostream &str, const SerialSendPacket &send_packet) {
    str << send_packet.yaw << " | " << send_packet.pitch << " | " << send_packet.delay << " | " << send_packet.fire;
    return str;
}

#endif  // PACKET_H_
