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
    float yaw;
    float pitch;
    float roll;
};

inline std::ostream &operator<<(std::ostream &str, const SerialReceivePacket &receive_packet) {
    str << receive_packet.mode << " | " << receive_packet.armor_kind << " | " << receive_packet.prior_enemy
        << " | " << receive_packet.color << " | " << receive_packet.bullet_speed << " | " << receive_packet.yaw
        << " | " << receive_packet.pitch << " | " << receive_packet.roll;
    return str;
}

/// \brief Packet format received.
/// \warning Do NOT use this data type except in Serial class.
struct SerialSendPacket {
    float yaw;
    float pitch;
    float delay;
    int distance_mode;  // maybe only used in sentry
    int fire;
    float check_sum;
};

inline std::ostream &operator<<(std::ostream &str, const SerialSendPacket &send_packet) {
    str << send_packet.yaw << " | " << send_packet.pitch << " | " << send_packet.delay << " | "
        << send_packet.distance_mode << " | " << send_packet.fire << " | " << send_packet.check_sum;
    return str;
}

#endif  // PACKET_H_
