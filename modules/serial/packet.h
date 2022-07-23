/**
 * Serial packet structure definition header.
 * @author trantuan-20048607
 * @date 2022.2.24
 */

#ifndef PACKET_H_
#define PACKET_H_

/// @brief Packet format received.
/// @warning Do NOT use this data type except in Serial class.
struct SerialReceivePacket {
    int mode;
    int armor_kind;
    int prior_enemy;
    int color;
    float bullet_speed;
    float yaw;
    float pitch;
    float roll;
    float chassis_speed;
};

inline std::ostream &operator<<(std::ostream &str, const SerialReceivePacket &receive_packet) {
    str << receive_packet.mode << " | " << receive_packet.armor_kind << " | " << receive_packet.prior_enemy
        << " | " << receive_packet.color << " | " << receive_packet.bullet_speed << " | " << receive_packet.yaw
        << " | " << receive_packet.pitch << " | " << receive_packet.roll << " | " << receive_packet.chassis_speed;
    return str;
}

/// @brief Packet format received.
/// @warning Do NOT use this data type except in Serial class.
struct SerialSendPacket {
    float yaw;
    float pitch;
    float delay;
    int distance_mode;  // maybe only used in sentry
    int fire;
    int point1_x, point1_y;
    int point2_x, point2_y;
    int point3_x, point3_y;
    int point4_x, point4_y;
    float check_sum;

    SerialSendPacket() = default;

    SerialSendPacket(float yaw, float pitch, float delay, int distance_mode, int fire, int point1_x,
                     int point1_y, int point2_x, int point2_y, int point3_x, int point3_y, int point4_x,
                     int point4_y):yaw(yaw),pitch(pitch),delay(delay),distance_mode(distance_mode),fire(fire),
                     point1_x(point1_x),point1_y(point1_y),point2_x(point2_x),point2_y(point2_y),point3_x(point3_x),
                     point3_y(point3_y),point4_x(point4_x),point4_y(point4_y){
        check_sum = yaw + pitch + delay + float(distance_mode + fire + point1_x + point1_y + point2_x + point2_y +
                    point3_x + point3_y + point4_x + point4_y);
    }
};

inline std::ostream &operator<<(std::ostream &str, const SerialSendPacket &send_packet) {
    str << send_packet.yaw << " | " << send_packet.pitch << " | " << send_packet.delay << " | "
        << send_packet.distance_mode << " | " << send_packet.fire << " | " << send_packet.check_sum;
    return str;
}

#endif  // PACKET_H_
