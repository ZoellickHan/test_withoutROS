#ifndef RM_SERIAL_DRIVER__NEWPROTOCOL_HPP_
#define RM_SERIAL_DRIVER__NEWPROTOCOL_HPP_

#include <sys/cdefs.h>
#include <cstring>
#include <algorithm>
#include <boost/cstdint.hpp>
#include <cstdint>
#include <iostream>
#include <vector>

// Protocol version: v3.0 (RM2024)
// note: CRC operations are defined in CRC.hpp
/*
 * CRC parameters:
 * width = 16, i.e. CRC-16
 * init = 0xFFFF
 * poly = 0x1189
 * final_xor_value = 0 (do nothing)
 */

// UART parameters:
/*
 * baud_rate = 2000000
 * stop bits = 1
 * parity = none
 * hardware_flow_ctrl = no
 * data_size = 8-bit
 */

namespace rm_serial_driver
{
// 0x stands for msg received (1 - 3 for msg from pcb to nuc)
// 0x stands for ring buffer msg received (4 - 5 for msg from pcb to nuc)
// 1x stands for msg sent (11 - 20 for msg from nuc to pcb)
// 2x stands for ring buffer msg sent (21 - 30 for msg from nuc to pcb)
#define ID_NUM 17
enum CommunicationType : uint8_t
{
    BEAT_MSG                 = 0x01,
    GIMBAL_MSG               = 0x02,
    CHASSIS_MSG              = 0x03,
    SENTRY_GIMBAL_MSG        = 0x07,
    FIELD_MSG                = 0x09,
    TWOCRC_GIMBAL_MSG        = 0xA2,
    TWOCRC_CHASSIS_MSG       = 0xA3,
    TWOCRC_SENTRY_GIMBAL_MSG = 0xA7,
    TWOCRC_FIELD_MSG         = 0xA9,

    GIMBAL_CMD               = 0x12,
    CHASSIS_CMD              = 0x13,
    ACTION_CMD               = 0x14,
    SENTRY_GIMBAL_CMD        = 0x17,
    TWOCRC_GIMBAL_CMD        = 0xB2,
    TWOCRC_CHASSIS_CMD       = 0xB3,
    TWOCRC_ACTION_CMD        = 0xB4,
    TWOCRC_SENTRY_GIMBAL_CMD = 0xB7,
};

// **************************** //
// * protocol for ring buffer * //
// **************************** //

struct Header
{
    uint8_t sof        = 0xAAu;
    uint8_t dataLen    = 0;
    uint8_t protocolID = 0;
    uint8_t crc_1;
    uint8_t crc_2;

} __attribute__((packed));

struct Header_4sof
{
    uint8_t sof1        = 0xAAu;
    uint8_t sof2        = 0xAAu;
    uint8_t sof3        = 0xAAu;
    uint8_t sof4        = 0xAAu;
    uint8_t dataLen     = 0;

    uint8_t little_endian  = 0;
    uint8_t protocolID     = 0x55;
    uint8_t crc_1;
    uint8_t crc_2;

} __attribute__((packed));

struct TwoCRC_GimbalMsg  // TWOCRC_GIMBAL_MSG, also 0xA2
{
    Header header;

    uint8_t cur_cv_mode;
    uint8_t target_color;
    float bullet_speed;
    float q_w;
    float q_x;
    float q_y;
    float q_z;

    uint8_t crc_3;
    uint8_t crc_4;

} __attribute__((packed));


struct TwoCRC_SentryGimbalMsg  // TWOCRC_GIMBALSTATUS_MSG, also 0xA3
{
    Header header;

    uint8_t cur_cv_mode;
    uint8_t target_color;
    float bullet_speed;
    // small gimbal q
    float small_q_w;
    float small_q_x;
    float small_q_y;
    float small_q_z;

    // main gimbal q
    float big_q_w;
    float big_q_x;
    float big_q_y;
    float big_q_z;

    uint8_t crc_3;
    uint8_t crc_4;

} __attribute__((packed));

struct TwoCRC_GimbalCommand  // TWOCRC_GIMBAL_CMD, also 0xB1
{
    Header header;

    float target_pitch;
    float target_yaw;
    uint8_t shoot_mode;

    uint8_t crc_3;
    uint8_t crc_4;

} __attribute__((packed));

struct TwoCRC_ChassisCommand  // TWOCRC_CHASSIS_CMD, also 0xB2
{
    Header header;

    float vel_x;
    float vel_y;
    float vel_w;

    uint8_t crc_3;
    uint8_t crc_4;

} __attribute__((packed));

struct TwoCRC_ActionCommand  // TWOCRC_ACTION_CMD, also 0xB3
{
    Header header;

    bool scan;
    bool spin;
    bool cv_enable;

    uint8_t crc_3;
    uint8_t crc_4;

} __attribute__((packed));

struct TwoCRC_SentryGimbalCommand
{
    Header header;

    float l_target_pitch;
    float l_target_yaw;
    uint8_t l_shoot_mode;

    float r_target_pitch;
    float r_target_yaw;
    uint8_t r_shoot_mode;

    float main_target_pitch;
    float main_target_yaw;

    uint8_t crc_3;
    uint8_t crc_4;

} __attribute__((packed));

// *********************************//
// * protocol without ring buffer * //
// *********************************//

struct BeatMsg  // BEAT_MSG
{
    uint8_t sof        = 0xAAu;
    uint8_t dataLen    = 0;
    uint8_t protocolID = 0;

    uint32_t beat = 0;

    uint8_t crc_1;
    uint8_t crc_2;

} __attribute__((packed));

struct GimbalMsg  // GIMBAL_MSG
{
    uint8_t sof        = 0xAAu;
    uint8_t dataLen    = 0;
    uint8_t protocolID = 0;

    uint8_t cur_cv_mode;
    uint8_t target_color;
    float bullet_speed;
    float q_w;
    float q_x;
    float q_y;
    float q_z;

    uint8_t crc_1;
    uint8_t crc_2;

} __attribute__((packed));

struct SentryGimbalMsg  // SENTRY_GIMBAL_MSG
{
    uint8_t sof        = 0xAAu;
    uint8_t dataLen    = 0;
    uint8_t protocolID = 0;

    uint8_t cur_cv_mode;
    uint8_t target_color;
    float bullet_speed;

    float small_gimbal_q_w;
    float small_gimbal_q_x;
    float small_gimbal_q_y;
    float small_gimbal_q_z;

    float big_gimbal_q_w;
    float big_gimbal_q_x;
    float big_gimbal_q_y;
    float big_gimbal_q_z;

    uint8_t crc_1;
    uint8_t crc_2;

} __attribute__((packed));

struct GameMsg
{
    uint8_t sof        = 0xAAu;
    uint8_t dataLen    = 0;
    uint8_t protocolID = 0;
    // game msg
    bool is_game_start;
    float sentryHPpercent;
    uint16_t accumulatedHeal;
    // crc
    uint8_t crc_1;
    uint8_t crc_2;
} __attribute__((packed));

struct GimbalCommand  // GIMBAL_CMD
{
    uint8_t sof        = 0xAAu;
    uint8_t dataLen    = 0;
    uint8_t protocolID = 0;

    float target_pitch;
    float target_yaw;
    uint8_t shoot_mode;

    uint8_t crc_1;
    uint8_t crc_2;

} __attribute__((packed));

struct SentryGimbalCommand  // SENTRY_GIMBAL_CMD
{
    uint8_t sof        = 0xAAu;
    uint8_t dataLen    = 0;
    uint8_t protocolID = 0;

    float left_target_pitch;
    float left_target_yaw;
    uint8_t left_shoot_mode;

    float right_target_pitch;
    float right_target_yaw;
    uint8_t right_shoot_mode;

    float vel_x;
    float vel_y;
    float vel_w;

    uint8_t crc_1;
    uint8_t crc_2;

} __attribute__((packed));

struct ChassisCommand  // CHASSIS_CMD
{
    // header
    uint8_t sof        = 0xAAu;
    uint8_t dataLen    = 0;
    uint8_t protocolID = 0;
    // navigation control command
    float vel_x;
    float vel_y;
    float vel_w;
    // crc
    uint8_t crc_1;
    uint8_t crc_2;

} __attribute__((packed));

struct ActionCommand
{
    uint8_t sof        = 0xAAu;
    uint8_t dataLen    = 0;
    uint8_t protocolID = 0;
    // action cmd
    bool scan;
    bool spin;
    // crc
    uint8_t crc_1;
    uint8_t crc_2;
} __attribute__((packed));

template<typename T>
inline T convertToStruct(const uint8_t* buffer) {
    T result;
    std::memcpy(&result, buffer, sizeof(T));
    return result;
}

template <typename T>
inline T fromVector(const std::vector<uint8_t> &data)
{
    T received_packet;
    std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&received_packet));
    return received_packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T &data)
{
    std::vector<uint8_t> sent_packet(sizeof(T));
    std::copy(reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T), sent_packet.begin());
    return sent_packet;
}
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PROTOCOL_HPP_
