#pragma once

/**
 * baud rate: 460800
 * stop bits: 1
 * parity: none
 * hardware flow control: cts/rts
 * data size: 8
 */

#include <sys/cdefs.h>
#include <cstring>
#include <algorithm>
#include <boost/cstdint.hpp>
#include <cstdint>

const uint8_t START_BYTE = 0xAAu;

struct FrameHeader
{
    uint8_t sof = START_BYTE;
#if ROS_COMM_16_BITS_DATA_LENGTH
    uint16_t dataLen = 0;
#else
    uint8_t dataLen = 0;
#endif
    uint8_t protocolID = 0;
    //  uint8_t isflowcontrolopen = 0;
} __attribute__((packed));

// 0 stands for msg received, 1 stands for msg sent
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
    TWOCRC_SENTRY_CV_CMD     = 0xB8,
};

/*****************************************************/
/*              MCU -> ROS message struct            */
/*****************************************************/
struct BestMsg
{
    uint32_t beat = 0;
} __attribute__((packed));
struct GimbalMsg
{
    uint8_t cur_cv_mode;
    uint8_t target_color;
    float bullet_speed;
    float q[4];  // w, x, y, z
} __attribute__((packed));

struct ChassisMsg  // UNUSED
{
    float xVel, yVel, wVel;
} __attribute__((packed));

struct SentryGimbalMsg  // 0x07
{
    uint8_t cur_cv_mode;
    uint8_t target_color;  // 0 - red 1 - green 2 - blue
    float bullet_speed;
    float small_q[4];  // w, x, y, z
    float big_q[4];    // w, x, y, z
} __attribute__((packed));

struct FieldMsg
{
    // uint8_t game_state;
    // uint16_t remain_time;
    // uint16_t bullet_remained;
    // uint16_t total_gold_coin;
    // uint16_t remaining_gold_coin;
    // uint8_t team;  // 0 - red 1 - blue

    // uint16_t red_HP[8];
    // uint16_t blue_HP[8];

    uint8_t game_state;
    uint16_t remain_time;
    uint16_t bullet_remain;
    uint8_t team;  // 0 - red 1 - blue

    uint16_t red_hp[8];
    uint16_t blue_hp[8];
    bool in_combat;
    bool bullet_supply_available;
    bool shooter_locked;
} __attribute__((packed));

/*############################### END OF MESSAGE ####################################*/

/*****************************************************/
/*               ROS -> MCU comand struct            */
/*****************************************************/
struct GimbalCommand
{
    float target_pitch;
    float target_yaw;
    uint8_t shoot_mode;
} __attribute__((packed));

struct ChassisCommand
{
    float xVel, yVel, wVel;
} __attribute__((packed));

struct ActionCommand
{
    // bool scan;
    // bool spin;
    // bool cv_enable;
    bool scan;
    float spin_vel;
    bool cv_enable;
} __attribute__((packed));

struct SentryGimbalCommand
{
    float l_target_pitch;
    float l_target_yaw;
    uint8_t l_shoot_mode;

    float r_target_pitch;
    float r_target_yaw;
    uint8_t r_shoot_mode;

    float main_target_pitch;
    float main_target_yaw;
} __attribute__((packed));

struct SentryCVCommand
{
    float target_pitch;
    float target_yaw;
    uint8_t shoot_mode;
    float main_yaw_target;
} __attribute__((packed));

/*############################### END OF COMMAND ####################################*/

/*****************************************************/
/*               ????????????????????????            */
/*****************************************************/

struct CVMode
{
    enum CurCVMode : uint8_t
    {
        MANULA = 0,
    } curCVMode;
    enum TargetColor : uint8_t
    {
        BLUE  = 0,
        GREEN = 1,
        RED   = 2
    } color;
};