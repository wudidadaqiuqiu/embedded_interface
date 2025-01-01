#pragma once
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include "connector/connector.hpp"
#include "connector/IdPack.h"
#include "connector/MotorFdb.h"

namespace connector {
#define pi (3.1415926f)
#define RAD2DEG (180.0f / pi)
#define DEG2RAD (pi / 180.0f)
using real = float;
inline void motor_6020_pack(MotorFdb& msg, const std::vector<uint8_t>& data, uint32_t id) {
    msg.id = id;
    msg.pos.deg.num = (real)(short)(((short)data[0]) << 8 | data[1]) * 360.0 / 8192.0;
    msg.pos.rad.num = msg.pos.deg.num * DEG2RAD;
    // rpm -> rad/s
    msg.vel.deg.num = (real)(short)(((short)data[2]) << 8 | data[3]) * 6.0f;
    msg.vel.rad.num = msg.vel.deg.num * DEG2RAD;
    msg.current.num = (real)(short)(((short)data[4]) << 8 | data[5]) * 3.0 / 16384.0;

    msg.temperature.num  = (float)data[6];
}

struct MotorPack {
    using MSGT = MotorFdb;
    static void pack(MotorFdb& msg, const std::vector<uint8_t>& data, uint32_t id) { motor_6020_pack(msg, data, id); };
};


}