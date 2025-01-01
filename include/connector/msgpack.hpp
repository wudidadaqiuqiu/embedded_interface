#pragma once
#include <cstring>

#include "connector/IdPack.h"
#include "connector/MotorFdb.h"
#include "connector/data_convert.hpp"

namespace connector {
#define pi (3.1415926f)
#define RAD2DEG (180.0f / pi)
#define DEG2RAD (pi / 180.0f)
using real = float;

struct CanFrame {
    using MSGT = IdPack;

    static void pack(IdPack& msg, const std::vector<uint8_t>& data, uint32_t id) {
        msg.id = id;
        msg.data = data;
    }

    static void unpack(const IdPack::ConstPtr& msg, std::vector<uint8_t>& data, uint32_t& id) {
        data.resize(msg->data.size());
        id = msg->id;
        memcpy(data.data(), msg->data.data(), msg->data.size());
    }
};

inline void motor_6020_pack(MotorFdb& msg, const std::vector<uint8_t>& data, uint32_t id) {
    msg.id = id;
    msg.pos.deg.num = (real)(short)(((short)data[0]) << 8 | data[1]) * 360.0 / 8192.0;
    msg.pos.rad.num = msg.pos.deg.num * DEG2RAD;
    // rpm -> rad/s
    msg.vel.deg.num = (real)(short)(((short)data[2]) << 8 | data[3]) * 6.0f;
    msg.vel.rad.num = msg.vel.deg.num * DEG2RAD;
    msg.current.num = (real)(short)(((short)data[4]) << 8 | data[5]) * 3.0 / 16384.0;

    msg.temperature.num = (float)data[6];
}

struct MotorPack {
    using MSGT = MotorFdb;
    static void pack(MotorFdb& msg, const std::vector<uint8_t>& data, uint32_t id) { motor_6020_pack(msg, data, id); };
};

template <>
inline void data_convert(const CanFrame::MSGT& msg, MotorFdb& data) {
    motor_6020_pack(data, msg.data, msg.id);
}
}  // namespace connector