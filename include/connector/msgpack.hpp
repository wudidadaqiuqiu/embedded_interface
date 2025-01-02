#pragma once
#include <cstring>

#include "robot_msg/MotorFdb.h"
#include "robot_msg/IdPack.h"

namespace connector {
#define pi (3.1415926f)
#define RAD2DEG (180.0f / pi)
#define DEG2RAD (pi / 180.0f)

using robot_msg::MotorFdb;
using robot_msg::IdPack;

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

}  // namespace connector