#pragma once
#include <cstring>

#include "robot_msg/IdPack.h"

namespace connector {

using robot_msg::IdPack;

struct CanFrame {
    using MSGT = IdPack;
    MSGT& msg;
    CanFrame(MSGT& msg) : msg(msg) {}
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