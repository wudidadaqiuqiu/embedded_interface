#pragma once
#include <cstring>

#include "msg_layer/msg.hpp"

namespace connector {

using con_used_msg::IdPack;

struct CanFrame {
    using MSGT = IdPack;
    MSGT& msg;
    CanFrame(MSGT& msg) : msg(msg) {}
    static void pack(IdPack& msg, const std::vector<uint8_t>& data, uint32_t id) {
        msg.id = id;
        msg.data = data;
    }

    static void unpack(const IdPack::SharedPtr& msg, std::vector<uint8_t>& data, uint32_t& id) {
        data.resize(msg->data.size());
        id = msg->id;
        memcpy(data.data(), msg->data.data(), msg->data.size());
    }
    
    static void unpack(const IdPack& msg, std::vector<uint8_t>& data, uint32_t& id) {
        data.resize(msg.data.size());
        id = msg.id;
        memcpy(data.data(), msg.data.data(), msg.data.size());
    }
};

struct TtyFrame {
    using MSGT = IdPack;
    MSGT& msg;
    TtyFrame(MSGT& msg) : msg(msg) {}
    static void pack(IdPack& msg, const std::vector<uint8_t>& data, uint32_t len) {
        // (void)id;
        // msg.id = id;
        // std::cout << "tty data.size() " << data.size() << std::endl;
        msg.data.assign(data.begin(), data.begin() + 
            std::min(len, static_cast<uint32_t>(data.size())));
        // std::cout << "msg data.size() " << msg.data.size() << std::endl;
    }

    static void unpack(const IdPack& msg, std::vector<uint8_t>& data, uint32_t& id) {
        CanFrame::unpack(msg, data, id);
    }
};


}  // namespace connector