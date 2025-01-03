#pragma once
#include <functional>
#include "connector/msgpack.hpp"
#include "motor/motor.hpp"
namespace motor {

template <>
class Motor<MotorType::DJI_6020> {
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame>* rnode_;
    MotorId id_;
    MotorFdb data;
    real last_pos_deg;
    int round = 0;
    
    public:
    static constexpr MotorId BASE_ID = 0x204;
    Motor(ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame>* rnode_,
        MotorId id);
    // Motor(std::string recv_topic) {}
    void register_callback(std::function<void(const CanFrame::MSGT&)> callback) {
        rnode_->register_callback(callback);
    }
    
    const MotorFdb& get_fdb() const { return data;}
};


inline void motor_6020_pack(MotorFdb& msg, const std::vector<uint8_t>& data, uint32_t id) {
    msg.id = id;
    msg.pos.deg.num = (real)(short)(((short)data[0]) << 8 | data[1]) * 360.0 / 8192.0;
    msg.pos.rad.num = msg.pos.deg.num * UsefulNum::DEG2RAD;
    // rpm -> rad/s
    msg.vel.deg.num = (real)(short)(((short)data[2]) << 8 | data[3]) * 6.0f;
    msg.vel.rad.num = msg.vel.deg.num * UsefulNum::DEG2RAD;
    msg.current.num = (real)(short)(((short)data[4]) << 8 | data[5]) * 3.0 / 16384.0;

    msg.temperature.num = (float)data[6];
}

struct MotorDji6020Pack {
    using MSGT = MotorFdb;
    MSGT& msg;
    MotorDji6020Pack(MSGT& msg) : msg(msg) {}
    static void pack(MotorFdb& msg, const std::vector<uint8_t>& data, uint32_t id) { motor_6020_pack(msg, data, id); };
};
}

template <>
inline void connector_common::data_convert<CanFrame, motor::MotorDji6020Pack>
    (const CanFrame::MSGT& src, MotorFdb& dst) {
    motor::motor_6020_pack(dst, src.data, src.id);
}

namespace motor {
Motor<MotorType::DJI_6020>::Motor(ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame>* rnode_,
        MotorId id) : 
    rnode_(rnode_), id_(id) {
    using connector_common::Deg;
    using robot_msg::AngleRelate;
    
    auto l2 = [this](const CanFrame::MSGT& msg) {
        if (msg.id != BASE_ID + id_) 
            return;
        last_pos_deg = data.pos.deg.num;
        data_convert<CanFrame, MotorDji6020Pack>(msg, data);
        if (data.pos.deg.num - last_pos_deg > 180.0)
            round--;
        else if (data.pos.deg.num - last_pos_deg < -180.0)
            round++;
        data_convert<Deg, AngleRelate>(data.pos.deg.num + round * 360.0f, data.pos_zero_cross);
    };
    rnode_->register_callback(l2);
}

}

