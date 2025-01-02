#pragma once

#include "connector/msgpack.hpp"
#include "motor/motor.hpp"
namespace motor {

template <>
class Motor<MotorType::DJI_6020> {
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame>* rnode_;
    MotorId id_;
    // MotorFdb data;
    public:
    static constexpr MotorId BASE_ID = 0x204;
    Motor(ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame>* rnode_,
        MotorId id) : 
        rnode_(rnode_), id_(id) {
        
        auto l2 = [this](const CanFrame::MSGT& msg) {
            if (msg.id == BASE_ID + id_) {
                
            }
        };
        rnode_->register_callback(l2);
    }
    // Motor(std::string recv_topic) {}

    
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

struct MotorDji6020Pack {
    using MSGT = MotorFdb;
    static void pack(MotorFdb& msg, const std::vector<uint8_t>& data, uint32_t id) { motor_6020_pack(msg, data, id); };
};


}

template <>
inline void data_convert(const CanFrame::MSGT& msg, MotorFdb& data) {
    motor::motor_6020_pack(data, msg.data, msg.id);
}