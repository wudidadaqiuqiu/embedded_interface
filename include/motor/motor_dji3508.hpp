#pragma once
#include <functional>
#include "connector/msgpack.hpp"
#include "connector/connector_node.hpp"
#include "motor/motor_def.hpp"
#include "motor/motor_dji6020.hpp"
#include "common/framerate.hpp"
#include "msg_layer/msg_layer.hpp"

namespace motor {

template <>
struct MotorConfig<MotorType::DJI_3508> {
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame>& rnode_;
    MotorId id_;
};

template <>
class Motor<MotorType::DJI_3508> : public Motor<MotorType::DJI_6020> {
    // ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame>& rnode_;
    // MotorId id_;
    // MotorFdb data;
    // real last_pos_deg;
    // int round = 0;
    // // 帧率计算
    // connector_common::Framerate framerate_;
    using PARENT = Motor<MotorType::DJI_6020>;
    public:
    using ConnectorSendNodeT = PARENT::ConnectorSendNodeT;
    static constexpr MotorId BASE_ID = 0x200;
    static constexpr MotorId CONTROL_FRAME_LOW = 0x200;
    static constexpr MotorId CONTROL_FRAME_HIGH = 0x1FF;
    static constexpr real MAX_CURRENT = 20.0;
    Motor(const MotorConfig<MotorType::DJI_3508>& config);

    MotorId set_send_buf(real& current, std::vector<uint8_t>& buf) {
        if (buf.size() != 8) {
            return MotorId(0);
        }
        MotorId id = (id_ <= 4) ? CONTROL_FRAME_LOW : CONTROL_FRAME_HIGH;
        uint8_t low_index = ((id_ - 1) % 4) * 2;
        uint8_t high_index = ((id_ - 1) % 4) * 2 + 1;
        short out = (short)(current * 16384.0 / MAX_CURRENT);

        buf[low_index] = (uint8_t)(out >> 8);
        buf[high_index] = (uint8_t)(out & 0xFF);
        return id;
    }
};

struct MotorDji3508Pack {
    using MSGT = MotorFdb;
    MSGT& msg;
    MotorDji3508Pack(MSGT& msg) : msg(msg) {}
    // 与6020的格式相同
    static void pack(MotorFdb& msg, const std::vector<uint8_t>& data, uint32_t id) { motor_6020_pack(msg, data, id); };
};
}

template <>
inline void connector_common::data_convert<CanFrame, motor::MotorDji3508Pack>
    (const CanFrame& src, motor::MotorDji3508Pack& dst) {
    motor::motor_6020_pack(dst.msg, src.msg.data, src.msg.id);
}

namespace motor {
Motor<MotorType::DJI_3508>::Motor(const MotorConfig<MotorType::DJI_3508>& config) : PARENT(config.rnode_, config.id_) {
    using connector_common::Deg;
    using con_used_msg::AngleRelate;
    
    auto l2 = [this](const CanFrame::MSGT& msg) {
        if (msg.id != BASE_ID + id_) 
            return;
        last_pos_deg = data.pos.deg.num;
        motor::MotorDji3508Pack temp(data);
        data_convert(CanFrame(const_cast<CanFrame::MSGT&>(msg)), temp);
        if (data.pos.deg.num - last_pos_deg > 180.0)
            round--;
        else if (data.pos.deg.num - last_pos_deg < -180.0)
            round++;
        data_convert(Deg(data.pos.deg.num + round * 360.0f), data.pos_zero_cross);
        // 帧率计算
        framerate_.update();
    };
    rnode_.register_callback(l2);
}

}

