#pragma once
#include <functional>

#include "common/framerate.hpp"
#include "connector/connector_node.hpp"
#include "connector/msgpack.hpp"
#include "motor/motor_def.hpp"
#include "motor/motor_dji6020.hpp"
#include "msg_layer/msg_layer.hpp"

namespace motor {

template <>
class Motor<MotorType::DJI_3508> : public Motor<MotorType::DJI_6020> {
	using PARENT = Motor<MotorType::DJI_6020>;

   public:
	using ConnectorSendNodeT = PARENT::ConnectorSendNodeT;
	constexpr auto base_id() -> MotorId override { return 0x200; };
	constexpr auto control_frame_low() -> MotorId override { return 0x200; };
	constexpr auto control_frame_high() -> MotorId override { return 0x1FF; };
	constexpr auto max_current() -> real override { return 20.0; };
	Motor(const Config& config) : PARENT(config) {}
};

}  // namespace motor
