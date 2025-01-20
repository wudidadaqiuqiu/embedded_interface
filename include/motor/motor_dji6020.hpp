#pragma once
#include <functional>

#include "common/framerate.hpp"
#include "connector/connector_node.hpp"
#include "connector/msgpack.hpp"
#include "motor/motor_def.hpp"
#include "msg_layer/msg_layer.hpp"
namespace motor {

inline void motor_6020_pack(MotorFdb& msg, const std::vector<uint8_t>& data,
							uint32_t id);

template <>
class Motor<MotorType::DJI_6020> {
   protected:
	ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame>& rnode_;
	MotorId id_;
	MotorFdb data;
	real last_pos_deg;
	int round = 0;
	// 帧率计算
	connector_common::Framerate framerate_;

   public:
	virtual constexpr auto base_id() -> MotorId { return 0x204; };
	virtual constexpr auto control_frame_low() -> MotorId { return 0x1FE; };
	virtual constexpr auto control_frame_high() -> MotorId { return 0x2FE; };
	virtual constexpr auto max_current() -> real { return 3.0; };
	const auto& get_fdb() const { return data; }
	auto get_framerate() const { return framerate_.fps; }
	auto& get_connector() const { return rnode_.get_connector(); }
	struct Config {
		ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame>& rnode_;
		MotorId id_;
	};
	using ConnectorSendNodeT =
		connector::ConnectorSendNode<ConnectorType::CAN, CanFrame>;
	Motor(const Config& config);

	void register_callback(
		std::function<void(const CanFrame::MSGT&)> callback) {
		rnode_.register_callback(callback);
	}
	static void motor_6020_pack(MotorFdb& msg, const std::vector<uint8_t>& data,
								uint32_t id, real max_current) {
		msg.id = id;
		msg.pos.deg.num =
			(real)(short)(((short)data[0]) << 8 | data[1]) * 360.0 / 8192.0;
		msg.pos.rad.num = msg.pos.deg.num * UsefulNum::DEG2RAD;
		// rpm -> rad/s
		msg.vel.deg.num = (real)(short)(((short)data[2]) << 8 | data[3]) * 6.0f;
		msg.vel.rad.num = msg.vel.deg.num * UsefulNum::DEG2RAD;
		msg.current.num = (real)(short)(((short)data[4]) << 8 | data[5]) *
						  max_current / 16384.0;

		msg.temperature.num = (float)data[6];
	}

	void pack(const CanFrame::MSGT& frame, MotorFdb& data) {
		motor_6020_pack(data, frame.data, frame.id, max_current());
	}
	MotorId set_send_buf(real& current, std::vector<uint8_t>& buf) {
		// LOG_DEBUG(1, "max current: %f", max_current());
		if (buf.size() != 8) {
			return MotorId(0);
		}
		MotorId id = (id_ <= 4) ? control_frame_low() : control_frame_high();
		uint8_t low_index = ((id_ - 1) % 4) * 2;
		uint8_t high_index = ((id_ - 1) % 4) * 2 + 1;
		short out = (short)(current * 16384.0 / max_current());

		buf[low_index] = (uint8_t)(out >> 8);
		buf[high_index] = (uint8_t)(out & 0xFF);
		return id;
	}
};

Motor<MotorType::DJI_6020>::Motor(const Config& config)
	: rnode_(config.rnode_), id_(config.id_) {
	using con_used_msg::AngleRelate;
	using connector_common::Deg;
	auto l2 = [this](const CanFrame::MSGT& msg) {
		if (msg.id != base_id() + id_) return;
		last_pos_deg = data.pos.deg.num;
		pack(msg, data);
		if (data.pos.deg.num - last_pos_deg > 180.0)
			round--;
		else if (data.pos.deg.num - last_pos_deg < -180.0)
			round++;
		data_convert(Deg(data.pos.deg.num + round * 360.0f),
					 data.pos_zero_cross);
		// 帧率计算
		framerate_.update();
	};
	rnode_.register_callback(l2);
}

}  // namespace motor
