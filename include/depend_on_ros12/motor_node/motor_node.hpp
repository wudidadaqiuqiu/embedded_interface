#pragma once
#include "motor/motor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "common/debug/log.hpp"
namespace motor_node {
using con_used_msg::MotorFdb;
using connector::CanFrame;
using motor::Motor;
using motor::MotorType;

template <MotorType MotorTypeT>
class MotorNode : public rclcpp::Node {
   public:
	using SharedPtr = std::shared_ptr<MotorNode<MotorTypeT>>;
	struct Config {
		std::string name;
		Motor<MotorTypeT>::Config motor_config;
		const rclcpp::QoS& fdb_pub_qos;
		const rclcpp::NodeOptions& options;
		const bool is_log = false;
	};
	MotorNode(const Config& config);
	~MotorNode() {
		LOG_INFO(1, "~MotorNode()");
	}
	auto get_motor() -> auto& { return motor_; }
	const std::string fdb_topic;
   private:
	Motor<MotorTypeT> motor_;
	rclcpp::Publisher<MotorFdb>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
};

template <MotorType MotorTypeT>
MotorNode<MotorTypeT>::MotorNode(const Config& config)
	: Node(config.name, config.options), fdb_topic("/" + config.name + "/motor_fdb"), motor_(config.motor_config) {
	LOG_INFO(1, "motor node start id: %d", config.motor_config.id_);
	publisher_ = this->create_publisher<MotorFdb>(
		fdb_topic, config.fdb_pub_qos);
	// 注册回调函数
	auto l = [this, config](const CanFrame::MSGT& msg) {
		(void)msg;
		publisher_->publish(motor_.get_fdb());
		// LOG_INFO_CNT(1000, "motor fdb id: %d, id: %d", config.motor_config.id_, msg.id);
	};
	motor_.register_callback(l);

	// 定时器
	if (config.is_log) {
		timer_ =
			this->create_wall_timer(std::chrono::seconds(1), [this]() -> void {
				// 打印 motor fps
				LOG_INFO(1,"motor fps: %lf", motor_.get_framerate());
			});
	}
}

}  // namespace motor_node