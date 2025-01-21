#pragma once
#include "motor/motor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace motor_node {
using motor::Motor;
using motor::MotorType;

template <MotorType MotorTypeT>
class MotorNode : public rclcpp::Node {
   public:
	struct Config {
		std::string name;
		Motor<MotorTypeT>::Config motor_config;
		const rclcpp::QoS& fdb_pub_qos;
		const rclcpp::NodeOptions& options;
		const bool is_log = false;
	};
	MotorNode(const Config& config);

   private:
	Motor<MotorTypeT> motor_;
	rclcpp::Publisher<MotorFdb>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
};

template <MotorType MotorTypeT>
MotorNode<MotorTypeT>::MotorNode(const Config& config)
	: Node(config.name, config.options), motor_(config.motor_config) {
	publisher_ = this->create_publisher<MotorFdb>(
		"/" + config.name + "/motor_fdb", config.fdb_pub_qos);
	// 注册回调函数
	auto l = [&](const CanFrame::MSGT& msg) {
		(void)msg;
		publisher_->publish(motor_.get_fdb());
	};
	motor_.register_callback(l);

	// 定时器
	if (config.is_log) {
		timer_ =
			this->create_wall_timer(std::chrono::seconds(1), [this]() -> void {
				// 打印 motor fps
				std::cout << "motor fps: " << motor_.get_framerate()
						  << std::endl;
			});
	}
}

}  // namespace motor_node