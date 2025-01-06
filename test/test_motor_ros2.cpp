
#include "rclcpp/rclcpp.hpp"
#include "connector/connector_node.hpp"
#include "motor/motor.hpp"
#include "controller/controller.hpp"
#include "motor_node/motor_node.hpp"

#include "robot_msg/msg/motor_ref.hpp"
using connector::Connector;
using connector::ConnectorType;
using connector::ConnectorSingleRecvNode;
using connector::ConnectorSendNode;
using connector::IdPack;
using connector::CanFrame;
// using motor::Motor;
using motor::MotorType;
// using controller::Controller;
using controller::ControllerType;
using controller::ControllerConfig;
using controller::ControllerBaseConfigNone;
using motor_node::MotorNode;
using robot_msg::msg::MotorRef;

constexpr motor::MotorId motor_id = 1;
class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode() 
        : Node("test_motor"), 
        connector("can0"), crn(connector), cs(connector),
        motor_node_(
            {&crn, motor_id}, 
            {
                .kp = 10.0 * 3.0 / 16384.0,
                .ki = 0.7 * 3.0 / 16384.0,
                .kd = 600.0 * 3.0 / 16384.0,
                .error_max = 500.0 * 3.0 / 16384.0,
                .Irange = 40.0 * 3.0 / 16384.0,
                .outmax = 3.0
            }
        ){
        motor_ref_.pos_ref.resize(1);
        motor_ref_.pos_ref[0].num = 0;

        // 订阅 MotorRef
        subscription_ = this->create_subscription<MotorRef>(
            "/motor6020_cmd", 10, [&](const MotorRef::SharedPtr msg) {
                motor_ref_ = *msg;
                if (motor_ref_.pos_ref.size() > 0) {
                    motor_node_.set_ref(motor_ref_.pos_ref[0].num);
                }
            }
        );

        // 发布 MotorFdb
        publisher_ = this->create_publisher<MotorFdb>("/motor6020_fdb", 10);

        // 注册回调函数
        auto l = [&](const CanFrame::MSGT& msg) {
            (void)msg;
            publisher_->publish(motor_node_.get_motor().get_fdb());
            motor_node_.set_fdb(motor_node_.get_motor().get_fdb().pos_zero_cross.deg.num);
            motor_node_.calc_control();
            // std::cout << "ref: " << ref_ << std::endl;
            motor_node_.control();
        };
        motor_node_.get_motor().register_callback(l);

        // 定时器
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() -> void {
                // 打印 motor fps
                std::cout << "motor fps: " << motor_node_.get_motor().get_framerate() << std::endl;
            }
        );
    }

private:
    MotorRef motor_ref_;
    Connector<ConnectorType::CAN> connector;
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame> crn;
    ConnectorSendNode<ConnectorType::CAN, CanFrame> cs;
    MotorNode<MotorType::DJI_6020, 
        ControllerConfig<ControllerType::PID, ControllerBaseConfigNone>> 
        motor_node_;
    rclcpp::Subscription<MotorRef>::SharedPtr subscription_;
    rclcpp::Publisher<MotorFdb>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}