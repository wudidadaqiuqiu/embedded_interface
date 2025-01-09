
#include "rclcpp/rclcpp.hpp"
#include "connector/connector.hpp"
#include "motor/motor.hpp"
#include "controller/controller.hpp"
#include "motor_node/motor_node.hpp"
#include "msg_layer/msg_layer.hpp"
#include "robot_msg/msg/motor_ref.hpp"
#include "robot_msg/msg/pid_param_set.hpp"

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

using con_used_msg::PidParam;
using con_used_msg::PidParamStruct;
using robot_msg::msg::PidParamSet; 

constexpr motor::MotorId motor_id = 1;
class MotorControlNode : public rclcpp::Node {
    static constexpr PidParamStruct PID_PARAM = {
        .kp = 10.0 * 3.0 / 16384.0 ,
        .ki = 0.7 * 3.0 / 16384.0,
        .kd = 600.0 * 3.0 / 16384.0,
        .error_max = 500.0 * 3.0 / 16384.0,
        .irange = 40.0 * 3.0 / 16384.0,
        .outmax = 3.0,
    };
public:
    MotorControlNode() 
        : Node("test_motor"), 
        connector("can0"), crn(connector), cs(connector),
        motor_node_(
            {&crn, motor_id}, PID_PARAM){
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

        subscription_controller_param = this->create_subscription<PidParamSet>(
            "/motor6020_param", 10, [&](const PidParamSet::SharedPtr msg) {
                if (msg->set == true)
                    motor_node_.get_controller_config().param = msg->param;
                else {
                    connector_common::data_convert
                        <PidParamStruct, decltype(motor_node_.get_controller_config().param)>
                        (PID_PARAM, motor_node_.get_controller_config().param);
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
            // std::cout << "kp: " << motor_node_.get_controller_config().kp << std::endl;
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
    // rclcpp::Subscription<MotorRef>::SharedPtr subscription_;
    rclcpp::Subscription<MotorRef>::SharedPtr subscription_;
    rclcpp::Subscription<PidParamSet>::SharedPtr subscription_controller_param;

    rclcpp::Publisher<MotorFdb>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}