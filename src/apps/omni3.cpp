#include "src/motor_node/motor_node.hpp"
#include "motor/motor.hpp"
#include "connector/connector.hpp"
#include "msg_layer/msg_layer.hpp"

using connector::Connector;
using connector::ConnectorType;
using connector::ConnectorSingleRecvNode;
using connector::ConnectorSendNode;

using motor_node::MotorNode;
using motor_node::MotorType;

class Omni3Node : public rclcpp::Node {
public:
    Omni3Node() 
        : Node("omni3"), connector("can0"), crn(connector), cs(connector){
        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true);  // 启用进程内通信
        WheelMotor::Config config = {
            "wheel1",
            {crn, 1},
            10, options
        };
        motor1 = std::make_shared<WheelMotor>(config);
        
    }

private:
    using WheelMotor = MotorNode<MotorType::DJI_3508>;
    Connector<ConnectorType::CAN> connector;
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame> crn;
    ConnectorSendNode<ConnectorType::CAN, CanFrame> cs;
    WheelMotor::SharedPtr motor1;
    WheelMotor::SharedPtr motor2;
    WheelMotor::SharedPtr motor3;
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Omni3Node>());
    rclcpp::shutdown();
    return 0;
}