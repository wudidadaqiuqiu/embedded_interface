#include "ros/ros.h"
#include "connector/connector_node.hpp"
#include "motor/motor.hpp"
#include "controller/controller.hpp"
#include "motor_node/motor_node.hpp"
#include "robot_msg/MotorRef.h"
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
using robot_msg::MotorRef;

// rostopic pub /test_can_lantency connector/IdPack "id: 1"
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_motor");
    ros::NodeHandle nh;
    
    Connector<ConnectorType::CAN> connector("can0");
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame> crn(connector);
    ConnectorSendNode<ConnectorType::CAN, CanFrame> cs(connector);
    constexpr motor::MotorId motor_id = 1;
    // Motor<MotorType::DJI_6020> motor({&crn, motor_id});
    // Controller<ControllerConfig<ControllerType::PID, ControllerBaseConfigNone>> 
    // pid_controller({10.0 * 3.0 / 16384.0, 0, 0, 0, 3, 0});
    MotorNode<MotorType::DJI_6020, ControllerConfig<ControllerType::PID, ControllerBaseConfigNone>> 
        motor_node({&crn, motor_id}, {
            .kp = 10.0 * 3.0 / 16384.0,
            .ki = 0.7 * 3.0 / 16384.0,
            .kd = 600.0 * 3.0 / 16384.0,
            .error_max = 500.0 * 3.0 / 16384.0,
            .Irange = 40.0 * 3.0 / 16384.0,
            .outmax = 3
        }
    );

    MotorRef motor_ref;
    motor_ref.pos_ref.resize(1);
    motor_ref.pos_ref[0].num = 0;

    auto sub = nh.subscribe<MotorRef>("/motor6020_cmd", 10, [&](const MotorRef::ConstPtr& msg) {
        motor_ref = *msg;
    });
    
    auto pub = nh.advertise<MotorFdb>("/motor6020_fdb", 10);
    auto l = [&](const CanFrame::MSGT& msg) {
        pub.publish(motor_node.get_motor().get_fdb());
        real ref_;
        if (motor_ref.pos_ref.size() > 0) {
            ref_ = motor_ref.pos_ref[0].num;
        }
        motor_node.calc_control(motor_node.get_motor().get_fdb().pos_zero_cross.deg.num, ref_);
        std::cout << "ref: " << ref_ << std::endl;
        motor_node.control();
    };
    motor_node.get_motor().register_callback(l);

    ros::Timer timer = nh.createTimer(ros::Duration(1.0), 
        [&motor_node](const ros::TimerEvent&) {
            // std::cout << "timer callback" << std::endl;
            std::cout << "motor fps: " << motor_node.get_motor().get_framerate() << std::endl;
        }
    );
    ros::spin();
    return 0;
}
