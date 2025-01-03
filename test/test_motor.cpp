#include "ros/ros.h"
#include "motor/motor.hpp"
#include "motor/motor_dji6020.hpp"

using connector::Connector;
using connector::ConnectorType;
using connector::ConnectorSingleRecvNode;
using connector::ConnectorSendNode;
using connector::IdPack;
using connector::CanFrame;
using motor::Motor;
using motor::MotorType;

// rostopic pub /test_can_lantency connector/IdPack "id: 1"
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_motor");
    ros::NodeHandle nh;
    
    Connector<ConnectorType::CAN> connector("can0");
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame> crn(connector);
    Motor<MotorType::DJI_6020> motor(&crn, 1);
    auto pub = nh.advertise<MotorFdb>("/motor6020_fdb", 10);
    auto l = [&pub, &motor](const CanFrame::MSGT& msg) {
        pub.publish(motor.get_fdb());
    };
    motor.register_callback(l);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), 
        [&motor](const ros::TimerEvent&) {
            // std::cout << "timer callback" << std::endl;
            std::cout << "motor fps: " << motor.get_framerate() << std::endl;
        }
    );
    ros::spin();
    return 0;
}
