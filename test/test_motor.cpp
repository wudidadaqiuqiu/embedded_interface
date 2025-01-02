#include "ros/ros.h"
#include "motor.hpp"

using connector::Connector;
using connector::ConnectorType;
using connector::ConnectorSingleRecvNode;
using connector::ConnectorSendNode;
using connector::IdPack;
using motor::MotorPack;
using motor::Motor;
using motor::MotorType;
using connector::CanFrame;

// rostopic pub /test_can_lantency connector/IdPack "id: 1"
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_motor");
    ros::NodeHandle nh;
    
    Connector<ConnectorType::CAN> connector("can0");
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame> crn(connector);
    Motor<MotorType::DJI_6020> motor(&crn, 1);
    ros::spin();
    return 0;
}