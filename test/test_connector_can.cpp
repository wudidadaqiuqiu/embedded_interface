#include "ros/ros.h"
#include "connector/connector_node.hpp"
#include "connector/msgpack.hpp"
#include "motor/motor.hpp"
#include "motor/motor_dji6020.hpp"

using connector::Connector;
using connector::ConnectorType;
using connector::ConnectorSingleRecvNode;
using connector::ConnectorSendNode;

using connector::CanFrame;
using motor::MotorDji6020Pack;

// rostopic pub /test_can_frame1 connector/IdPack "id: 1"
void connentor_once_test(Connector<ConnectorType::CAN>& connector);
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_can");
    
    Connector<ConnectorType::CAN> connector("can1");
    try {
        // connentor_once_test(connector);    
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }
    
    ros::NodeHandle nh;
    auto pub = nh.advertise<CanFrame::MSGT>("test_can_frame", 10);
    auto pub2 = nh.advertise<MotorDji6020Pack::MSGT>("motor6020_fdb", 10);
    auto l = [&pub, &pub2](const CanFrame::MSGT& msg) {
        if (msg.id > 0x204 && msg.id <= 0x204 + 0x4) {
            MotorDji6020Pack::MSGT msg2;
            data_convert(msg, msg2);
            pub2.publish(msg2);
        } else 
            pub.publish(msg);
    };
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame> crn(connector);
    crn.register_callback(l);
    ConnectorSendNode<ConnectorType::CAN, CanFrame> crn1(connector);
    auto sub = nh.subscribe("test_can_frame1", 10, decltype(crn1)::get_callback(), &crn1);
    ros::spin();
    return 0;
}

void connentor_once_test(Connector<ConnectorType::CAN>& connector) {
    std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04, 0x05};
    connector.con_send(data, 0x123);

    uint32_t id;
    connector.con_recv(data, id);
    for (auto byte : data) {
        std::cout << std::hex << (int)byte << " ";
    }
    std::cout << std::endl;
}