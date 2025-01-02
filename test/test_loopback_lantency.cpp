#include "ros/ros.h"
#include "connector/connector_node.hpp"
#include "connector/msgpack.hpp"

#include "connector/funtional/lantency_test.hpp"
using connector::Connector;
using connector::ConnectorType;
using connector::ConnectorSingleRecvNode;
using connector::ConnectorSendNode;
using connector::IdPack;
using connector::MotorPack;
using connector::CanFrame;
using connector::LatencyTest;

// rostopic pub /test_can_lantency connector/IdPack "id: 1"
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_can_loopback_lantency");
    ros::NodeHandle nh;
    
    LatencyTest latency_test(nh, "can1", "test_can_lantency", "test_can_lantency_recv", 1, 2);
    Connector<ConnectorType::CAN> connector("can0");
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame> crn(connector);
    ConnectorSendNode<ConnectorType::CAN, CanFrame> crn1(connector);

    auto pub = nh.advertise<CanFrame::MSGT>("test_can_lantency_cli", 10);
    auto l = [&pub](const CanFrame::MSGT& msg) {
        if (msg.id == 1) {
            std::cout << "recv: " << msg.id << std::endl;
            CanFrame::MSGT out;
            out.id = 2;
            pub.publish(out);
        }
    };
    crn.register_callback(l);
    auto sub = nh.subscribe("test_can_lantency_cli", 10, decltype(crn1)::get_callback(), &crn1);
    ros::spin();
    return 0;
}