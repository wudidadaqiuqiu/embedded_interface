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

struct MyPack {
    using MSGT = IdPack;

    static void unpack(const IdPack::ConstPtr& msg, std::vector<uint8_t>& data, uint32_t& id) {
        
    }
};
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_can_loopback_lantency");
    
    Connector<ConnectorType::CAN> connector("can1");
    ros::NodeHandle nh;
    auto pub = nh.advertise<CanFrame::MSGT>("test_can_frame", 10);
    auto l = [&pub](const CanFrame::MSGT& msg) {
        pub.publish(msg);
    };
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame> crn(connector);
    ConnectorSendNode<ConnectorType::CAN, CanFrame> crn1(connector);
    auto sub = nh.subscribe<CanFrame::MSGT>("test_can_lantency", 10, &decltype(crn1)::callback, &crn1);
    ros::spin();
    return 0;
}