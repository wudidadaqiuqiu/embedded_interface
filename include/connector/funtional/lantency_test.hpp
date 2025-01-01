#include "ros/ros.h"
#include "connector/connector_node.hpp"
#include "connector/msgpack.hpp"
#include <chrono>

namespace connector {


class LatencyTest {
    Connector<ConnectorType::CAN> connector;
    // ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame> rnode;
    ConnectorSendNode<ConnectorType::CAN, CanFrame> snode;
    ros::Publisher pub;

    std::chrono::time_point<std::chrono::high_resolution_clock> start;

    public:
    LatencyTest(ros::NodeHandle& nh, 
        const std::string& device_name,
        const std::string& send_topic_name, const std::string& recv_topic_name,
        uint32_t id)
        : connector(device_name), snode(connector) {
        // pub = nh.advertise<CanFrame::MSGT>(send_topic_name, 10);
        
        auto l = [this, id](const CanFrame::MSGT::ConstPtr& msg) {
            if (msg->id == id) 
                start = std::chrono::high_resolution_clock::now();
            // pub.publish(msg);
            // auto end = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double> duration = end - start;
            // std::cout << "Time taken: " << duration.count() << " seconds" << std::endl;
        };
        snode.register_callback(l);

        //  crn(connector, l);
        //  crn1(nh, connector, "test_can_lantency");
        // node_.create_publisher<msgpack::MsgPack>(topic_name_, 10);
    }

};


}