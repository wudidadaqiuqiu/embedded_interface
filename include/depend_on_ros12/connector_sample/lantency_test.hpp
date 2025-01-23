#include <chrono>

#include "connector/connector_node.hpp"
#include "connector/msgpack.hpp"
#include "ros/ros.h"

namespace connector {

class LatencyTest {
    std::string device_name_;
    Connector<ConnectorType::CAN> connector;
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame> rnode;
    ConnectorSendNode<ConnectorType::CAN, CanFrame> snode;
    ros::Publisher pub;
    ros::Subscriber sub;

    std::chrono::time_point<std::chrono::high_resolution_clock> start;

   public:
    LatencyTest(ros::NodeHandle& nh,
                const std::string& device_name,
                const std::string& send_topic_name, const std::string& recv_topic_name,
                uint32_t id, uint32_t recv_id)
        : device_name_(device_name), connector(device_name), rnode(connector), snode(connector) {
        auto l1 = [this, id](const CanFrame::MSGT::ConstPtr& msg) {
            std::cout << device_name_ << " sub" << msg->id << std::endl;
            if (msg->id == id)
                start = std::chrono::high_resolution_clock::now();
        };
        snode.register_callback(l1);
        sub = nh.subscribe(send_topic_name, 10, decltype(snode)::get_callback(), &snode);

        // pub = nh.advertise<CanFrame::MSGT>(recv_topic_name, 10);
        auto l2 = [this, recv_id](const CanFrame::MSGT& msg) {
            if (msg.id == recv_id) {
                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> duration = end - start;
                std::cout << "Time taken: " << duration.count() << " seconds" << std::endl;
            }
        };
        rnode.register_callback(l2);
    }
};

}  // namespace connector