#pragma once
#include <ros/ros.h>
#include <thread>
#include <std_msgs/UInt8MultiArray.h>

#include "connector/connector.hpp"
#include <atomic>

namespace connector {
template <ConnectorType CON_TYPE, typename MSGPackT>
class ConnectorRecvNode
{
private:
    ros::Publisher pub;
    Connector<CON_TYPE>& connector;
    std::thread thread;
    std::vector<uint8_t> buffer;
    uint32_t id;
    std::atomic<bool> is_end;
    
public:
    ConnectorRecvNode(ros::NodeHandle& nh, Connector<CON_TYPE>& con, std::string topic_name) :
        connector(con) {
        is_end = false;
        pub = nh.advertise<typename MSGPackT::MSGT>(topic_name, 10);
        thread = std::thread(&ConnectorRecvNode<CON_TYPE, MSGPackT>::run, this);
    }
    ConnectorRecvNode(const ConnectorRecvNode&) = delete;
    ~ConnectorRecvNode() {
        // 在这里设置成非阻塞没用
        is_end = true;
        if (thread.joinable()) {
            thread.join();
        }
    }

    void run() {
        while (!is_end) {
            try {
                connector.con_recv(buffer, id);
            } catch (const TimeoutException& e) {
                // std::cout << e.what() << std::endl;
                continue;
            }
            catch (const std::exception& e) {
                std::cout << "error: " << e.what() << std::endl;
                continue;
            }
            // std::cout << "recv: " << id << std::endl;
            pub.publish(MSGPackT::pack(buffer, id));
        }
    }

    // void reconnect

};

template <ConnectorType CON_TYPE, typename MSGPackT>
class ConnectorSendNode {
    ros::Subscriber sub;
    Connector<CON_TYPE>& connector;
    std::vector<uint8_t> buffer;
    uint32_t id_;

    public:
    ConnectorSendNode(ros::NodeHandle& nh, Connector<CON_TYPE>& con, std::string topic_name, uint32_t id = 0) : 
        connector(con),
        id_(id) {
        sub = nh.subscribe(topic_name, 10, &ConnectorSendNode::callback, this);
    }

    void callback(const typename MSGPackT::MSGT::ConstPtr& msg) {
        MSGPackT::unpack(msg, buffer, id_);

        try {
            connector.con_send(buffer, id_);
        } catch (const std::exception& e) {
            std::cout << "error: " << e.what() << std::endl;
            return;
        }
    }

};

}