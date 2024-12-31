#include "connector/connector.hpp"
#include "connector/connector_node.hpp"
#include "connector/IdPack.h"

using connector::Connector;
using connector::ConnectorType;
using connector::ConnectorRecvNode;
using connector::IdPack;
struct CanFrame;
void connentor_once_test(Connector<ConnectorType::CAN>& connector);
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_can");
    
    Connector<ConnectorType::CAN> connector("can0");
    try {
        connentor_once_test(connector);    
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }
    
    ConnectorRecvNode<ConnectorType::CAN, CanFrame> crn(connector, "test_can_frame");
    ros::spin();
    return 0;
}

struct CanFrame {
    using MSGT = IdPack;
    
    static IdPack pack(const std::vector<uint8_t>& data, uint32_t id) {
        auto msg = IdPack();
        msg.id = id;
        msg.data = data;
        return msg;
    }
};

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