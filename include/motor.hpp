#pragma once
#include "connector/msgpack.hpp"
#include "connector/connector_node.hpp"

using connector::CanFrame;
using connector::ConnectorType;
using connector::ConnectorSingleRecvNode;

enum MotorType {
    DJI_6020 = 0,
};

using MotorId = uint32_t;

template <MotorType MotorT>
class Motor { };

template <>
class Motor<MotorType::DJI_6020> {
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame>* rnode_;
    MotorId id_;
    // MotorFdb data;
    public:
    static constexpr MotorId BASE_ID = 0x204;
    Motor(ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame>* rnode_,
        MotorId id) : 
        rnode_(rnode_), id_(id) {
        
        auto l2 = [this](const CanFrame::MSGT& msg) {
            if (msg.id == BASE_ID + id_) {
                
            }
        };
        rnode_->register_callback(l2);
    }
    // Motor(std::string recv_topic) {}

    
};
