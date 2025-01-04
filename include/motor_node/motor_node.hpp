#pragma once

#include "connector/connector_node.hpp"
#include "motor/motor.hpp"
#include "controller/controller.hpp"

namespace motor_node {
using connector::ConnectorSendNode;
using motor::Motor;
using motor::MotorConfig;
using motor::MotorType;
using controller::ControllerBase;
using controller::Controller;
using controller::ControllerConfig;

template <MotorType MotorT, typename ControllerConfigT>
class MotorNode {
    Motor<MotorT> motor;
    Controller<ControllerConfigT> controller;
    typename Motor<MotorT>::ConnectorSendNodeT connector_send_node;
public:
    MotorNode(const MotorConfig<MotorT>& motor_config, const ControllerConfigT& controller_config) : 
        motor(motor_config), controller(controller_config), connector_send_node(motor.get_connector()) {}

    void calc_control(const decltype(controller.fdb)& fdb, const decltype(controller.ref)& ref) {
        controller.fdb = fdb;
        controller.ref = ref;
        controller.update();
    }
    Motor<MotorT>& get_motor() { return motor; }
    
    void control() {
        CanFrame::MSGT id_pack;
        id_pack.data.resize(8);
        // std::cout << controller.out << std::endl;
        id_pack.id = motor.set_send_buf(controller.out, id_pack.data);
        connector_send_node.send(id_pack);
    }

    uint32_t control(CanFrame::MSGT& id_pack) {
        uint32_t id = motor.set_send_buf(controller.out, id_pack.data);
        return id;
    }
};

}