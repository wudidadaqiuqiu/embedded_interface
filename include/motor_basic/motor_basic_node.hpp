#pragma once

#include "connector/connector_node.hpp"
#include "controller/controller.hpp"
#include "motor/motor.hpp"

namespace motor_node {
using connector::ConnectorSendNode;
using connector::CanFrame;
using controller::Controller;
using controller::ControllerType;
using motor::Motor;
using motor::MotorConfig;
using motor::MotorType;

template <MotorType MotorT, ControllerType ControllerTypeT,
		  typename... ControllerArgs>
class MotorBasicNode {
	Motor<MotorT> motor;
	Controller<ControllerTypeT>::template Type<ControllerArgs...> controller;
	typename Motor<MotorT>::ConnectorSendNodeT connector_send_node;

   public:
	MotorBasicNode(const Motor<MotorT>::Config& motor_config,
			  const typename Controller<ControllerTypeT>::template ConstructT<ControllerArgs...>&
				  controller_config)
		: motor(motor_config),
		  controller(controller_config),
		  connector_send_node(motor.get_connector()) {}

	void calc_control(const decltype(controller.fdb)& fdb,
					  const decltype(controller.ref)& ref) {
		controller.fdb = fdb;
		controller.ref = ref;
		controller.update();
	}

	void calc_control() { controller.update(); }

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
	auto& get_controller_config() { return controller.config; }
	void set_fdb(const decltype(controller.fdb)& fdb) { controller.fdb = fdb; }
	void set_ref(const decltype(controller.ref)& ref) { controller.ref = ref; }
};

}  // namespace motor_node