#pragma once

#include "common/debug/log.hpp"
#include "common/type_def.hpp"
#include "controller/controller.hpp"
#include "depend_on_ros12/attached_node/attached_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace attached_node {
using controller::Controller;
using controller::ControllerType;
using connector_common::BasicType;
using connector_common::for_each_unfolded;

template <ControllerType ControllerTypeT, typename... ControllerTypeArgs>
class ControllerNode : public AttachedNode<Controller<ControllerTypeT>, ControllerTypeArgs...> {
	public:
	ControllerNode(const std::string& name) : AttachedNode<Controller<ControllerTypeT>, ControllerTypeArgs...>(name) {}
};

}  // namespace controller_node