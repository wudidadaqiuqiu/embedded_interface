#pragma once

#include "common/type_def.hpp"
#include "controller/controller.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controller_node_attached {
using controller::Controller;
using controller::ControllerType;
using connector_common::BasicType;
using connector_common::for_each_unfolded;

template <ControllerType ControllerTypeT, typename... ControllerTypeArgs>
class ControllerNode {
	using ControllerPack = Controller<ControllerTypeT>;
	const std::string name_;
	ControllerPack::template Type<ControllerTypeArgs...> controller;
	rclcpp::Node::SharedPtr node_;
	std::vector<std::shared_ptr<rclcpp::ParameterEventHandler>> param_subscribers;
    std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> cb_handles;
   public:
	ControllerNode(const std::string& name="") : name_(name) {}

	using THIS = ControllerNode<ControllerTypeT, ControllerTypeArgs...>;
	struct DeclareAllParameters {
		template<std::size_t Index>
		static void func(const THIS* node_ptr) {
			const auto& pair = ControllerPack::Config::PARAM_MAP_DATA[Index];
			const std::string param_name = node_ptr->name_ + "_" + pair.first;
			if (node_ptr->node_->has_parameter(param_name)) {
				LOG_ERROR(1, "ControllerNode %s Parameter %s already exists", 
					node_ptr->name_.c_str(), param_name.c_str()
				);
				throw std::runtime_error("ControllerNode Parameter already exists");
			}
			node_ptr->node_->declare_parameter(param_name, BasicType::TypeT<pair.second>{});
			LOG_INFO(1, "ControllerNode %s Declared parameter: %s", 
				node_ptr->name_.c_str(), param_name.c_str()); 
		}
	};

	struct SubscribeAllParameters {
		template<std::size_t Index>
		static void func(THIS* const node_ptr) {
			const auto& pair = ControllerPack::Config::PARAM_MAP_DATA[Index];
			const std::string param_name = node_ptr->name_ + "_" + pair.first;
			auto param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(node_ptr->node_.get());
			auto cb = [node_ptr, pair](const rclcpp::Parameter & p) {
				auto value = p.get_value<BasicType::TypeT<pair.second>>(); // 获取值
				node_ptr->controller.config.template get<Index>() = value;
				std::string value_str = std::to_string(value); // 显式转换为字符串（根据类型调整转换逻辑）

				RCLCPP_INFO(
					node_ptr->node_->get_logger(), 
					"ControllerNode %s: Received an update to parameter \"%s\" of type %s: \"%s\"",
					node_ptr->name_.c_str(),
					p.get_name().c_str(),
					p.get_type_name().c_str(),
					value_str.c_str()
				);
			};
			auto cb_handle = param_subscriber->add_parameter_callback(param_name, cb);
			node_ptr->param_subscribers.push_back(param_subscriber);
			node_ptr->cb_handles.push_back(cb_handle);
		}
	};

	void init (const rclcpp::Node::SharedPtr& node) {
		node_ = node;
		for_each_unfolded<DeclareAllParameters, ControllerPack::Config::PARAM_MAP_DATA.size()>(this);
		for_each_unfolded<SubscribeAllParameters, ControllerPack::Config::PARAM_MAP_DATA.size()>(this);
	}

	auto get_controller() -> auto& {
		return controller;
	}

};

}  // namespace controller_node