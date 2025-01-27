#pragma once

#include "common/debug/log.hpp"
#include "common/type_def.hpp"
#include "common/concepts.hpp"
#include "common/param_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#define ATTACHED_NODE_DEBUG (1)
namespace attached_node {
using connector_common::BasicType;
using connector_common::for_each_unfolded;
using connector_common::to_string;
using connector_common::PairHint;

template <std::size_t M, typename ObjPackT, typename... Args>
#if __cplusplus >= 202002L
requires connector_common::IsPackOfObjects<ObjPackT, Args...>
#endif
class AttachedNode { 
protected:
    using ObjT = typename ObjPackT::template Type<Args...>;
	const std::array<char, M> name_;
    ObjT obj_;
    rclcpp::Node* node_;
    	std::vector<std::shared_ptr<rclcpp::ParameterEventHandler>> param_subscribers;
    std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> cb_handles;
public:
	AttachedNode(const std::array<char, M>& name) : name_(name) {}
    AttachedNode(const ObjPackT::template Config<Args...>& config, const std::array<char, M>& name
		) : name_(name), obj_(config) {}
    using THIS = AttachedNode<M, ObjPackT, Args...>;
    struct DeclareAllParameters {
		template<std::size_t Index>
		static void func(THIS* node_ptr) {
			PairHint pair_hint = node_ptr->obj_.config.template param_interface().template index_param_hint<Index>(node_ptr->name_);
			const std::string param_name = pair_hint.get_name();
			if (node_ptr->node_->has_parameter(param_name)) {
				LOG_ERROR(1, "AttachedNode %s Parameter %s already exists", 
					node_ptr->name_.data(), param_name.c_str()
				);
				throw std::runtime_error("AttachedNode Parameter already exists");
			}
			node_ptr->node_->declare_parameter(param_name, typename decltype(pair_hint)::ValueT{});
			LOG_DEBUG(ATTACHED_NODE_DEBUG, "param_name: %s, size: %ld", param_name.c_str(), param_name.size());
			auto value = node_ptr->node_->get_parameter(param_name).template get_value<typename decltype(pair_hint)::ValueT>(); // 获取值
			try {
				node_ptr->obj_.config.template set<Index>(value);
			} catch (const std::exception& e) {
			    LOG_ERROR(1, "AttachedNode Parameter %s set failed: %s", param_name.c_str(), to_string(value).c_str());
				throw std::runtime_error("AttachedNode Parameter set failed" + std::string(e.what()));
			}
			LOG_INFO(1, "AttachedNode %s Declared parameter: %s and set to \"%s\"", 
				node_ptr->name_.data(), param_name.c_str(), to_string(value).c_str());
		}
	};

    struct SubscribeAllParameters {
		template<std::size_t Index>
		static void func(THIS* const node_ptr) {
			PairHint pair_hint = node_ptr->obj_.config.template param_interface().template index_param_hint<Index>(node_ptr->name_);
			const std::string param_name = pair_hint.get_name();
			auto param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(node_ptr->node_);
			auto cb = [node_ptr, pair_hint](const rclcpp::Parameter & p) {
				auto value = p.get_value<typename decltype(pair_hint)::ValueT>(); // 获取值
				node_ptr->obj_.config.template set<Index>(value);
				std::string value_str = connector_common::to_string(value); // 显式转换为字符串（根据类型调整转换逻辑）

				RCLCPP_INFO(
					node_ptr->node_->get_logger(), 
					"AttachedNode %s: Received an update to parameter \"%s\" of type %s: \"%s\"",
					node_ptr->name_.data(),
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

    void init (rclcpp::Node& node) {
		node_ = &node;
		for_each_unfolded<DeclareAllParameters, decltype(obj_.config.param_interface())::PARAMS_COUNT>(this);
		for_each_unfolded<SubscribeAllParameters, decltype(obj_.config.param_interface())::PARAMS_COUNT>(this);
	}

    auto get() -> auto& {
        return obj_;
    }

};

}