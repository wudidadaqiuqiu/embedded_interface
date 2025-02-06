#pragma once

#include "common/debug/log.hpp"
#include "common/type_def.hpp"
#include "common/concepts.hpp"
#include "common/param_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <any>

#define ATTACHED_NODE_DEBUG (1)
namespace attached_node {
using connector_common::BasicType;
using connector_common::for_each_unfolded;
using connector_common::to_string;
using connector_common::PairHint;

template <BasicType::Type TypeT>
struct MsgTypeStruct {
    using MsgType = void;
};

template<>
struct MsgTypeStruct<BasicType::Type::FLOAT> {
    using MsgType = std_msgs::msg::Float32;
};


class WatchNode { 
protected:
    rclcpp::Node* node_;
    std::vector<std::shared_ptr<rclcpp::PublisherBase>> publishers_;
    std::vector<rclcpp::TimerBase::SharedPtr> timers;
public:
	WatchNode(rclcpp::Node& node) { node_ = &node; };

    template <typename T, BasicType::Type TypeT = BasicType::type<T>()>
    void add_publisher(const T& data, int mills, const std::string& topic) {
        auto publisher_ = node_->create_publisher<typename MsgTypeStruct<TypeT>::MsgType>(topic, 10);
        auto timer_ =
            node_->create_wall_timer(std::chrono::milliseconds(mills), 
            [this, &data, publisher_]() -> void {
            typename MsgTypeStruct<TypeT>::MsgType msg;
            msg.data = data;
            publisher_->publish(msg);
        });
        publishers_.push_back(std::static_pointer_cast<rclcpp::PublisherBase>(publisher_));
        timers.push_back(timer_);
    }
};

}