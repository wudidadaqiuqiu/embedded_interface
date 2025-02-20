#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "depend_on_ros12/attached_node/attached_node.hpp"
#include "depend_on_ros12/attached_node/watch_node.hpp"
#include "observer/observer.hpp"

using std_msgs::msg::Float32;
using std_msgs::msg::Float32MultiArray;

using connector_common::concat;
using observer::ObserverType;
using observer::Observer;
using observer::StateSpaceModel;
using observer::KalmanFilter;
using attached_node::AttachedNode;
using attached_node::WatchNode;
template <ObserverType ObserverTypeT, typename... ObserverArgs>
using ObserverNode = AttachedNode<Observer<ObserverTypeT>, ObserverArgs...>;


class ObserverTestNode : public rclcpp::Node {
public:
    ObserverTestNode() 
        : Node("observer_test_node"),
         kf_node(kfconfig),
         watch_node(*this)
        {
        td_node.init(*this, concat("td"));
        kf_node.init(*this, concat("kf"));
        kf_node.get().config.model.A << 1;
        kf_node.get().config.model.H << 1;
        pub_alpha = this->create_publisher<Float32MultiArray>("/observer/motor_alpha", 10);
        omega_sub = this->create_subscription<Float32>(
            "motor_omega", 10, [this](const Float32::SharedPtr msg) {
            // pub_alpha->publish(*msg.get());
            td_node.get().update({msg.get()->data});
            msg_.data = {td_node.get().get_state().v1, td_node.get().get_state().v2};
            pub_alpha->publish(msg_);
        });
        power_real_sub = this->create_subscription<Float32>(
            "power_real", 10, [this](const Float32::SharedPtr msg) {
            ObserverKf::UpdateData update_data = {
                .z = {msg.get()->data},
            };
            kf_node.get().predict(ObserverKf::PredictData{});
            kf_node.get().update(update_data);
        });
        watch_node.add_publisher(kf_node.get().get_state().x.front(), 1, "/observer/power");
    }
    
private:
    KalmanFilter<StateSpaceModel<1, 0, 1>>::Config kfconfig;
    ObserverNode<ObserverType::TD> td_node;
    ObserverNode<ObserverType::KF, decltype(kfconfig.model)> kf_node;
    WatchNode watch_node;
    using ObserverKf = std::remove_reference_t<decltype(kf_node.get())>;
    rclcpp::Subscription<Float32>::SharedPtr omega_sub;
    rclcpp::Subscription<Float32>::SharedPtr power_real_sub;

    rclcpp::Publisher<Float32MultiArray>::SharedPtr pub_alpha;
    Float32MultiArray msg_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObserverTestNode>();
    rclcpp::spin(node);
    LOG_INFO(1, "Shutting down");
    rclcpp::shutdown();
    LOG_INFO(1, "Shutdown complete");
    return 0;
}