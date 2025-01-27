#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "common/common_macro_dependencies.hpp"

#include "depend_on_ros12/attached_node/attached_node.hpp"
#include "observer/observer.hpp"

using std_msgs::msg::Float32;
using std_msgs::msg::Float32MultiArray;

using connector_common::concat;
using observer::ObserverType;
using observer::Observer;
using observer::StateSpaceModel;
using observer::KalmanFilter;
using attached_node::AttachedNode;
template <std::size_t M, ObserverType ObserverTypeT, typename... ObserverArgs>
using ObserverNode = AttachedNode<M, Observer<ObserverTypeT>, ObserverArgs...>;

KalmanFilter<1, 0, 1>::Config kfconfig;

class ObserverTestNode : public rclcpp::Node {
public:
    ObserverTestNode() 
        : Node("observer_test_node"),
        td_node(concat("td"))
        , kf_node(kfconfig, concat("kf"))
        {
        td_node.init(*this);
        kf_node.init(*this);
        pub_alpha = this->create_publisher<Float32MultiArray>("/observer/motor_alpha", 10);
        pub_power = this->create_publisher<Float32>("/observer/power", 10);
        omega_sub = this->create_subscription<Float32>(
            "motor_omega", 10, [this](const Float32::SharedPtr msg) {
            // pub_alpha->publish(*msg.get());
            td_node.get().update({msg.get()->data});
            msg_.data = {td_node.get().get_state().v1, td_node.get().get_state().v2};
            pub_alpha->publish(msg_);
        });
        power_real_sub = this->create_subscription<Float32>(
            "power_real", 10, [this](const Float32::SharedPtr msg) {
            // ObserverKf::UpdateData update_data = {
            //     .z = {msg.get()->data},
            // };
            // kf_node.get().predict(ObserverKf::PredictData{});
            // kf_node.get().update(update_data);
            
            // msg_power.data = kf_node.get().get_state().x.front();
            // pub_power->publish(msg_power);
        });
        timer_ =
            this->create_wall_timer(std::chrono::milliseconds(1), [this]() -> void {

            });
    }
    
private:
    ObserverNode<3, ObserverType::TD> td_node;
    ObserverNode<3, ObserverType::KF, decltype(kfconfig.model)> kf_node;
    using ObserverKf = std::remove_reference_t<decltype(kf_node.get())>;
    rclcpp::Subscription<Float32>::SharedPtr omega_sub;
    rclcpp::Subscription<Float32>::SharedPtr power_real_sub;

    rclcpp::Publisher<Float32MultiArray>::SharedPtr pub_alpha;
    rclcpp::Publisher<Float32>::SharedPtr pub_power;
    Float32MultiArray msg_;
    Float32 msg_power;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    kfconfig.model.A << 1;
    // config.model.B << 0;
    kfconfig.model.H << 1;
    kfconfig.P << 1;
    auto node = std::make_shared<ObserverTestNode>();
    rclcpp::spin(node);
    LOG_INFO(1, "Shutting down");
    rclcpp::shutdown();
    LOG_INFO(1, "Shutdown complete");
    return 0;
}