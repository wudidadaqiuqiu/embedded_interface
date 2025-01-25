#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "common/common_macro_dependencies.hpp"

#include "depend_on_ros12/attached_node/attached_node.hpp"
#include "observer/observer.hpp"

using std_msgs::msg::Float32;
using std_msgs::msg::Float32MultiArray;
using observer::ObserverType;
using observer::Observer;
using observer::StateSpaceModel;
using observer::KalmanFilter;
using attached_node::AttachedNode;
template <ObserverType ObserverTypeT, typename... ObserverArgs>
using ObserverNode = AttachedNode<Observer<ObserverTypeT>, ObserverArgs...>;

KalmanFilter<1, 0, 1>::Config config;

class ObserverTestNode : public rclcpp::Node {
public:
    ObserverTestNode() 
        : Node("observer_test_node"), kf(config) {
        td_node.init(std::shared_ptr<rclcpp::Node>(this));
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
            decltype(kf)::UpdateData update_data = {
                .z = {msg.get()->data},
            };
            kf.predict(decltype(kf)::PredictData{});
            kf.update(update_data);
            
            msg_power.data = state.x.front();
            pub_power->publish(msg_power);
        });
        timer_ =
            this->create_wall_timer(std::chrono::milliseconds(1), [this]() -> void {

            });
    }
    
private:
    ObserverNode<ObserverType::TD> td_node;
    KalmanFilter<1, 0, 1> kf;
    const decltype(kf)::StateData& state = kf.get_state();
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
    config.model.A << 1;
    // config.model.B << 0;
    config.model.H << 1;
    config.P << 1;
    config.Q << 0.1;
    config.R << 1;

    auto node = std::make_shared<ObserverTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}