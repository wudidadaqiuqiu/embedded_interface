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
using attached_node::AttachedNode;
template <ObserverType ObserverTypeT, typename... ObserverArgs>
using ObserverNode = AttachedNode<Observer<ObserverTypeT>, ObserverArgs...>;

class ObserverTestNode : public rclcpp::Node {
public:
    ObserverTestNode() 
        : Node("observer_test_node") {
        td_node.init(std::shared_ptr<rclcpp::Node>(this));
        pub_alpha = this->create_publisher<Float32MultiArray>("/observer/motor_alpha", 10);
        refsubscription = this->create_subscription<Float32>(
            "motor_omega", 10, [this](const Float32::SharedPtr msg) {
            // pub_alpha->publish(*msg.get());
            td_node.get().update({msg.get()->data});
            msg_.data = {td_node.get().get_state().v1, td_node.get().get_state().v2};
            pub_alpha->publish(msg_);
        });
        timer_ =
            this->create_wall_timer(std::chrono::milliseconds(1), [this]() -> void {

            });
    }
    
private:
    ObserverNode<ObserverType::TD> td_node;
    rclcpp::Subscription<Float32>::SharedPtr refsubscription;
    rclcpp::Publisher<Float32MultiArray>::SharedPtr pub_alpha;
    Float32MultiArray msg_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObserverTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}