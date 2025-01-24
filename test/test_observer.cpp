#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "observer/observer.hpp"

using std_msgs::msg::Float32;
using observer::ObserverType;
using observer::Observer;

class ObserverTestNode : public rclcpp::Node {
public:
    ObserverTestNode() 
        : Node("observer_test_node") {
        pub_alpha = this->create_publisher<Float32>("/observer/motor_alpha", 10);
        refsubscription = this->create_subscription<Float32>(
            "motor_omega", 10, [this](const Float32::SharedPtr msg) {
            // pub_alpha->publish(*msg.get());
            
        });
        timer_ =
            this->create_wall_timer(std::chrono::milliseconds(1), [this]() -> void {

            });
    }
    
private:
    Observer<ObserverType::TD>::Type<2, 0, 1> td;
    rclcpp::Subscription<Float32>::SharedPtr refsubscription;
    rclcpp::Publisher<Float32>::SharedPtr pub_alpha; 
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObserverTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}