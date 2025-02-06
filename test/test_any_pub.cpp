#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <unordered_map>
#include <memory>
#include <any>

class MultiTypePublisher : public rclcpp::Node {
public:
    MultiTypePublisher() : Node("multi_type_publisher") {
        publishers_["float32"] = this->create_publisher<std_msgs::msg::Float32>("float_topic", 10);
        publishers_["float32_array"] = this->create_publisher<std_msgs::msg::Float32MultiArray>("array_topic", 10);
    }

    void publish_float32(float value) {
        auto pub = std::any_cast<std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>>>(publishers_["float32"]);
        std_msgs::msg::Float32 msg;
        msg.data = value;
        pub->publish(msg);
    }

    void publish_float32_array(const std::vector<float>& values) {
        auto pub = std::any_cast<std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>>>(publishers_["float32_array"]);
        std_msgs::msg::Float32MultiArray msg;
        msg.data = values;
        pub->publish(msg);
    }

private:
    std::unordered_map<std::string, std::any> publishers_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiTypePublisher>();

    node->publish_float32(3.14f);
    node->publish_float32_array({1.1f, 2.2f, 3.3f});

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
