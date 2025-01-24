#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <array>
#include <algorithm>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("example_node");

    // 声明一个 std::vector 类型的参数
    node->declare_parameter<std::vector<int>>("my_array", {1, 2, 3, 4, 5});

    // 获取参数值
    auto param = node->get_parameter("my_array");
    auto vector_value = param.get_value<std::vector<int>>();

    // 将 std::vector 转换为 std::array
    std::array<int, 5> array_value;
    std::copy_n(vector_value.begin(), array_value.size(), array_value.begin());

    // 打印转换后的数组
    RCLCPP_INFO(node->get_logger(), "Array parameter values:");
    for (const auto &val : array_value) {
        RCLCPP_INFO(node->get_logger(), "%d", val);
    }

    rclcpp::shutdown();
    return 0;
}
