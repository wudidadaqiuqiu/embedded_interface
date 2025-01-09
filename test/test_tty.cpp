#include "rclcpp/rclcpp.hpp"
#include "connector/connector.hpp"

using connector::Connector;
using connector::ConnectorType;
using connector::ConnectorSingleRecvNode;
using connector::ConnectorSendNode;
using connector::IdPack;
using connector::TtyFrame;
inline void printArray(const uint8_t* arr, size_t length) {
    std::cout << std::hex;
    for (size_t i = 0; i < length; ++i) {
        std::cout << "0x" << static_cast<int>(arr[i]) << " ";
    }

    std::cout << std::dec;
    if (length != 0 && arr[length - 1] == 0x7e)
        std::cout << std::endl;
}

class TtyNode : public rclcpp::Node {
public:
    TtyNode() 
        : Node("test_tty"), 
        connector(), crn(connector), cs(connector) {
        connector.con_open("/dev/ttyUSB0");
        std::cout << "open tty" << std::endl;
        crn.register_callback([&](const TtyFrame::MSGT& frame) -> void {
            std::cout << "recv: ";
            printArray(frame.data.data(), frame.data.size());
            // RCLCPP_INFO(this->get_logger(), "recv: %s", frame.data.c_str());
        });

        // 定时器
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() -> void {
            }
        );
    }

private:
    Connector<ConnectorType::TTY> connector;
    ConnectorSingleRecvNode<ConnectorType::TTY, TtyFrame> crn;
    ConnectorSendNode<ConnectorType::TTY, TtyFrame> cs;
    // rclcpp::Subscription<MotorRef>::SharedPtr subscription_;
    // rclcpp::Subscription<PidParamSet>::SharedPtr subscription_controller_param;
    // rclcpp::Publisher<MotorFdb>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TtyNode>());
    rclcpp::shutdown();
    return 0;
}