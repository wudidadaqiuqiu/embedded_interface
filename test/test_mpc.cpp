#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "robot_msg/msg/two_float.hpp"
#include "mpc.hpp"

#include <iostream>
using namespace Eigen;
using ModelT = DiscreteStateSpaceModel<2, 1, 2>;
using MPC = ModelPredictController<2, 1, 2, 3, 2>;
using std_msgs::msg::Float32MultiArray;
using std_msgs::msg::Float32;
using robot_msg::msg::TwoFloat;
class MPCNode : public rclcpp::Node {
public:
    ModelT model;
    MPC mpc;
    MPC::QarrT Qarr;
    MPC::ParrT Parr;
    MPC::X0T x0;
    MPC::ZrefarrT zrefarr;
    rclcpp::Subscription<Float32MultiArray>::SharedPtr param_sub;
    rclcpp::Subscription<Float32MultiArray>::SharedPtr fdb_sub;
    rclcpp::Subscription<TwoFloat>::SharedPtr ref_sub;
    rclcpp::Publisher<Float32>::SharedPtr output_pub;
    std::mutex mtx;
    MPCNode() : Node("mpc_node"), model(0.001), mpc(model) {
        constexpr float dt = 0.001;
        ModelT::AmatT A;
        ModelT::BmatT B;
        ModelT::CmatT C;
        A << 1, dt, 0, 1;
        B << 0.5 * dt * dt, dt;
        C << 1, 0, 0, 1;
        model.ChangeModel(A, B, C);
        Qarr[0] << 10;
        Qarr[1] << 10;

        Parr[0] << 1, 0, 0, 0.001;
        Parr[1] << 1, 0, 0, 0.001;
        Parr[2] << 1, 0, 0, 0.001;
        mpc.ChanegeParams(Qarr, Parr);
        std::cout << "\n" << mpc.get_gain() << std::endl;
        x0 << 0, 0;
        zrefarr[0] << 0.7, 0;
        zrefarr[1] << 0.7, 0;
        zrefarr[2] << 0.7, 0;
        std::cout << mpc.Calc(x0, zrefarr) << std::endl;

        param_sub = this->create_subscription<Float32MultiArray>(
            "/mpc_param", 10, [&](const Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() != 14) {
                    RCLCPP_ERROR(this->get_logger(), "Invalid param size");
                    return;
                }
                std::lock_guard<std::mutex> lock(mtx);
                Qarr[0] << msg->data[0];
                Qarr[1] << msg->data[1];
                Parr[0] << msg->data[2], msg->data[3], msg->data[4], msg->data[5];
                Parr[1] << msg->data[6], msg->data[7], msg->data[8], msg->data[9];
                Parr[2] << msg->data[10], msg->data[11], msg->data[12], msg->data[13];
                mpc.ChanegeParams(Qarr, Parr);
            }
        );
        output_pub = this->create_publisher<Float32>("/mpc_output", 10);
        fdb_sub = this->create_subscription<Float32MultiArray>(
            "/mpc_fdb", 10, [&](const Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() != 2) {
                    RCLCPP_ERROR(this->get_logger(), "Invalid fdb size");
                    return;
                }
                // for (int i = 0; i < 3; i++) {
                //     zrefarr[i] << msg->data[2], msg->data[3];
                // }
                x0 << msg->data[0], msg->data[1];
                std::lock_guard<std::mutex> lock(mtx);
                auto& output = mpc.Calc(x0, zrefarr);
                Float32 output_msg;
                output_msg.data = output(0, 0);
                output_pub->publish(output_msg);
            }
        );
        ref_sub = this->create_subscription<TwoFloat>(
            "/mpc_ref", 10, [&](const TwoFloat::SharedPtr msg) {
                // if (msg->data.size() != 2) {
                //     RCLCPP_ERROR(this->get_logger(), "Invalid ref size");
                //     return;
                // }
                std::cout << "get ref: " << msg->x1 << " " << msg->x2 << std::endl;
                for (int i = 0; i < 3; i++) {
                    zrefarr[i] << msg->x1, msg->x2;
                }
            }
        );
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCNode>());
    rclcpp::shutdown();
    return 0;
}