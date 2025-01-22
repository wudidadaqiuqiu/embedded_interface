#include "controller/controller.hpp"
#include "src/motor_node/motor_node.hpp"
#include "motor/motor.hpp"
#include "connector/connector.hpp"
#include "msg_layer/msg_layer.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
using connector::Connector;
using connector::ConnectorType;
using connector::ConnectorSingleRecvNode;
using connector::ConnectorSendNode;

using motor_node::MotorNode;
using motor_node::MotorType;

class Omni3Node : public rclcpp::Node {
public:
    Omni3Node() 
        : Node("omni3"), connector("can0"), crn(connector), cs(connector){
        create_wheel_motor(1);
        create_wheel_motor(2);
        create_wheel_motor(3);
        refsubscription = this->create_subscription<std_msgs::msg::Float32MultiArray>
            ("vel3", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            for (int i = 0; i < 3; i++){
                ref[i] = msg->data[i];
            }
        });
        timer_ =
            this->create_wall_timer(std::chrono::milliseconds(1), [this]() -> void {
                CanFrame::MSGT id_pack;
                id_pack.data.resize(8);
                motors[0]->get_motor().set_send_buf(controllers[0].out, id_pack.data);
                motors[1]->get_motor().set_send_buf(controllers[1].out, id_pack.data);
                id_pack.id = motors[2]->get_motor().set_send_buf(controllers[2].out, id_pack.data);
                cs.send(id_pack);
            });
    }
    
    void create_wheel_motor(motor::MotorId id){
        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true);  // 启用进程内通信
        // LOG_INFO(1, "name: %s, id: %d", std::to_string(id).c_str(), id);
        WheelMotor::Config config = {
            "wheel" + std::to_string(id),
            {crn, id},
            10, options
        };
        size_t index = id - 1;
        motors[index] = std::make_shared<WheelMotor>(config);

        decltype(controllers[index].config)::ConstructT param = {
            .kp = 10.0 * 20.0 / 16384.0,
            .kd = 0.0,
            .outmax = 10.0,
        };
        controllers[index].config = param;
        subscriptions_[index] = this->create_subscription<MotorFdb>(
            motors[index]->fdb_topic, 10, [this, index](const MotorFdb::SharedPtr msg) {
                // motor_node_.set_fdb(motor_node_.get_motor().get_fdb().pos_zero_cross.deg.num);
                controllers[index].fdb(0) = msg->vel.rad.num;
                controllers[index].fdb(1) = 0;

                controllers[index].ref(0) = ref[index];
                controllers[index].ref(1) = 0;

                // LOG_INFO(1, "kp: %f outmax: %f", controllers[index].config.kp, controllers[index].config.outmax);
                controllers[index].update();
                // LOG_INFO(1, "out: %f", controllers[index].out);
            }
        );
    }

private:
    using WheelMotor = MotorNode<MotorType::DJI_3508>;
    Connector<ConnectorType::CAN> connector;
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame> crn;
    ConnectorSendNode<ConnectorType::CAN, CanFrame> cs;
    std::array<WheelMotor::SharedPtr, 3> motors;
    std::array<controller::LqrController, 3> controllers;
    std::array<rclcpp::Subscription<MotorFdb>::SharedPtr, 3> subscriptions_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr refsubscription;
    
    std::array<float, 3> ref;
    rclcpp::TimerBase::SharedPtr timer_;
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Omni3Node>());
    rclcpp::shutdown();
    return 0;
}