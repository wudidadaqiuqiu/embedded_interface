#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "depend_on_ros12/attached_node/attached_node.hpp"
#include "observer/observer.hpp"

using std_msgs::msg::Float32;
using std_msgs::msg::Float32MultiArray;

using connector_common::concat;
using connector_common::fsgn;
using observer::ObserverType;
using observer::Observer;
// using observer::StateSpaceModel;
using observer::VarientStateSpaceModel;
using observer::KalmanFilter;
using attached_node::AttachedNode;
template <ObserverType ObserverTypeT, typename... ObserverArgs>
using ObserverNode = AttachedNode<Observer<ObserverTypeT>, ObserverArgs...>;

class ObserverTestNode : public rclcpp::Node {
public:
    ObserverTestNode() 
        : Node("observer_test_node"),
        kf_node(kfconfig)
        {
        td_node.init(*this, concat("td"));
        kf_node.init(*this, concat("kf"));
        const float Jdown = 1.0 / 0.1f;
        const float b_ = 0.02f;
        const float delatt = 0.001f;
        const float a = 0.00479301;
        const float b = 0.01457816;
        const float c = 0.08997533;
        kf_node.get().config.model.A << 1 - b_ * Jdown * delatt, 0.3 * Jdown * delatt, 0, 1;
        kf_node.get().config.model.H << 1, 0, 0, 1, 0, 0;
        kf_node.get().config.model.get_H_update_func() = [this, a, b, c](
            const auto& x, auto& H) {
            (void)x;
            (void)H;

            H << 1, 0,
                 0, 1,
                (a * fsgn(omega * current) + b) * current, 
                (a * fsgn(omega * current) + b) * omega + 2 * c * current;
            // LOG_DEBUG(1, "jacobian_H_update_func");
            return; 
        };
        kf_node.get().config.model.get_A_update_func() = [](
            const auto& x, const auto& u, auto& A) {
            (void)x;
            (void)u;
            (void)A;
            // LOG_DEBUG(1, "jacobian_A_update_func");
            return; 
        };
        kf_node.get().config.model.config.h = [a, b, c](const Eigen::Vector<float, 2>& x, Eigen::Vector<float, 3>& zpre) {
            float om = x(0);
            float cur = x(1);
            // float pwr = x(2);
            zpre << om, cur, (a * fsgn(om * cur) + b) * cur * om + c * cur * cur + 1.04431114f;
        };
        pub_alpha = this->create_publisher<Float32MultiArray>("/observer/motor_alpha", 10);
        pub_power = this->create_publisher<Float32>("/observer/power", 10);
        omega_sub = this->create_subscription<Float32>(
            "motor_omega", 10, [this](const Float32::SharedPtr msg) {
            // pub_alpha->publish(*msg.get());
            td_node.get().update({msg.get()->data});
            msg_.data = {td_node.get().get_state().v1, td_node.get().get_state().v2};
            pub_alpha->publish(msg_);
            omega = msg->data;
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
            power_real = msg->data;
        });
        motor_current_sub = this->create_subscription<Float32>(
            "motor_current", 10, [this](const Float32::SharedPtr msg) {
            current = msg->data;

            ObserverKf::UpdateData update_data = {
                // .z = {omega, power_real},
                .z = {omega, current, power_real},
            };
            kf_node.get().predict(ObserverKf::PredictData{});
            kf_node.get().update(update_data);
            msg_power.data = kf_node.get().get_state().x.front();
            pub_power->publish(msg_power);
            power_real = msg->data;
        });
        timer_ =
            this->create_wall_timer(std::chrono::milliseconds(1), [this]() -> void {

            });
    }
    
private:
    KalmanFilter<VarientStateSpaceModel<2, 0, 3>>::Config kfconfig;
    ObserverNode<ObserverType::TD> td_node;
    ObserverNode<ObserverType::KF, decltype(kfconfig.model)> kf_node;
    using ObserverKf = std::remove_reference_t<decltype(kf_node.get())>;
    rclcpp::Subscription<Float32>::SharedPtr omega_sub;
    rclcpp::Subscription<Float32>::SharedPtr power_real_sub;
    rclcpp::Subscription<Float32>::SharedPtr motor_current_sub;

    rclcpp::Publisher<Float32MultiArray>::SharedPtr pub_alpha;
    rclcpp::Publisher<Float32>::SharedPtr pub_power;
    Float32MultiArray msg_;
    Float32 msg_power;
    float current;
    float omega;
    float power_real;
    rclcpp::TimerBase::SharedPtr timer_;

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