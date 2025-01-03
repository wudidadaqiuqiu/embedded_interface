#include "ros/ros.h"
#include "connector/connector_node.hpp"
#include "motor/motor.hpp"
#include "controller/controller.hpp"

using connector::Connector;
using connector::ConnectorType;
using connector::ConnectorSingleRecvNode;
using connector::ConnectorSendNode;
using connector::IdPack;
using connector::CanFrame;
using motor::Motor;
using motor::MotorType;
using controller::PidController;

// rostopic pub /test_can_lantency connector/IdPack "id: 1"
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_motor");
    ros::NodeHandle nh;
    
    Connector<ConnectorType::CAN> connector("can0");
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame> crn(connector);
    ConnectorSendNode<ConnectorType::CAN, CanFrame> cs(connector);
    constexpr motor::MotorId motor_id = 1;
    Motor<MotorType::DJI_6020> motor(&crn, motor_id);
    PidController pid_controller(10.0 * 3.0 / 16384.0, 0, 0, 0, 3, 0);
    auto pub = nh.advertise<MotorFdb>("/motor6020_fdb", 10);
    auto l = [&pub, &motor, &cs, &pid_controller](const CanFrame::MSGT& msg) {
        pub.publish(motor.get_fdb());
        pid_controller.fdb() = motor.get_fdb().pos_zero_cross.deg.num;
        pid_controller.ref() = 0;
        pid_controller.update();
        
        CanFrame::MSGT id_pack;
        id_pack.data.resize(8);
        // std::cout << pid_controller.out() << std::endl;
        id_pack.id = motor.set_send_buf(pid_controller.out(), id_pack.data);
        cs.send(id_pack);
    };
    motor.register_callback(l);

    ros::Timer timer = nh.createTimer(ros::Duration(1.0), 
        [&motor](const ros::TimerEvent&) {
            // std::cout << "timer callback" << std::endl;
            std::cout << "motor fps: " << motor.get_framerate() << std::endl;
        }
    );
    ros::spin();
    return 0;
}
