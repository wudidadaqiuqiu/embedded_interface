#pragma once

#include "motor/motor.hpp"

namespace motor {

template <MotorType MotorT, typename ControllerT>
class MotorNode {
    Motor<MotorT> motor;
    ControllerT controller;
public:
    MotorNode(const MotorConfig<MotorT>& config) : motor(config) {}
};

}