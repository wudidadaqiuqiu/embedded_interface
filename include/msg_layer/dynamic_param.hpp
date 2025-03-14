#pragma once
#include "robot_msg/msg/pid_param.h"
#include "robot_msg/msg/pid_param.hpp"
#include "robot_msg/msg/lqr_param.h"
#include "robot_msg/msg/lqr_param.hpp"
#include "common/data_convert.hpp"

namespace con_used_msg {
    using PidParam = robot_msg::msg::PidParam;
    using PidParamStruct = robot_msg__msg__PidParam;
    using LqrParam = robot_msg::msg::LqrParam;
    using LqrParamStruct = robot_msg__msg__LqrParam;
}

