#pragma once
#include "robot_msg/msg/angle_relate.hpp"
#include "robot_msg/msg/motor_fdb.hpp"
#include "robot_msg/msg/id_pack.hpp"
#include "robot_msg/msg/num_real.hpp"
#include "robot_msg/msg/num_real.h"

#include "common/data_convert.hpp"

namespace con_used_msg {
    using AngleRelate = robot_msg::msg::AngleRelate;
    using MotorFdb = robot_msg::msg::MotorFdb;
    using IdPack = robot_msg::msg::IdPack;
    using NumReal = robot_msg::msg::NumReal;
    using NumRealStruct = robot_msg__msg__NumReal;
}