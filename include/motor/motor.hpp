#pragma once
#include "data_convert.hpp"
#include "robot_msg/MotorFdb.h"
#include "connector/msgpack.hpp"
#include "connector/connector_node.hpp"

using connector::CanFrame;
using connector::ConnectorType;
using connector::ConnectorSingleRecvNode;
using robot_msg::MotorFdb;

namespace motor {

enum MotorType {
    DJI_6020 = 0,
};

using connector_inner::real;
using MotorId = uint32_t;

template <MotorType MotorT>
class Motor { };

}