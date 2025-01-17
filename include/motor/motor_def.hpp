#pragma once
#include "common/data_convert.hpp"
#include "msg_layer/msg.hpp"
#include "connector/msgpack.hpp"
#include "connector/connector_node.hpp"

using connector::CanFrame;
using connector::ConnectorType;
using connector::ConnectorSingleRecvNode;
using con_used_msg::MotorFdb;

namespace motor {

enum MotorType {
    DJI_6020,
    DJI_3508
};

using connector_common::data_convert;
using connector_common::real;
using connector_common::UsefulNum;

using MotorId = uint32_t;

template <MotorType MotorT>
class Motor { };

template <MotorType MotorT>
struct MotorConfig { };


}