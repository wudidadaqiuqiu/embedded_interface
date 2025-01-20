#pragma once

#include "common/common_math.hpp"
#include "common/data_convert.hpp"

using connector_common::real;
using connector_common::get_mid;

namespace controller {

enum ControllerType {
    PID = 0,
    LQR = 1,
};

template <typename FdbTypeT, typename RefTypeT, typename OutTypeT>
struct ControllerData {
    FdbTypeT fdb;
    RefTypeT ref;
    OutTypeT out;
};

}