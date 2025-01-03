#pragma once
#include "robot_msg/AngleRelate.h"

namespace connector_common {
template <typename WrapT, typename WrapU, typename T, typename U>
inline void data_convert(const T& t, U& u);

using real = float;
using Deg = real;
using Rad = real;

struct UsefulNum {
    constexpr static real PI = 3.1415926f;
    constexpr static real RAD2DEG = 180.0f / PI;
    constexpr static real DEG2RAD = PI / 180.0f;
};

template <>
inline void data_convert<Deg, robot_msg::AngleRelate>
    (const real& deg, robot_msg::AngleRelate& t) {
    t.deg.num = deg;
    t.rad.num = deg * UsefulNum::DEG2RAD;
}

}