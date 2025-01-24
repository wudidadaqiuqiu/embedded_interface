#pragma once
#include <Eigen/Dense>
#include "common/common_math.hpp"

namespace observer {
using connector_common::real;
template <std::size_t XNUM, std::size_t UNUM, std::size_t ZNUM>
struct StateSpaceModel {
    Eigen::Matrix<real, XNUM, XNUM> A;
    Eigen::Matrix<real, XNUM, UNUM> B;
    Eigen::Matrix<real, ZNUM, XNUM> C;
    
};

// template <unsigned int XNUM, unsigned int ZNUM>
// class StateSpaceModel<XNUM, 0, ZNUM> {
//     Eigen::Matrix<real, XNUM, XNUM> A;
//     Eigen::Matrix<real, ZNUM, XNUM> H;
// };
}