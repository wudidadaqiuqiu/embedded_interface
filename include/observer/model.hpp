#pragma once
#include <Eigen/Dense>
#include <iostream>
#include "common/common_math.hpp"

namespace observer {
using connector_common::real;
template <std::size_t XNm, std::size_t UNm, std::size_t ZNm>
struct StateSpaceModel {
    static constexpr std::size_t XNUM = XNm;
    static constexpr std::size_t UNUM = UNm;
    static constexpr std::size_t ZNUM = ZNm;

    Eigen::Matrix<real, XNUM, XNUM> A;
    Eigen::Matrix<real, XNUM, UNUM> B;
    Eigen::Matrix<real, ZNUM, XNUM> H;

    StateSpaceModel() = default;
    auto operator=(const StateSpaceModel<XNm, UNm, ZNm>& other) -> auto& {
        A = other.A;
        B = other.B;
        H = other.H;
        return *this;
    }
};

// template <unsigned int XNUM, unsigned int ZNUM>
// class StateSpaceModel<XNUM, 0, ZNUM> {
//     Eigen::Matrix<real, XNUM, XNUM> A;
//     Eigen::Matrix<real, ZNUM, XNUM> H;
// };
}