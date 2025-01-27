#pragma once
#include <Eigen/Dense>
#include "common/common_math.hpp"
#include "common/param_interface.hpp"
namespace observer {
using connector_common::real;
using connector_common::ParamsInterface;

template <std::size_t XNm, std::size_t UNm, std::size_t ZNm>
struct StateSpaceModel {
    static constexpr std::size_t XNUM = XNm;
    static constexpr std::size_t UNUM = UNm;
    static constexpr std::size_t ZNUM = ZNm;

    struct Config {
        real temp1;
        real temp2;

        constexpr auto param_interface() {
            return ParamsInterface(temp1, temp2, "temp1", "temp2");
        }

        template <std::size_t Index>
        void set(const auto& value) {
            param_interface().template set<Index>(value);
        }

    } config;
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