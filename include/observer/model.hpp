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

        constexpr auto param_interface() {
            return ParamsInterface();
            // return ParamsInterface(temp1, temp2, "temp1", "temp2");
        }

        template <std::size_t Index>
        void set(const auto& value) {
            // param_interface().template set<Index>(value);
        }

    } config;
    Eigen::Matrix<real, XNUM, XNUM> A;
    Eigen::Matrix<real, XNUM, UNUM> B;
    Eigen::Matrix<real, ZNUM, XNUM> H;

    auto f(const auto& x, const auto& u) -> auto {
        return A * x + B * u;
    }
    auto h(const auto& x) -> auto {
        return H * x;
    }
    StateSpaceModel() = default;
    auto operator=(const StateSpaceModel<XNm, UNm, ZNm>& other) -> auto& {
        A = other.A;
        B = other.B;
        H = other.H;
        return *this;
    }
};

template <std::size_t XNm, std::size_t UNm, std::size_t ZNm>
struct VarientStateSpaceModel : StateSpaceModel<XNm, UNm, ZNm> {
    using ParentT = StateSpaceModel<XNm, UNm, ZNm>;
    using ParentT::A;
    using ParentT::B;
    using ParentT::H;
    static constexpr std::size_t XNUM = XNm;
    static constexpr std::size_t UNUM = UNm;
    static constexpr std::size_t ZNUM = ZNm;

    struct Config : StateSpaceModel<XNm, UNm, ZNm>::Config {
        std::function<void(const Eigen::Vector<real, XNm>&, const Eigen::Vector<real, UNm>&, Eigen::Vector<real, XNm>&)> f;
        std::function<void(const Eigen::Vector<real, XNm>&, Eigen::Vector<real, ZNm>&)> h;
        std::function<void(const Eigen::Vector<real, XNm>&, const Eigen::Vector<real, UNm>&, 
            decltype(ParentT::A)& A)> jacobian_A_update_func;
        std::function<void(const Eigen::Vector<real, XNm>&, decltype(ParentT::H)& H)> jacobian_H_update_func;
    } config;

    auto get_H_update_func() -> decltype(config.jacobian_H_update_func)& {
        return config.jacobian_H_update_func;
    }

    auto get_A_update_func() -> decltype(config.jacobian_A_update_func)& {
        return config.jacobian_A_update_func;
    }

    auto f(const auto& x, const auto& u) -> Eigen::Vector<real, XNm> {
        if (!config.f) {
            return A * x + B * u;
            // throw std::runtime_error("f function not defined");
        }
        Eigen::Vector<real, XNm> xx;
        config.f(x, u, xx);
        return xx;
    }
    auto h(const auto& x) -> Eigen::Vector<real, ZNm> {
        if (!config.h) {
            return H * x;
            // throw std::runtime_error("h function not defined");
        }
        Eigen::Vector<real, ZNm> z;
        config.h(x, z);
        return z;
    }
};

// template <unsigned int XNUM, unsigned int ZNUM>
// class StateSpaceModel<XNUM, 0, ZNUM> {
//     Eigen::Matrix<real, XNUM, XNUM> A;
//     Eigen::Matrix<real, ZNUM, XNUM> H;
// };
}