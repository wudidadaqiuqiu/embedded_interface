#pragma once
#include <Eigen/Dense>
#include <iostream>
#include "common/common_math.hpp"
#include "common/common_macro_dependencies.hpp"
#include "observer/function_def.hpp"
namespace observer {
using connector_common::real;
using connector_common::concat;
using connector_common::get_pair_impl_t;

template <std::size_t XNm, std::size_t UNm, std::size_t ZNm>
struct StateSpaceModel {
    static constexpr std::size_t XNUM = XNm;
    static constexpr std::size_t UNUM = UNm;
    static constexpr std::size_t ZNUM = ZNm;

    struct Config {
        real temp1;
        real temp2;

        using ParamsT = std::tuple<
            typename tuple_convert<decltype(temp1), BasicType::type<decltype(temp1)>() == BasicType::Type::VOID>::type,
            typename tuple_convert<decltype(temp2), BasicType::type<decltype(temp2)>() == BasicType::Type::VOID>::type>;

        template <std::size_t Index>
        constexpr auto get_pair(auto const& prefix) {
            static_assert(Index < 2, "Index out of range");
            if constexpr(Index == 0) {
                return get_pair_impl_t<Index - 0, BasicType::type<decltype(temp1)>() == BasicType::Type::VOID>::
                    get_pair_impl(prefix, temp1, "temp1");
            }
            if constexpr(Index == 1) {
                return get_pair_impl_t<Index - 1, BasicType::type<decltype(temp2)>() == BasicType::Type::VOID>::
                    get_pair_impl(prefix, temp2, "temp2");
            }
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