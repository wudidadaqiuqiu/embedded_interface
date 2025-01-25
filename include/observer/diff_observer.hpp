#pragma once
#include "common/common_math.hpp"
#include "common/data_convert.hpp"
#include "common/type_def.hpp"
#include "common/macro_def.h"
namespace observer {
using connector_common::real;
using connector_common::ConstexprStringMap;
using connector_common::BasicType;

template <std::size_t XNUM, std::size_t UNUM, std::size_t ZNUM>
struct TdObserver {
    static_assert(XNUM == 2, "TD observer only support 2 state");
    static_assert(UNUM == 0, "TD observer don't have inputs");
    static_assert(ZNUM == 1, "TD observer only support 1 measurement");
    struct Config {
        real r;
        real h;
        real h0;
        DECLARE_PARAM_MAP_DATA(r, h, h0)
        DECLARE_SET_FUNCTION(r, h, h0)
        Config() = default;
    } config;
    struct UpdateData {
        real ref;
    };
    struct PredictData {};
    struct StateData {
        real v1;
        real v2;
    } state = {0, 0};
    
    TdObserver() = default;
    TdObserver(const Config &c) : config(c) {}
    void update(const UpdateData &data) {
        real last_v1, last_v2;
        last_v1 = state.v1;
        last_v2 = state.v2;
        state.v1 = last_v1 + config.h * last_v2;
        state.v2 = last_v2 + config.h * connector_common::fst(
            last_v1 - data.ref, last_v2, config.r, config.h0);
    }

    void predict(const PredictData &data) { (void) data; }
    auto get_state() const -> const auto& { return state; }
};
}