#pragma once
#include "common/common_math.hpp"
#include "common/data_convert.hpp"

namespace observer {
using connector_common::real;

struct TdObserver {
    real r;
    real h;
    real h0;
    // real ref;
    real v1;
    real v2;

    TdObserver(real r, real h, real h0) : r(r), h(h), h0(h0) {}
    void update(real ref) {
        real last_v1, last_v2;
        last_v1 = v1;
        last_v2 = v2;
        v1 = last_v1 + h * last_v2;
        v2 = last_v2 + h * connector_common::fst(last_v1 - ref, last_v2, r, h0);
    }
};
}