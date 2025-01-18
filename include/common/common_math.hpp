#pragma once
#include <cmath>

namespace connector_common {
using real = float;
struct Deg {
    real val;
};
struct Rad {
    real val;
};
// 传入参数为浮点数的符号函数
inline int fsgn(float x) {
// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wfloat-equal"
    return (x != 0.0f ? (x < 0.0f ? -1 : 1) : 0); 
// #pragma GCC diagnostic pop
}

inline int fsg(float x, float d) {
    return ((fsgn(x + d) - fsgn(x - d)) / 2);
}

inline float fst(float x1_delta, float x2, float r, float h0) {
    float d, y, a0, a, a1, a2;
    d = r * h0 * h0;
    a0 = h0 * x2;
    y = x1_delta + a0;
    a1 = sqrt(d * (d + 8 * fabs(y)));
    a2 = a0 + fsgn(y) * (a1 - d) / 2;
    a = (a0 + y) * fsg(y, d) + a2 * (1 - fsg(y, d));
    return (-r * (a / d) * fsg(a, d) - r * fsgn(a) * (1 - fsg(a, d)));
}

}