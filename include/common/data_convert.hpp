#pragma once
namespace connector_common {
template <typename WrapT, typename WrapU, typename T, typename U>
inline void data_convert(const T& t, U& u);

struct UsefulNum {
    constexpr static real PI = 3.1415926f;
    constexpr static real RAD2DEG = 180.0f / PI;
    constexpr static real DEG2RAD = PI / 180.0f;
};


template <typename T>
T get_mid(T x, T a, T b) {
    T l = std::min(a, b);
    T u = std::max(a, b);
    if (x < l) {
        return l;
    }
    if (x > u) {
        return u;
    }
    return x;
}

}