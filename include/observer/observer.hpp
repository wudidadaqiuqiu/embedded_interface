#pragma once


#include "observer/diff_observer.hpp"
#include "observer/kf_observer.hpp"

namespace observer {

enum ObserverType {
    TD,
    KF
};

template <ObserverType ObserverTypeT>
struct Observer {};

template <>
struct Observer<TD> {
    template<typename... Args>
    using Type = TdObserver<2, 0, 1>;
    template<typename... Args>
    using Config = Type<>::Config;
};

template <>
class Observer<KF> {
    template<std::size_t XNUM, std::size_t UNUM, std::size_t ZNUM>
    using Type = KalmanFilter<XNUM, UNUM, ZNUM>;
};

}