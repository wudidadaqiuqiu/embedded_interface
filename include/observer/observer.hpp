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
    template<std::size_t XNUM, std::size_t UNUM, std::size_t ZNUM>
    using Type = TdObserver<XNUM, UNUM, ZNUM>;
    template<std::size_t XNUM, std::size_t UNUM, std::size_t ZNUM>
    using Config = Type<XNUM, UNUM, ZNUM>::Config;
};

template <>
class Observer<KF> {
    template<std::size_t XNUM, std::size_t UNUM, std::size_t ZNUM>
    using Type = KalmanFilter<XNUM, UNUM, ZNUM>;
};

}