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
    using Type = TdObserver;
};

template <>
class Observer<KF> {
    template<size_t XNUM, size_t UNUM, size_t ZNUM>
    using Type = KalmanFilter<XNUM, UNUM, ZNUM>;
};

}