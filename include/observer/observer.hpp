#pragma once


#include "observer/diff_observer.hpp"
#include "observer/kf_observer.hpp"

namespace observer {

enum ObserverType {
    TD,
    KF
};

template <ObserverType ObserverTypeT>
struct Observer {
    template<typename... Args>
    using Type = void;
    template<typename... Args>
    using Config = void;
};

template <>
struct Observer<TD> {
    template<typename... Args>
    using Type = TdObserver<2, 0, 1>;
    template<typename... Args>
    using Config = Type<>::Config;
};

template <>
struct Observer<KF> {
    template<typename StateSpaceModelT>
    using Type = KalmanFilter<StateSpaceModelT>;
    template<typename StateSpaceModelT>
    using Config = Type<StateSpaceModelT>::Config;
};

}