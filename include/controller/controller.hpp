#pragma once

#include "controller/controller_def.hpp"
#include "controller/pid.hpp"
#include "controller/lqr.hpp"

namespace controller {

template <ControllerType ControllerTypeT>
struct Controller {
    template<typename... _>
    using Type = void;
    template<typename... _>
    using Config = void;
    using ConstructT = void;
};

template <>
struct Controller<PID> {
    template<typename... _>
    using Type = PidController;
    template<typename... _>
    using Config = Type<_...>::Config;
    template<typename... _>
    using ConstructT = Type<_...>::Config::ConstructT;
};


template <>
struct Controller<LQR> {
    template<typename... _>
    using Type = LqrController;
    template<typename... _>
    using Config = Type<_...>::Config;
    template<typename... _>
    using ConstructT =Type<_...>::Config::ConstructT;
};

template <>
struct Controller<LQRI> {
    template<typename... _>
    using Type = LqriController;
    template<typename... _>
    using Config = Type<_...>::Config;
    template<typename... _>
    using ConstructT =Type<_...>::Config::ConstructT;
};

}