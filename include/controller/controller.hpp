#pragma once

#include "controller/controller_def.hpp"
#include "controller/pid.hpp"
#include "controller/lqr.hpp"

namespace controller {

template <ControllerType ControllerTypeT>
struct Controller {};

template <>
struct Controller<PID> {
    template<typename... _>
    using Type = PidController;
    using Config = Type<>::Config;
    using ConstructT = Type<>::Config::ConstructT;
};


template <>
struct Controller<LQR> {
    template<typename... _>
    using Type = LqrController;
    using Config = Type<>::Config;
    using ConstructT =Type<>::Config::ConstructT;
};

}