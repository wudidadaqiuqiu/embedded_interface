#pragma once

#include "Eigen/Dense"
#include "controller/controller_def.hpp"
#include "msg_layer/msg_layer.hpp"

namespace controller {
using connector_common::data_convert;
using connector_common::real;
template <size_t SIZE>
using VVrControllerConfig = ControllerBaseConfig<Eigen::Vector<real, SIZE>,
                                                 Eigen::Vector<real, SIZE>, real>;

template <>
struct ControllerConfig<ControllerType::LQR, VVrControllerConfig<2>> {
    using ConstructT = con_used_msg::LqrParamStruct;
    con_used_msg::LqrParam param;
    decltype(param.kp.num)& kp = param.kp.num;
    decltype(param.kd.num)& kd = param.kd.num;
    decltype(param.outmax.num)& outmax = param.outmax.num;
    ControllerConfig(const ConstructT& param_struct) {
        data_convert<ConstructT, decltype(param)>(param_struct, param);
    }
};

template <>
class Controller<ControllerConfig<ControllerType::LQR, VVrControllerConfig<2>>> : 
    public ControllerBase<VVrControllerConfig<2>> {
   public:
    // using THIS = Controller<ControllerType::LQR, ControllerBaseConfigAllReal>;
    ControllerConfig<ControllerType::LQR, VVrControllerConfig<2>> config;
    Controller(const decltype(config)::ConstructT& config_) : config(config_) {}
    Eigen::Vector<real, 2> error;
    
    void update() override {
        error = ref - fdb;
        out = config.kp * error(0) + config.kd * error(1);
        out = get_mid(out, -config.outmax, config.outmax);
    }
};

}  // namespace controller