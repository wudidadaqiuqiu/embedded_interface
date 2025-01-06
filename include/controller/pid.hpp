#pragma once

#include "controller/controller_def.hpp"
#include "msg_layer/msg_layer.hpp"
namespace controller {
using connector_common::data_convert;
template <>
struct ControllerConfig<ControllerType::PID, ControllerBaseConfigNone> {
	using ConstructT = con_used_msg::PidParamStruct; 
	con_used_msg::PidParam param;
	decltype(param.kp.num)& kp = param.kp.num;
	decltype(param.ki.num)& ki = param.ki.num;
	decltype(param.kd.num)& kd = param.kd.num;
	decltype(param.error_max.num)& error_max = param.error_max.num;
	decltype(param.irange.num)& irange = param.irange.num;
	decltype(param.outmax.num)& outmax = param.outmax.num;
	ControllerConfig(const ConstructT& param_struct) {
		data_convert<ConstructT, decltype(param)>(param_struct, param);
	}
	// sizeof(param) + 6 * sizeof(void*)
};

template <>
class Controller<ControllerConfig<ControllerType::PID, ControllerBaseConfigNone>> :
	public ControllerBase<ControllerBaseConfigAllReal> {
public:
	// using THIS = Controller<ControllerType::PID, ControllerBaseConfigAllReal>;
	ControllerConfig<ControllerType::PID, ControllerBaseConfigNone> config;
	real error[3] = {0, 0, 0};
	real derror = 0;
	real error_sum = 0;
	Controller(const ControllerConfig<ControllerType::PID, 
		ControllerBaseConfigNone>::ConstructT& config_) : 
		config(config_) {}
	void update() override {
		error[2] = error[1];        // 上上次误差
		error[1] = error[0];        // 上次误差
		error[0] = ref - fdb;  // 本次误差

		if (abs(error[0] - error[1]) > 1e-6 || abs(derror) < (1e-3)) {
            derror = error[0] - error[1];
        }
		if (abs(error[0]) <  config.error_max) {
		    error_sum += error[0];
			error_sum = get_mid(error_sum, -config.error_max, config.error_max);
		} else {
			error_sum = 0;
		}
		out = config.kp * error[0] + config.ki * error_sum + config.kd * derror;
		out = get_mid(out, -config.outmax, config.outmax);
	}
};


}