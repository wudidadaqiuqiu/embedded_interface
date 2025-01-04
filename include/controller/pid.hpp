#pragma once

#include "controller/controller_def.hpp"

namespace controller {

template <>
struct ControllerConfig<ControllerType::PID, ControllerBaseConfigNone> {
	real kp;
	real ki;
	real kd;
	real error_max;
	real Irange;
	real outmax;
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
	Controller<ControllerConfig<ControllerType::PID, ControllerBaseConfigNone>>
		(const ControllerConfig<ControllerType::PID, ControllerBaseConfigNone>& config_) : 
		config(config_) {}
	void update() override {
		error[2] = error[1];        // 上上次误差
		error[1] = error[0];        // 上次误差
		error[0] = ref_ - fdb_;  // 本次误差

		if (abs(error[0] - error[1]) > 1e-6 || abs(derror) < (1e-3)) {
            derror = error[0] - error[1];
        }
		if (abs(error[0]) <  config.error_max) {
		    error_sum += error[0];
			error_sum = get_mid(error_sum, -config.error_max, config.error_max);
		} else {
			error_sum = 0;
		}
		out_ = config.kp * error[0] + config.ki * error_sum + config.kd * derror;
		out_ = get_mid(out_, -config.outmax, config.outmax);
	}
};


}