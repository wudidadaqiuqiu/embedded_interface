#pragma once

#include "controller/controller_def.hpp"

namespace controller {


class PidController : public ControllerBase<real, real, real>{
public:
	real kp_, ki_, kd_;
	real outmax_;
	real error_max_;
	real Irange_;
	real error[3] = {0, 0, 0};
	real derror = 0;
	real error_sum = 0;
	PidController(real kp, real ki, real kd, real error_max, real outmax, real Irange) : 
		kp_(kp), ki_(ki), kd_(kd), outmax_(outmax), error_max_(error_max), Irange_(Irange) {}
	void update() override {
		error[2] = error[1];        // 上上次误差
		error[1] = error[0];        // 上次误差
		error[0] = ref_ - fdb_;  // 本次误差

		if (abs(error[0] - error[1]) > 1e-6 || abs(derror) < (1e-3)) {
            derror = error[0] - error[1];
        }
		if (abs(error[0]) < error_max_) {
		    error_sum += error[0];
			error_sum = get_mid(error_sum, -error_max_, error_max_);
		} else {
			error_sum = 0;
		}
		out_ = kp_ * error[0] + ki_ * error_sum + kd_ * derror;
		out_ = get_mid(out_, -outmax_, outmax_);
	}
};


}