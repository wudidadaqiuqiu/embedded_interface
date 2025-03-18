#pragma once

#include <Eigen/Dense>
#include "controller/controller_def.hpp"
#include "msg_layer/msg_layer.hpp"
#include "common/debug/log.hpp"
#include "common/common_math.hpp"
#include "common/param_interface.hpp"

namespace controller {
using connector_common::data_convert;
using connector_common::real;
using connector_common::ParamsInterface;

class LqrController : public ControllerData<Eigen::Vector<real, 2>, Eigen::Vector<real, 2>, real> {
   public:
    struct Config {
        using ConstructT = con_used_msg::LqrParamStruct;
        con_used_msg::LqrParam param;
        decltype(param.kp.num)& kp = param.kp.num;
        decltype(param.kd.num)& kd = param.kd.num;
        decltype(param.outmax.num)& outmax = param.outmax.num;
        constexpr auto param_interface() {
            return ParamsInterface(kp, kd, outmax, "kp", "kd", "outmax");
        }
        template <std::size_t Index>
        void set(const auto& value) {
            param_interface().template set<Index>(value);
        }
        Config(const ConstructT& param_struct) {
            data_convert(param_struct, param);
        }
        Config() = default;
        // opetator=
        auto& operator=(const Config& config){
            this->param = config.param;
            return *this;
        }
    };
    Config config;
    LqrController(const Config::ConstructT& config_) : config(config_) {}
    LqrController() = default;
    Eigen::Vector<real, 2> error;

    void update() {
        error = ref - fdb;
        // LOG_INFO(1, "error: %f, %f", error(0), error(1));
        out = config.kp * error(0) + config.kd * error(1);
        out = get_mid(out, -config.outmax, config.outmax);
        // LOG_INFO(1, "error: %f, %f, %f", error(0), error(1), out);
    }
};

class LqriController : public ControllerData<Eigen::Vector<real, 2>, Eigen::Vector<real, 2>, real> {
   public:
    struct Config {
        using ConstructT = con_used_msg::PidParamStruct;
		con_used_msg::PidParam param;
		decltype(param.kp.num)& kp = param.kp.num;
		decltype(param.ki.num)& ki = param.ki.num;
		decltype(param.kd.num)& kd = param.kd.num;
		decltype(param.error_max.num)& error_max = param.error_max.num;
		decltype(param.irange.num)& irange = param.irange.num;
		decltype(param.outmax.num)& outmax = param.outmax.num;

        constexpr auto param_interface() {
            return ParamsInterface(kp, ki, kd, error_max, irange, outmax, "kp", "ki", "kd", "error_max", "irange", "outmax");
        }

        template <std::size_t Index>
        void set(const auto& value) {
            param_interface().template set<Index>(value);
        }
        Config(const ConstructT& param_struct) {
            data_convert(param_struct, param);
        }
        Config() = default;
        // opetator=
        auto& operator=(const Config& config){
            this->param = config.param;
            return *this;
        }
    };
    Config config;

    LqriController(const Config::ConstructT& config_) : config(config_) {}
    LqriController() = default;
    Eigen::Vector<real, 2> error;
    real error_sum = 0;

    void update() {
        error = ref - fdb;
        // LOG_INFO(1, "error: %f, %f", error(0), error(1));
        if (abs(error(0)) <  config.error_max) {
		    error_sum += error(0);
			error_sum = get_mid(error_sum, -config.error_max, config.error_max);
		} else {
			error_sum = 0;
		}
        out = config.kp * error(0) + config.kd * error(1) + config.ki * error_sum;
        out = get_mid(out, -config.outmax, config.outmax);
        // LOG_INFO(1, "error: %f, %f, %f", error(0), error(1), out);
    }
};

}  // namespace controller