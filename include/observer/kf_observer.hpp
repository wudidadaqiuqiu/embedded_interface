#pragma once
#include <Eigen/Dense>
#include "common/debug/log.hpp"

#include "common/param_interface.hpp"
#include "observer/model.hpp"

#ifndef KALMAN_FILTER_DEBUG
#define KALMAN_FILTER_DEBUG (1)
#endif
namespace observer {
using connector_common::to_string;
using connector_common::ParamsInterface;
using connector_common::BasicType;
using connector_common::assign_operate;
using connector_common::for_each_conditional_return;

template <std::size_t XNm, std::size_t UNm, std::size_t ZNm>
class KalmanFilter {
   public:
    struct Config {
        // 让 KalmanFilter能拿到private
        friend class KalmanFilter;
        private:
        std::vector<real> q_mat;
        std::vector<real> r_mat;
        public:
        StateSpaceModel<XNm, UNm, ZNm> model;
        // 方便调参
        Eigen::Matrix<real, XNm, XNm, Eigen::RowMajor> P;
        Eigen::Matrix<real, XNm, XNm, Eigen::RowMajor> Q;
        Eigen::Matrix<real, ZNm, ZNm, Eigen::RowMajor> R;
        
        // std::mutex mutex;
        // copy constructor 显式调用默认构造函数
        Config(const Config& other) : Config() {
            P = other.P;
            Q = other.Q;
            R = other.R;
            model = other.model;
        }
        Config() {
            q_mat.resize(Q.SizeAtCompileTime);
            r_mat.resize(R.SizeAtCompileTime);
        }
        constexpr auto param_interface() {
            return ParamsInterface(q_mat, model, r_mat, "q_mat", "model", "r_mat");
        }

        template <std::size_t Index>
        void set(const auto& value) {
            using ParamIntT = decltype(param_interface());
            param_interface().template set<Index>(value);
            constexpr auto COUNT = ParamIntT::template index_count<Index>();
            if constexpr (ParamIntT::template index_count<Index>() == 1) {
                auto index = ParamIntT::template get_ele_sub_index<Index, COUNT>();
                LOG_DEBUG(KALMAN_FILTER_DEBUG, "model index: %ld, %ld", Index, index);
                LOG_DEBUG(KALMAN_FILTER_DEBUG, "model: %lf, %lf", model.config.temp1, model.config.temp2);
            } else {
                // 要用else 括起来，否则会编译所有的情况
                auto& v = param_interface().template get_ele<COUNT>();
                vector_set(v, value);
            }
        }

        void vector_set(auto& v, const auto& value) {
            if (v.size() != value.size()) {
                // 逆天 string加法 std::format依赖太多了
                throw std::runtime_error("Size of q_mat " + std::to_string(q_mat.size()) + 
                                        " or r_mat " + std::to_string(r_mat.size()) + 
                                        " is not equal to Q " + std::to_string(Q.rows() * Q.cols()) +
                                        " or R " + std::to_string(R.rows() * R.cols()));
            }
            // v.resize(0);
            // std::transform(value.begin(), value.end(), std::back_inserter(v),
            //     [](decltype(*value.begin()) val) { return static_cast<real>(val); });
            // std::lock_guard<std::mutex> lock(mutex);
            LOG_DEBUG(KALMAN_FILTER_DEBUG, "set q and r value: %s", connector_common::to_string(value).c_str());
            LOG_DEBUG(KALMAN_FILTER_DEBUG, "be setted: %s", to_string(v).c_str());
            static_assert(Q.SizeAtCompileTime == Q.RowsAtCompileTime * Q.ColsAtCompileTime, "assert test failed");
            for (std::size_t i = 0; i < Q.SizeAtCompileTime; ++i) {
                Q.coeffRef(i) = q_mat[i];
            }
            for (std::size_t i = 0; i < R.SizeAtCompileTime; ++i) {
                R.coeffRef(i) = r_mat[i];
            }
            LOG_DEBUG(KALMAN_FILTER_DEBUG, "Kalman filter config updated: \nQ:\n%s\nR:\n%s", 
                to_string(Q).c_str(), to_string(R).c_str());
        }
	} config;
    struct UpdateData {
        std::array<real, ZNm> z;
    };
    struct PredictData {
        std::array<real, ZNm> u;
    };
    struct StateData {
        std::array<real, XNm> x;
    };
    KalmanFilter(const Config& config_) : config(config_) {
        I.setIdentity();
        LOG_DEBUG(KALMAN_FILTER_DEBUG, "Kalman filter initialized with config: \nP:\n%s\nQ:\n%s\nR:\n%s", 
            to_string(config.P).c_str(), to_string(config.Q).c_str(), to_string(config.R).c_str());
        LOG_DEBUG(KALMAN_FILTER_DEBUG, "Kalman filter initialized with model: \nA:\n%s\nB:\n%s\nH:\n%s",
            to_string(config.model.A).c_str(), to_string(config.model.B).c_str(), to_string(config.model.H).c_str());
        LOG_DEBUG(KALMAN_FILTER_DEBUG, "q_mat size: %ld, r_mat size: %ld", config.q_mat.size(), config.r_mat.size());

    }
    ~KalmanFilter() {
        LOG_DEBUG(KALMAN_FILTER_DEBUG, "Kalman filter destroyed");
    }
    void update(const UpdateData& data) {
        for (std::size_t i = 0; i < ZNm; ++i) {
            z[i] = data.z[i];
        }
        try {
            K = config.P * config.model.H.transpose() * (config.model.H * config.P * config.model.H.transpose() + config.R).inverse();
            x = xpre + K * (z - config.model.H * xpre);
            config.P = (I - K * config.model.H) * config.P;
        }
        catch(const std::exception& e) {
            LOG_ERROR(1, "%s", e.what());
        }
    }

    void predict(const PredictData& data) {
        for (std::size_t i = 0; i < UNm; ++i) {
            u[i] = data.u[i];
        }
        xpre = config.model.A * x + config.model.B * u;
        config.P = config.model.A * config.P * config.model.A.transpose() + config.Q;
    }
    
    auto get_state() const -> const auto& { 
        return state; 
    }
   protected:
    StateData state;
    Eigen::Matrix<real, XNm, ZNm> K;
    Eigen::Vector<real, XNm> xpre;
    Eigen::Map<Eigen::Vector<real, XNm>> x{state.x.data()};
    Eigen::Vector<real, UNm> u;
    Eigen::Vector<real, ZNm> z;

    Eigen::Matrix<real, XNm, XNm> I;
};
}