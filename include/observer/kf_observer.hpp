#pragma once
#include <Eigen/Dense>
#include "common/common_macro_dependencies.hpp"
#include "observer/model.hpp"
#include "common/debug/log.hpp"

#ifndef KALMAN_FILTER_DEBUG
#define KALMAN_FILTER_DEBUG (1)
#endif
namespace observer {

template <typename T>
inline auto to_string(const T& mat) -> std::string{
    std::stringstream ss;
    ss << mat;
    return ss.str();
}
template <std::size_t XNm, std::size_t UNm, std::size_t ZNm>
class KalmanFilter {
   public:
    struct Config {
        private:
        std::vector<real> q;
        std::vector<real> r;
        public:
        StateSpaceModel<XNm, UNm, ZNm> model;
        Eigen::Matrix<real, XNm, XNm, Eigen::RowMajor> P;
        Eigen::Matrix<real, XNm, XNm, Eigen::RowMajor> Q;
        Eigen::Matrix<real, ZNm, ZNm, Eigen::RowMajor> R;
        
        // std::mutex mutex;
        // copy constructor
        Config(const Config& other) {
            P = other.P;
            Q = other.Q;
            R = other.R;
            model = other.model;
        }
        Config() = default;
        DECLARE_PARAM_MAP_DATA(q, r)
        template <std::size_t Index>
        void set(const auto& value) {
            auto& v = tuple_get<Index>(q, r);
            v = value;
            // std::lock_guard<std::mutex> lock(mutex);
            if (q.size() != Q.rows() * Q.cols() || r.size() != R.rows() * R.cols()) {
                throw std::runtime_error("Size of q %d or r %d is not equal to Q %d or R %d", 
                    q.size(), r.size(), Q.rows() * Q.cols(), R.rows() * R.cols());
            }
            for (std::size_t i = 0; i < Q.size(); ++i) {
                Q.coeffRef(i) = q[i];
            }
            for (std::size_t i = 0; i < r.size(); ++i) {
                R.coeffRef(i) = r[i];
            }
            LOG_INFO(KALMAN_FILTER_DEBUG, "Kalman filter config updated: \nQ:\n%sR:\n%s", 
                to_string(Q).c_str(), to_string(R).c_str());
        }
	};
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
    Config config;
    StateData state;
    Eigen::Matrix<real, XNm, ZNm> K;
    Eigen::Vector<real, XNm> xpre;
    Eigen::Map<Eigen::Vector<real, XNm>> x{state.x.data()};
    Eigen::Vector<real, UNm> u;
    Eigen::Vector<real, ZNm> z;

    Eigen::Matrix<real, XNm, XNm> I;
};
}