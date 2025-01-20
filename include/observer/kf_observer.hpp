#pragma once
#include <Eigen/Dense>
#include "observer/model.hpp"
#include "common/debug/log.hpp"
namespace observer {
template <size_t xnum, size_t unum, size_t znum>
class KalmanFilter {
   public:
    KalmanFilter(const StateSpaceModel<xnum, unum, znum>& model) : model(model) {
        I.setIdentity();
    }
    void update() {
        try {
            K = P * model.C.transpose() * (model.C * P * model.C.transpose() + R).inverse();
            x = xpre + K * (z - model.C * xpre);
            P = (I - K * model.C) * P;

            xpre = model.A * x + model.B * u;
            P = model.A * P * model.A.transpose() + Q;
        } catch (const std::exception& e) {
            LOG_ERROR(1, "%s", e.what());
        }
    }

    void pre() {
        xpre = model.A * x + model.B * u;
        P = model.A * P * model.A.transpose() + Q;
    }
    
    void post() {
        try {
            K = P * model.C.transpose() * (model.C * P * model.C.transpose() + R).inverse();
            x = xpre + K * (z - model.C * xpre);
            P = (I - K * model.C) * P;
        }
        catch(const std::exception& e) {
            LOG_ERROR(1, "%s", e.what());
        }
    }
   protected:
    StateSpaceModel<xnum, unum, znum> model;
    Eigen::Matrix<real, xnum, xnum> P;
    Eigen::Matrix<real, xnum, xnum> Q;
    Eigen::Matrix<real, znum, znum> R;
    Eigen::Matrix<real, xnum, znum> K;
    Eigen::Vector<real, xnum> xpre;
    Eigen::Vector<real, xnum> x;
    Eigen::Vector<real, unum> u;
    Eigen::Vector<real, znum> z;

    Eigen::Matrix<real, xnum, xnum> I;
};
}