#pragma once
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

template <uint8_t x_nums, uint8_t u_nums, uint8_t z_nums>
class DiscreteStateSpaceModel {
   private:

   public:
    using AmatT = Matrix<float, x_nums, x_nums>;
    using BmatT = Matrix<float, x_nums, u_nums>;
    using CmatT = Matrix<float, z_nums, x_nums>;
    const float t_step;
    Eigen::Matrix<float, x_nums, x_nums> A_mat;
    Eigen::Matrix<float, x_nums, u_nums> B_mat;
    Eigen::Matrix<float, z_nums, x_nums> C_mat;
    DiscreteStateSpaceModel(float t_step_)
        : t_step(t_step_) {
        if (t_step <= 0) throw -1;
    }
    void ChangeModel(Eigen::Matrix<float, x_nums, x_nums>& A,
                    Eigen::Matrix<float, x_nums, u_nums>& B, 
                    Eigen::Matrix<float, z_nums, x_nums>& C) {
        A_mat.noalias() = A;
        B_mat.noalias() = B;
        C_mat.noalias() = C;
    }
};

template <int X, int U, int Z, int P, int C>
class ModelPredictController {
   private:
    const DiscreteStateSpaceModel<X, U, Z>& model;
    Matrix<float, C * U, C * U> W1;
    Matrix<float, C * U, C * U> W2;
    Matrix<float, C * U, C * U> W3;
    Matrix<float, P * Z, P * Z> W4;
    Matrix<float, P * Z, X> O;
    Matrix<float, P * Z, C * U> M;
    Matrix<float, C * U, P * Z> gain;
    Matrix<float, U, 1> res;
    Matrix<float, P * Z, 1> s;
    Matrix<float, C * U, 1> u;
    Matrix<float, P * Z, 1> zd;

   public:
    auto& get_gain() {
        return gain;
    }
    using QarrT = std::array<Matrix<float, U, U>, C>;
    using ParrT = std::array<Matrix<float, Z, Z>, P>;
    using X0T = Matrix<float, X, 1>;
    using ZrefarrT = std::array<Matrix<float, Z, 1>, P>;
    ModelPredictController(const DiscreteStateSpaceModel<X, U, Z>& model_) : model(model_) {};
    void ChanegeParams(std::array<Matrix<float, U, U>, C>& Q_arr, std::array<Matrix<float, Z, Z>, P>& P_arr) {
        Matrix<float, U, U> Im;

        Im = Matrix<float, U, U>::Identity();
        W1.setZero();
        W2.setZero();
        W4.setZero();

        for (int i = 0; i < C; i++) {
            if (i == 0) {
                W1.block(i * U, i * U, U, U) = Im;
            } else {
                W1.block(i * U, i * U, U, U) = Im;
                W1.block(i * U, (i - 1) * U, U, U) = -Im;
            }
        }
        
        // std::cout << "W1: \n" << W1 << std::endl;

        for (int i = 0; i < C; i++) {
            W2.block(i * U, i * U, U, U) = Q_arr[i];
        }

        // std::cout << "W2: \n" << W2 << std::endl;
        W3 = W1.transpose() * W2 * W1;

        // std::cout << "W3: \n" << W3 << std::endl;

        for (int i = 0; i < P; i++) {
            W4.block(i * Z, i * Z, Z, Z) = P_arr[i];
        }

        // std::cout << "W4: \n" << W4 << std::endl;

        Matrix<float, X, X> powA;
        powA = Matrix<float, X, X>::Identity();
        O.setZero();
        for (int i = 0; i < P; i++) {
            powA = powA * model.A_mat;
            O.block(i * Z, 0, Z, X) = model.C_mat * powA;
        }

        // std::cout << "O: \n" << O << std::endl;

        Matrix<float, X, X> sumLast;
        sumLast.setZero();
        M.setZero();

        for (int i = 0; i < C; i++) {
            powA = Matrix<float, X, X>::Identity();
            for (int j = 0; j < i + 1; j++) {
                M.block(i * Z, (i - j) * U, Z, U) = model.C_mat * powA * model.B_mat;
                powA = powA * model.A_mat;
            }
        }
        for (int i = C; i < P; i++) {
            for (int j = 0; j < C; j++) {
                if (j == 0) {
                    sumLast.setZero();
                    powA = Matrix<float, X, X>::Identity();
                    for (int s = 0; s < i + 2 - C; s++) {
                        sumLast = sumLast + powA;
                        powA = powA * model.A_mat;
                    }
                    M.block(i * Z, (C - 1) * U, Z, U) = model.C_mat * sumLast * model.B_mat;
                } else {
                    M.block(i * Z, (C - 1 - j) * U, Z, U) = model.C_mat * powA * model.B_mat;
                    powA = powA * model.A_mat;
                }
            }
        }

        // std::cout << "M: \n" << M << std::endl;
        gain = (M.transpose() * W4 * M + W3).inverse() * M.transpose() * W4;
    }

    Matrix<float, U, 1>& Calc(Matrix<float, X, 1>& x0, std::array<Matrix<float, Z, 1>, P>& z_ref_arr) {
        for (size_t i = 0; i < P; ++i) {
            zd.block(i * Z, 0, Z, 1) = z_ref_arr[i];
        }
        // std::cout << "zd: \n" << zd << std::endl;

        s = zd - O * x0;
        u = gain * s;
        res = u.block(0, 0, U, 1);
        // std::cout << "u: \n" << u << std::endl;
        return res;
    }
};