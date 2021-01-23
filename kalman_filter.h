//
// Created by matt on 22/01/2021.
//

#ifndef SIMPLE_UKF_KALMAN_FILTER_H
#define SIMPLE_UKF_KALMAN_FILTER_H

#include "state.h"

#include <functional>

#include <Eigen/Core>
#include <Eigen/LU>

namespace kf1 {

// TODO alternative to std::function??
// I'd like to capture the process/measurement model and associated noise in a single 'system'
// And ideally just template on this
// If I store the process/measurement models, I can precalculate the equivalent matrices
// TODO add nicer way to get all matrices


template<size_t N>
using ProcessModel = std::function<Eigen::Matrix<double, N, 1>(const Eigen::Matrix<double, N, 1>&)>;

template<size_t N, size_t M>
using MeasurementModel = std::function<Eigen::Matrix<double, M, 1>(const Eigen::Matrix<double, N, 1>&)>;


template<size_t N>
class KalmanFilter {
  public:
    KalmanFilter() {};
    void Predict(ProcessModel<N> process, const Eigen::Matrix<double, N, N>& process_noise) {
        const Eigen::Matrix<double, N, N>& Q = process_noise;
//        std::cout << "Predict\n";
//        state_.x = process(state_.x);
        // How to update covariance?
        // Easy if process were passed as a matrix
        // P_ = F_ * P_ * F_.transpose() + Q_;

        // Appy process model
//        Eigen::Matrix<double, N, N> FP;
//        for (size_t col_idx = 0; col_idx < N; ++col_idx) {
//            FP.col(col_idx) = process(state_.P.col(col_idx));
//        }
//        Eigen::Matrix<double, N, N> FPF;
//        for (size_t row_idx = 0; row_idx < N; ++row_idx) {
//            FPF.row(row_idx) = process(FP.row(row_idx));
//        }

        // Or extract F matrix from the function
        Eigen::Matrix<double, N, N> I = Eigen::Matrix<double, N, N>::Identity();
        Eigen::Matrix<double, N, N> F;
        for (size_t col_idx = 0; col_idx < N; ++col_idx) {
            F.col(col_idx) = process(I.col(col_idx));
        }
//        std::cout << F << '\n';
        state_.x = F * state_.x;
        state_.P = F * state_.P * F.transpose() + Q;
//        std::cout << "x\n";
//        std::cout << state_.x.transpose() << "\n\n";
//        std::cout << "P\n";
//        std::cout << state_.P << "\n\n";

    };
    template<size_t M>
    void Update(MeasurementModel<N, M> measurement_model, const Eigen::Matrix<double, M, 1>& measurement,
                const Eigen::Matrix<double, M, M>& measurement_noise) {
        const Eigen::Matrix<double, M, M>& R = measurement_noise;

        Eigen::Matrix<double, N, N> I = Eigen::Matrix<double, N, N>::Identity();
        Eigen::Matrix<double, M, N> H; // measurement model as matrix
        for (size_t col_idx = 0; col_idx < N; ++col_idx) {
            H.col(col_idx) = measurement_model(I.col(col_idx));
        }
//        std::cout << "Update\n";

       Eigen::Matrix<double, M, 1> z_hat = H * state_.x;
        Eigen::Matrix<double, M, 1> y = measurement - z_hat;
        Eigen::Matrix<double, M, M> S = H * state_.P * H.transpose() + R;
        Eigen::Matrix<double, N, M> K = state_.P * H.transpose() * S.inverse();

        state_.x = state_.x + K * y;
//        std::cout << "K\n";
//        std::cout << K << "\n\n";
//        std::cout << "H\n";
//        std::cout << H << "\n\n";
//        std::cout << "K * H\n";
//        std::cout << K * H << "\n\n";
//        std::cout << "I - K * H\n";
//        std::cout << Eigen::Matrix<double, N, N>::Identity() - K * H << "\n\n";
        state_.P = (Eigen::Matrix<double, N, N>::Identity() - K * H) * state_.P;

//        std::cout << "z_hat\n";
//        std::cout << z_hat.transpose() << "\n\n";
//        std::cout << "y\n";
//        std::cout << y.transpose() << "\n\n";
//        std::cout << "S\n";
//        std::cout << S << "\n\n";
//        std::cout << "K\n";
//        std::cout << K << "\n\n";


//        std::cout << "x\n";
//        std::cout << state_.x.transpose() << "\n\n";
//        std::cout << "P\n";
//        std::cout << state_.P << "\n\n";
    };

    Eigen::Matrix<double, N, 1> GetState() const {
        return state_.x;
    };
    Eigen::Matrix<double, N, N> GetCov() const {
        return state_.P;
    };
    void SetState(const Eigen::Matrix<double, N, 1>& new_state) {
        state_.x = new_state;
    };
    void SetCov(const Eigen::Matrix<double, N, N>& new_cov) {
        state_.P = new_cov;
    };

  private:
    kf::State<N> state_;
};

} // namespace kf1


#endif //SIMPLE_UKF_KALMAN_FILTER_H
