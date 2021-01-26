//
// Created by matt on 24/01/2021.
//

#ifndef KALMAN_FILTERS_UNSCENTED_KF_H_
#define KALMAN_FILTERS_UNSCENTED_KF_H_

#include <functional>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Cholesky>

#include "state.h"
#include "system.h"

// References
// https://www3.nd.edu/~lemmon/courses/ee67033/pubs/julier-ukf-tac-2000.pdf
// https://www.youtube.com/watch?t=24m27s&v=DWDzmweTKsQ&feature=youtu.be
// https://towardsdatascience.com/the-unscented-kalman-filter-anything-ekf-can-do-i-can-do-it-better-ce7c773cf88d
// https://github.com/koide3/hdl_localization/blob/master/include/kkl/alg/unscented_kalman_filter.hpp



namespace experimental {
namespace ukf1 {

template<typename System>
class UnscentedKalmanFilter {
 public:
  using StateSizeVector = Eigen::Matrix<double, System::kStateSize, 1>;
  using StateSizeMatrix = Eigen::Matrix<double, System::kStateSize, System::kStateSize>;
  using MeasurementSizeVector = Eigen::Matrix<double, System::kMeasurementSize, 1>;
  using MeasurementSizeMatrix = Eigen::Matrix<double, System::kMeasurementSize, System::kMeasurementSize>;

  UnscentedKalmanFilter() {
    CalculateWeights();
  };

  void Predict(const StateSizeMatrix& process_noise) {
    const StateSizeMatrix& Q = process_noise;

    Eigen::Matrix<double, kNumSigma, System::kStateSize> SP = CalculateSigmaPoints();
    Eigen::Matrix<double, kNumSigma, System::kStateSize> TSP; // transformed sigma points
    for (size_t idx = 0; idx < kNumSigma; ++idx) {
      TSP.row(idx) = System::processModel(SP.row(idx).transpose()).transpose();
    }

    state_.x = TSP.transpose() * W; // (weighted) mean of transformed sigma points
    Eigen::Matrix<double, kNumSigma, System::kStateSize> TSP_minus_mean = TSP.rowwise() - state_.x.transpose();
    state_.P = Q + TSP_minus_mean.transpose() * (TSP_minus_mean.array().colwise() * W.array()).matrix();

    // Possibly a more readable version that the above
    //    state_.P.setZero();
    //    for (size_t idx = 0; idx < kNumSigma; ++idx) {
    //      state_.P += W(idx) * TSP_minus_mean.row(idx).transpose() * TSP_minus_mean.row(idx);
    //    }
    //    state_.P += Q;
  };

  void Update(const MeasurementSizeVector& measurement,
              const MeasurementSizeMatrix& measurement_noise) {
    const MeasurementSizeMatrix& R = measurement_noise;

    Eigen::Matrix<double, kNumSigma, System::kStateSize> SP = CalculateSigmaPoints();
    Eigen::Matrix<double, kNumSigma, System::kMeasurementSize> TSP; // transformed sigma points
    for (size_t idx = 0; idx < kNumSigma; ++idx) {
      TSP.row(idx) = System::measurementModel(SP.row(idx).transpose()).transpose();
    }

    MeasurementSizeVector z_hat = TSP.transpose() * W; // (weighted) mean of transformed sigma points (sampled predicted measurements)
    Eigen::Matrix<double, kNumSigma, System::kMeasurementSize> TSP_minus_pred_meas = TSP.rowwise() - z_hat.transpose();
    MeasurementSizeMatrix S = R + TSP_minus_pred_meas.transpose() * (TSP_minus_pred_meas.array().colwise() * W.array()).matrix();

    // Now calculate the cross-covariance between sigma point in state space, and sigma points in measurement space
    // Call this PHt because it's equivalent to P * H' in the standard equation K = P * H' * inv(S)
    Eigen::Matrix<double, kNumSigma, System::kStateSize> SP_minus_mean = SP.rowwise() - state_.x.transpose();
    Eigen::Matrix<double, System::kStateSize, System::kMeasurementSize> PHt = SP_minus_mean.transpose() * (TSP_minus_pred_meas.array().colwise() * W.array()).matrix();

    // Kalman gain
    Eigen::Matrix<double, System::kStateSize, System::kMeasurementSize> K = PHt * S.inverse();

    // Update state
    state_.x += K * (measurement - z_hat);
    state_.P = state_.P - K * S * K.transpose();
  };

  StateSizeVector GetState() const { return state_.x; };
  StateSizeMatrix GetCov() const { return state_.P; };
  void SetState(const StateSizeVector& new_state) { state_.x = new_state; };
  void SetCov(const StateSizeMatrix& new_cov) { state_.P = new_cov; };

 private:
  kf::State<System::kStateSize> state_;
  static constexpr size_t kNumSigma = 2 * System::kStateSize + 1;
  Eigen::Matrix<double, kNumSigma, 1> W; // Sigma point weights
  static constexpr double lambda = 1.0; // TODO review

  void CalculateWeights() {
    double w = 1.0 / (2.0 * (static_cast<double>(System::kStateSize) + lambda)); // all weights except w0
    W.setConstant(w);
    W(0) = lambda / (static_cast<double>(System::kStateSize) + lambda);
  };

  Eigen::Matrix<double, kNumSigma, System::kStateSize> CalculateSigmaPoints() const {
    // Use LLT decomposition to get the square root, https://eigen.tuxfamily.org/dox/classEigen_1_1LLT.html
    StateSizeMatrix P_mod = (static_cast<double>(System::kStateSize) + lambda) * state_.P;
    Eigen::LLT<StateSizeMatrix> llt_of_P_mod(P_mod);
    StateSizeMatrix L = llt_of_P_mod.matrixL();

    Eigen::Matrix<double, kNumSigma, System::kStateSize> SigmaPoints;
    SigmaPoints.row(0) = state_.x;
    for (size_t idx = 0; idx < System::kStateSize; ++idx) {
      // "for a root of the form P=LL', the columns of L are used." [1] (as opposed to P=L'L)
      SigmaPoints.row(2 * idx + 1) = (state_.x + L.col(idx)).transpose();
      SigmaPoints.row(2 * idx + 1 + 1) = (state_.x - L.col(idx)).transpose();
    }
    return SigmaPoints;
  }

};

} // namespace ekf1
} // namespace experimental

// 1. A New Method for the Nonlinear Transformation of Means and Covariances in Filters and Estimators, Julier, Uhlman, Durrant-Whyte

#endif //KALMAN_FILTERS_UNSCENTED_KF_H_
