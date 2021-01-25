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
//    std::cout << SP << "\n\n";
    Eigen::Matrix<double, kNumSigma, System::kStateSize> TSP; // transformed sigma points
    for (size_t idx = 0; idx < kNumSigma; ++idx) {
      TSP.row(idx) = System::processModel(SP.row(idx).transpose()).transpose();
    }
//    std::cout << TSP << "\n\n";

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

    // try copying koide version to see where differences are

    Eigen::Matrix<double, kNumSigma, System::kStateSize> SP = CalculateSigmaPoints();
    Eigen::Matrix<double, kNumSigma, System::kMeasurementSize> TSP; // transformed sigma points
    for (size_t idx = 0; idx < kNumSigma; ++idx) {
      TSP.row(idx) = System::measurementModel(SP.row(idx).transpose()).transpose();
    }

    std::cout << "SP\n" << SP << "\n\n";
    std::cout << "TSP\n" << TSP << "\n\n";

    MeasurementSizeVector z_hat = TSP.transpose() * W; // (weighted) mean of transformed sigma points (sampled predicted measurements)
    Eigen::Matrix<double, kNumSigma, System::kMeasurementSize> TSP_minus_measurement = TSP.rowwise() - measurement.transpose();
    MeasurementSizeMatrix S = R + TSP_minus_measurement.transpose() * (TSP_minus_measurement.array().colwise() * W.array()).matrix();

    std::cout << "W\n" << W.transpose() << "\n\n";
    std::cout << "TSP_minus_measurement\n" << TSP_minus_measurement << "\n\n";

    // try koide
    Eigen::VectorXd expected_measurement_mean = Eigen::VectorXd::Zero(System::kMeasurementSize);
    for (int i = 0; i < kNumSigma; i++) {
      expected_measurement_mean += W[i] * TSP.row(i);
    }
    Eigen::VectorXd expected_measurement_cov = Eigen::VectorXd::Zero(System::kMeasurementSize, System::kMeasurementSize);
    for (int i = 0; i < kNumSigma; i++) {
      Eigen::VectorXd diff = TSP.row(i).transpose() - expected_measurement_mean;
      expected_measurement_cov += W[i] * diff * diff.transpose();
    }
    expected_measurement_cov += R;

    std::cout << "expected_measurement_mean\n" << expected_measurement_mean.transpose() << "\n\n";
    std::cout << "expected_measurement_cov\n" << expected_measurement_cov << "\n\n";



//    MeasurementSizeMatrix S_alt;
//    S_alt.setZero();
//    for (size_t i = 0; i < kNumSigma; ++i) {
//      S_alt += W(i) * TSP_minus_measurement.row(i).transpose() * TSP_minus_measurement.row(i);
//    }
//    S_alt += R;
//    std::cout << "S_alt\n" << S << "\n\n";

    // Now calculate the cross-covariance between sigma point in state space, and sigma points in measurement space
    // Call this PHt because it's equivalent to P * H' in the standard equation K = P * H' * inv(S)
    Eigen::Matrix<double, kNumSigma, System::kStateSize> SP_minus_mean = SP.rowwise() - state_.x.transpose();
    Eigen::Matrix<double, System::kStateSize, System::kMeasurementSize> PHt = SP_minus_mean.transpose() * (TSP_minus_measurement.array().colwise() * W.array()).matrix();

    // Kalman gain
    Eigen::Matrix<double, System::kStateSize, System::kMeasurementSize> K = PHt * S.inverse();

    std::cout << "z_hat\n" << z_hat << "\n\n";
    std::cout << "S\n" << S << "\n\n";
    std::cout << "K\n" <<  K << "\n\n";
    std::cout << "y\n" <<  (measurement - z_hat).transpose() << "\n\n";

    // K looks wrong in second iteration


    // Update state
    state_.x += K * (measurement - z_hat);
//    state_.P = (StateSizeMatrix::Identity() - K * PHt) * state_.P;
//    std::cout << state_.P << "\n\n";
    state_.P = state_.P - K * S * K.transpose();

//    MeasurementSizeVector z_hat = System::measurementModel(state_.x);
//    MeasurementSizeVector y = measurement - z_hat;
//    MeasurementSizeMatrix S = JH * state_.P * JH.transpose() + R;
//    Eigen::Matrix<double, System::kStateSize, System::kMeasurementSize> K = state_.P * JH.transpose() * S.inverse();
//    state_.x = state_.x + K * y;
//    state_.P = (StateSizeMatrix::Identity() - K * JH) * state_.P;
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
    double w = 1.0 / (2.0 * (System::kStateSize + lambda)); // all weights except w0
    W.setConstant(w);
    W(0) = lambda / (System::kStateSize + lambda);
  };

  Eigen::Matrix<double, kNumSigma, System::kStateSize> CalculateSigmaPoints() const {
    // Use LLT decomposition to get the square root, https://eigen.tuxfamily.org/dox/classEigen_1_1LLT.html
    StateSizeMatrix P_mod = (static_cast<double>(System::kStateSize) + lambda) * state_.P;
    Eigen::LLT<StateSizeMatrix> llt_of_P_mod(P_mod);
    StateSizeMatrix L = llt_of_P_mod.matrixL();

    std::cout << P_mod << "\n\n";
    std::cout << L << "\n\n";
    std::cout << L * L.transpose() << "\n\n";


    Eigen::Matrix<double, kNumSigma, System::kStateSize> SigmaPoints;
    SigmaPoints.row(0) = state_.x;
    for (size_t idx = 0; idx < System::kStateSize; ++idx) {
      std::cout << idx << ": " << L.col(idx).transpose() << '\n';
      // "for a root of the form P=LL', the columns of L are used." [1] (as opposed to P=L'L)
      SigmaPoints.row(2 * idx + 1) = (state_.x + L.col(idx)).transpose();
      SigmaPoints.row(2 * idx + 1 + 1) = (state_.x - L.col(idx)).transpose();
      std::cout << '\n';
    }
    return SigmaPoints;
  }

};

} // namespace ekf1
} // namespace experimental

// 1. A New Method for the Nonlinear Transformation of Means and Covariances in Filters and Estimators, Julier, Uhlman, Durrant-Whyte

#endif //KALMAN_FILTERS_UNSCENTED_KF_H_
