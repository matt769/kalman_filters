//
// Created by matt on 24/01/2021.
//

#ifndef KALMAN_FILTERS_EXTENDED_KF_H_
#define KALMAN_FILTERS_EXTENDED_KF_H_

#include <functional>

#include <Eigen/Core>
#include <Eigen/LU>

#include "state.h"
#include "system.h"
#include "numeric_differentiation.h"

namespace ekf1 {

template<typename System>
class ExtendedKalmanFilter {
 public:
  using StateSizeVector = Eigen::Matrix<double, System::kStateSize, 1>;
  using StateSizeMatrix = Eigen::Matrix<double, System::kStateSize, System::kStateSize>;
  using MeasurementSizeVector = Eigen::Matrix<double, System::kMeasurementSize, 1>;
  using MeasurementSizeMatrix = Eigen::Matrix<double, System::kMeasurementSize, System::kMeasurementSize>;

  ExtendedKalmanFilter() {};

  void Predict(const StateSizeMatrix& process_noise) {
    const StateSizeMatrix& Q = process_noise;

    state_.x = System::processModel(state_.x);

    StateSizeMatrix JF = numeric_differentiation::CalculateJacobian<System::kStateSize, System::kStateSize>(state_.x,
                                                                                                            System::processModel);
    state_.P = JF * state_.P * JF.transpose() + Q;
  };

  void Update(const MeasurementSizeVector& measurement,
              const MeasurementSizeMatrix& measurement_noise) {
    const MeasurementSizeMatrix& R = measurement_noise;

    Eigen::Matrix<double, System::kMeasurementSize, System::kStateSize> JH =
        numeric_differentiation::CalculateJacobian<System::kStateSize, System::kMeasurementSize>(state_.x,
                                                                                                 System::measurementModel);

    MeasurementSizeVector z_hat = System::measurementModel(state_.x);
    MeasurementSizeVector y = measurement - z_hat;
    MeasurementSizeMatrix S = JH * state_.P * JH.transpose() + R;
    Eigen::Matrix<double, System::kStateSize, System::kMeasurementSize> K = state_.P * JH.transpose() * S.inverse();

    state_.x = state_.x + K * y;
    state_.P = (StateSizeMatrix::Identity() - K * JH) * state_.P;
  };

  StateSizeVector GetState() const { return state_.x; };
  StateSizeMatrix GetCov() const { return state_.P; };
  void SetState(const StateSizeVector& new_state) { state_.x = new_state; };
  void SetCov(const StateSizeMatrix& new_cov) { state_.P = new_cov; };

 private:
  kf::State<System::kStateSize> state_;
};

} // namespace ekf1

#endif // KALMAN_FILTERS_EXTENDED_KF_H_
