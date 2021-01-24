//
// Created by matt on 23/01/2021.
//

#ifndef SIMPLE_UKF_SYSTEM_H_
#define SIMPLE_UKF_SYSTEM_H_

#include <cstddef>
#include <functional>

#include <Eigen/Core>

#include "state.h"

namespace systems {

class SimpleSystem {
public:
  static constexpr size_t kStateSize = 4;
  using StateVector = kf::State<kStateSize>::StateVector;
  using StateMatrix = kf::State<kStateSize>::CovMatrix;

  StateVector processModel(const StateVector &x) {
    StateMatrix F = StateMatrix::Identity();
    F(0, 2) = 0.1; // px' = px + vx*dt
    F(1, 3) = 0.1; // py' = py + vy*dt
    return F * x;
  };

  static constexpr size_t kMeasurementSize = 2;
  using MeasurementVector = Eigen::Matrix<double, kMeasurementSize, 1>;
  using MeasurementNoise =
      Eigen::Matrix<double, kMeasurementSize, kMeasurementSize>;

  MeasurementVector measurementModel(const StateVector &x) {
    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    // Measure position only
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    return H * x;
  };
};

class AnotherSystem {
public:
  static constexpr size_t kStateSize = 6;
  using StateVector = kf::State<kStateSize>::StateVector;
  using StateMatrix = kf::State<kStateSize>::CovMatrix;

  StateVector processModel(const StateVector &x) {
    StateMatrix F = StateMatrix::Identity();
    F(0, 3) = 0.1; // px' = px + vx*dt
    F(1, 4) = 0.1; // py' = py + vy*dt
    F(2, 5) = 0.1; // pz' = pz + vz*dt
    return F * x;
  };

  static constexpr size_t kMeasurementSize = 3;
  using MeasurementVector = Eigen::Matrix<double, kMeasurementSize, 1>;
  using MeasurementNoise =
  Eigen::Matrix<double, kMeasurementSize, kMeasurementSize>;

  MeasurementVector measurementModel(const StateVector &x) {
    Eigen::Matrix<double, kMeasurementSize, kStateSize> H = Eigen::Matrix<double, kMeasurementSize, kStateSize>::Zero();
    // Measure position only
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 2) = 1.0;
    return H * x;
  };
};


} // namespace systems

#endif // SIMPLE_UKF_SYSTEM_H_
