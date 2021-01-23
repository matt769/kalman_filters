//
// Created by matt on 23/01/2021.
//

#ifndef SIMPLE_UKF_SYSTEM_H_
#define SIMPLE_UKF_SYSTEM_H_

#include <cstddef>
#include <functional>

#include <Eigen/Core>

#include "state.h"

// This should just be a template
// If it's too much of a template, is it actually useful?
// Maybe it should be abstract class that needs to be implemented
// Or just the types?
// Should this just be instructions? No code
// How will KF class this?
// Will KF class use this?

// Should system be templated on State?
// Or state size?
// Should system include state?

// I want a place to nicely collect the models and noise

// Ok, let's actually define everything here
// It's heavily tied to the state size, so can we include that here?

// Ultimately, I'd like to define a filter like
// KalmanFiler<systems::SimpleSystem>



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

} // namespace systems

#endif // SIMPLE_UKF_SYSTEM_H_
