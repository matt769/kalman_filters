//
// Created by matt on 23/01/2021.
//

#ifndef KALMAN_FILTERS_SYSTEM_H_
#define KALMAN_FILTERS_SYSTEM_H_

#include <cstddef>
#include <functional>
#include <cmath>

#include <Eigen/Core>

#include "state.h"

namespace systems {

// state is p and v, 1 dimension
class TestSystem {
 public:
  static constexpr size_t kStateSize = 2;
  using StateVector = kf::State<kStateSize>::StateVector;
  using StateMatrix = kf::State<kStateSize>::CovMatrix;

  static StateVector processModel(const StateVector &x) {
    StateMatrix F = StateMatrix::Identity();
    F(0, 2) = 0.1; // p = p + v*dt
    return F * x;
  };

  static constexpr size_t kMeasurementSize = 1;
  using MeasurementVector = Eigen::Matrix<double, kMeasurementSize, 1>;
  using MeasurementNoise =
  Eigen::Matrix<double, kMeasurementSize, kMeasurementSize>;

  static MeasurementVector measurementModel(const StateVector &x) {
    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    // Measure position only
    H(0, 0) = 1.0;
    return H * x;
  };
};

class SimpleSystem {
public:
  static constexpr size_t kStateSize = 4;
  using StateVector = kf::State<kStateSize>::StateVector;
  using StateMatrix = kf::State<kStateSize>::CovMatrix;

  static StateVector processModel(const StateVector &x) {
    StateMatrix F = StateMatrix::Identity();
    F(0, 2) = 0.1; // px' = px + vx*dt
    F(1, 3) = 0.1; // py' = py + vy*dt
    return F * x;
  };

  static constexpr size_t kMeasurementSize = 2;
  using MeasurementVector = Eigen::Matrix<double, kMeasurementSize, 1>;
  using MeasurementNoise =
      Eigen::Matrix<double, kMeasurementSize, kMeasurementSize>;

  static MeasurementVector measurementModel(const StateVector &x) {
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

  static StateVector processModel(const StateVector &x) {
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

  static MeasurementVector measurementModel(const StateVector &x) {
    Eigen::Matrix<double, kMeasurementSize, kStateSize> H = Eigen::Matrix<double, kMeasurementSize, kStateSize>::Zero();
    // Measure position only
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 2) = 1.0;
    return H * x;
  };
};


// TODO class NonLinearProcessSystem
// TODO class NonLinearMeasurementSystem

// Now let's say our state is px, py, heading, velocity
// And we measure vx and vy (a bit weird/contrived but just want to make it non-linear)
class NonLinearSystem {
 public:
  static constexpr size_t kStateSize = 4;
  using StateVector = kf::State<kStateSize>::StateVector;
  using StateMatrix = kf::State<kStateSize>::CovMatrix;
  using ProcessNoise = kf::State<kStateSize>::CovMatrix;

  static StateVector processModel(const StateVector &x) {
    const double& px = x(0);
    const double& py = x(1);
    const double& heading = x(2);
    const double& v = x(3);


    StateVector new_state;
    const double dt = 0.1; // fixed timestep for now
    new_state(0) = px + dt * v * std::cos(heading);
    new_state(1) = py + dt * v * std::sin(heading);
    new_state(2) = heading; // constant
    new_state(3) = v; // constant

    return new_state;
  };

  static constexpr size_t kMeasurementSize = 2;
  using MeasurementVector = Eigen::Matrix<double, kMeasurementSize, 1>;
  using MeasurementNoise =
  Eigen::Matrix<double, kMeasurementSize, kMeasurementSize>;

  static MeasurementVector measurementModel(const StateVector &x) {
    MeasurementVector predicted_measurement;
//    const double& px = x(0);
//    const double& py = x(1);
    const double& heading = x(2);
    const double& v = x(3);

    predicted_measurement(0) = v * std::cos(heading);
    predicted_measurement(1) = v * std::sin(heading);

    return predicted_measurement;
  };
};


} // namespace systems

#endif // KALMAN_FILTERS_SYSTEM_H_
