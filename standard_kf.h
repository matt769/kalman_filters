//
// Created by matt on 22/01/2021.
//

#ifndef KALMAN_FILTERS_STANDARD_KF_H
#define KALMAN_FILTERS_STANDARD_KF_H

#include <functional>

#include <Eigen/Core>
#include <Eigen/LU>

#include "state.h"
#include "system.h"

namespace kf1 {

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

    // Extract F matrix from the function
    Eigen::Matrix<double, N, N> I = Eigen::Matrix<double, N, N>::Identity();
    Eigen::Matrix<double, N, N> F;
    for (size_t col_idx = 0; col_idx < N; ++col_idx) {
      F.col(col_idx) = process(I.col(col_idx));
    }

    state_.x = F * state_.x;
    state_.P = F * state_.P * F.transpose() + Q;
  };
  template<size_t M>
  void Update(MeasurementModel<N, M> measurement_model, const Eigen::Matrix<double, M, 1>& measurement,
              const Eigen::Matrix<double, M, M>& measurement_noise) {
    const Eigen::Matrix<double, M, M>& R = measurement_noise;

    // Extract H matrix from the function
    Eigen::Matrix<double, N, N> I = Eigen::Matrix<double, N, N>::Identity();
    Eigen::Matrix<double, M, N> H;
    for (size_t col_idx = 0; col_idx < N; ++col_idx) {
      H.col(col_idx) = measurement_model(I.col(col_idx));
    }

    Eigen::Matrix<double, M, 1> z_hat = H * state_.x;
    Eigen::Matrix<double, M, 1> y = measurement - z_hat;
    Eigen::Matrix<double, M, M> S = H * state_.P * H.transpose() + R;
    Eigen::Matrix<double, N, M> K = state_.P * H.transpose() * S.inverse();

    state_.x = state_.x + K * y;
    state_.P = (Eigen::Matrix<double, N, N>::Identity() - K * H) * state_.P;
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



namespace kf2 {

template<size_t N, size_t M>
class KalmanFilter {
 public:
  using ProcessModel = std::function<Eigen::Matrix<double, N, 1>(const Eigen::Matrix<double, N, 1>&)>;
  using MeasurementModel = std::function<Eigen::Matrix<double, M, 1>(const Eigen::Matrix<double, N, 1>&)>;

  KalmanFilter(ProcessModel process_model, MeasurementModel measurement_model)
      : process_model_(process_model), measurement_model_(measurement_model) {
    // Convert process model function into matrix
    const Eigen::Matrix<double, N, N> IF = Eigen::Matrix<double, N, N>::Identity();
    for (size_t col_idx = 0; col_idx < N; ++col_idx) {
      F_.col(col_idx) = process_model_(IF.col(col_idx));
    }
    // Convert measurement model function into matrix
    const Eigen::Matrix<double, N, N> IH = Eigen::Matrix<double, N, N>::Identity();
    for (size_t col_idx = 0; col_idx < N; ++col_idx) {
      H_.col(col_idx) = measurement_model_(IH.col(col_idx));
    }
  };

  void Predict(const Eigen::Matrix<double, N, N>& process_noise) {
    const Eigen::Matrix<double, N, N>& Q = process_noise;

    state_.x = F_ * state_.x;
    state_.P = F_ * state_.P * F_.transpose() + Q;
  };

  void Update(const Eigen::Matrix<double, M, 1>& measurement,
              const Eigen::Matrix<double, M, M>& measurement_noise) {
    const Eigen::Matrix<double, M, M>& R = measurement_noise;
    Eigen::Matrix<double, M, 1> z_hat = H_ * state_.x;
    Eigen::Matrix<double, M, 1> y = measurement - z_hat;
    Eigen::Matrix<double, M, M> S = H_ * state_.P * H_.transpose() + R;
    Eigen::Matrix<double, N, M> K = state_.P * H_.transpose() * S.inverse();

    state_.x = state_.x + K * y;
    state_.P = (Eigen::Matrix<double, N, N>::Identity() - K * H_) * state_.P;
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
  ProcessModel process_model_;
  Eigen::Matrix<double, N, N> F_; // process_model_ as matrix
  MeasurementModel measurement_model_;
  Eigen::Matrix<double, M, N> H_; // measurement_model_ as matrix
};

} // namespace kf2


namespace kf3 {

// Remove N and M, take from System instead
template<size_t N, size_t M, typename System>
class KalmanFilter {
public:
  using ProcessModel = std::function<Eigen::Matrix<double, N, 1>(const Eigen::Matrix<double, N, 1>&)>;
  using MeasurementModel = std::function<Eigen::Matrix<double, M, 1>(const Eigen::Matrix<double, N, 1>&)>;

  KalmanFilter()
  {
    // Convert process model function into matrix
    const Eigen::Matrix<double, N, N> IF = Eigen::Matrix<double, N, N>::Identity();
    for (size_t col_idx = 0; col_idx < N; ++col_idx) {
      F_.col(col_idx) = system_.processModel(IF.col(col_idx));
    }
    // Convert measurement model function into matrix
    const Eigen::Matrix<double, N, N> IH = Eigen::Matrix<double, N, N>::Identity();
    for (size_t col_idx = 0; col_idx < N; ++col_idx) {
      H_.col(col_idx) = system_.measurementModel(IH.col(col_idx));
    }
  };

  void Predict(const Eigen::Matrix<double, N, N>& process_noise) {
    const Eigen::Matrix<double, N, N>& Q = process_noise;

    state_.x = F_ * state_.x;
    state_.P = F_ * state_.P * F_.transpose() + Q;
  };

  void Update(const Eigen::Matrix<double, M, 1>& measurement,
              const Eigen::Matrix<double, M, M>& measurement_noise) {
    const Eigen::Matrix<double, M, M>& R = measurement_noise;
    Eigen::Matrix<double, M, 1> z_hat = H_ * state_.x;
    Eigen::Matrix<double, M, 1> y = measurement - z_hat;
    Eigen::Matrix<double, M, M> S = H_ * state_.P * H_.transpose() + R;
    Eigen::Matrix<double, N, M> K = state_.P * H_.transpose() * S.inverse();

    state_.x = state_.x + K * y;
    state_.P = (Eigen::Matrix<double, N, N>::Identity() - K * H_) * state_.P;
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
  ProcessModel process_model_;
  Eigen::Matrix<double, N, N> F_; // process_model_ as matrix
  MeasurementModel measurement_model_;
  Eigen::Matrix<double, M, N> H_; // measurement_model_ as matrix
  System system_;
};

} // namespace kf3

namespace kf4 {

// Add some typedefs
template<typename System>
class KalmanFilter {
public:
  using StateVector = Eigen::Matrix<double, System::kStateSize, 1>;
  using StateMatrix = Eigen::Matrix<double, System::kStateSize, System::kStateSize>;
  using MeasurementVector = Eigen::Matrix<double, System::kMeasurementSize, 1>;
  using MeasurementMatrix = Eigen::Matrix<double, System::kMeasurementSize, System::kMeasurementSize>;

  KalmanFilter()
  {
    // Convert process model function into matrix
    const StateMatrix IF = StateMatrix::Identity();
    for (size_t col_idx = 0; col_idx < System::kStateSize; ++col_idx) {
      F_.col(col_idx) = System::processModel(IF.col(col_idx));
    }
    // Convert measurement model function into matrix
    const StateMatrix IH = StateMatrix::Identity();
    for (size_t col_idx = 0; col_idx < System::kStateSize; ++col_idx) {
      H_.col(col_idx) = System::measurementModel(IH.col(col_idx));
    }
  };

  void Predict(const StateMatrix& process_noise) {
    const StateMatrix& Q = process_noise;

    state_.x = F_ * state_.x;
    state_.P = F_ * state_.P * F_.transpose() + Q;
  };

  void Update(const MeasurementVector& measurement, const MeasurementMatrix& measurement_noise) {
    const MeasurementMatrix& R = measurement_noise;
    MeasurementVector z_hat = H_ * state_.x;
    MeasurementVector y = measurement - z_hat;
    MeasurementMatrix S = H_ * state_.P * H_.transpose() + R;
    Eigen::Matrix<double, System::kStateSize, System::kMeasurementSize> K = state_.P * H_.transpose() * S.inverse();

    state_.x = state_.x + K * y;
    state_.P = (StateMatrix::Identity() - K * H_) * state_.P;
  };

  StateVector GetState() const {
    return state_.x;
  };
  StateMatrix GetCov() const {
    return state_.P;
  };
  void SetState(const StateVector& new_state) {
    state_.x = new_state;
  };
  void SetCov(const StateMatrix& new_cov) {
    state_.P = new_cov;
  };

private:
  kf::State<System::kStateSize> state_;
  StateMatrix F_; // process_model_ as matrix
  Eigen::Matrix<double, System::kMeasurementSize, System::kStateSize> H_; // measurement_model_ as matrix
};

} // namespace kf4

namespace kf5 {

template<typename System>
class KalmanFilter {
 public:
  using StateVector = Eigen::VectorXd;
  using StateMatrix = Eigen::MatrixXd;
  using MeasurementVector = Eigen::VectorXd;
  using MeasurementMatrix = Eigen::MatrixXd;

  KalmanFilter(const StateVector& x)
  : state_size_(x.size()), state_(x)
  {

    // Convert process model function into matrix
    const StateMatrix IF = StateMatrix::Identity(state_size_, state_size_);
    F_.resize(state_size_, state_size_);
    for (size_t col_idx = 0; col_idx < state_size_; ++col_idx) {
      F_.col(col_idx) = System::processModel(IF.col(col_idx));
    }

    // Convert measurement model function into matrix
    const StateMatrix IH = StateMatrix::Identity(state_size_, state_size_);
    H_.resize(System::kMeasurementSize, state_size_);
    for (size_t col_idx = 0; col_idx < state_size_; ++col_idx) {
      H_.col(col_idx) = System::measurementModel(IH.col(col_idx));
    }
  };

  void Predict(const StateMatrix& process_noise) {
    const StateMatrix& Q = process_noise;

    state_.x = F_ * state_.x;
    state_.P = F_ * state_.P * F_.transpose() + Q;
  };

  void Update(const MeasurementVector& measurement, const MeasurementMatrix& measurement_noise) {
    const MeasurementMatrix& R = measurement_noise;
    MeasurementVector z_hat = H_ * state_.x;
    MeasurementVector y = measurement - z_hat;
    MeasurementMatrix S = H_ * state_.P * H_.transpose() + R;
    Eigen::MatrixXd K = state_.P * H_.transpose() * S.inverse();

    state_.x = state_.x + K * y;
    state_.P = (StateMatrix::Identity(state_size_, state_size_) - K * H_) * state_.P;
  };

  StateVector GetState() const {
    return state_.x;
  };
  StateMatrix GetCov() const {
    return state_.P;
  };
  void SetState(const StateVector& new_state) {
    state_.x = new_state;
  };
  void SetCov(const StateMatrix& new_cov) {
    state_.P = new_cov;
  };

 private:
  size_t state_size_;
  kf_dynamic::State state_;
  StateMatrix F_; // process_model_ as matrix
  Eigen::MatrixXd H_; // measurement_model_ as matrix
};

} // namespace kf5

// Attempted to remove System from the class completely, but have 'invalid use of incomplete type'
//  errors with the process_model and measurement_model calls
// Maybe fix later (maybe...)
//namespace kf6 {
//
//class KalmanFilter {
// public:
//  using StateVector = Eigen::VectorXd;
//  using StateMatrix = Eigen::MatrixXd;
//  using MeasurementVector = Eigen::VectorXd;
//  using MeasurementMatrix = Eigen::MatrixXd;
//  using SystemModel = std::function<Eigen::VectorXd>(const Eigen::VectorXd&);
//
//  KalmanFilter(const StateVector& x)
//      : state_size_(x.size()), state_(x)
//  { };
//
//  void Predict(const StateMatrix& process_noise, const SystemModel& process_model) {
//    const StateMatrix& Q = process_noise;
//
//    // Convert process model function into matrix
//    const StateMatrix IF = StateMatrix::Identity(state_size_, state_size_);
//    F_.resize(state_size_, state_size_);
//    for (size_t col_idx = 0; col_idx < state_size_; ++col_idx) {
//      F_.col(col_idx) = process_model(IF.col(col_idx));
//    }
//
//    state_.x = F_ * state_.x;
//    state_.P = F_ * state_.P * F_.transpose() + Q;
//  };
//
//  void Update(const MeasurementVector& measurement, const MeasurementMatrix& measurement_noise, const SystemModel& measurement_model) {
//    const MeasurementMatrix& R = measurement_noise;
//
//    // Convert measurement model function into matrix
//    const StateMatrix IH = StateMatrix::Identity(state_size_, state_size_);
//    H_.resize(measurement.size(), state_size_);
//    for (size_t col_idx = 0; col_idx < state_size_; ++col_idx) {
//      H_.col(col_idx) = measurement_model(IH.col(col_idx));
//    }
//
//    MeasurementVector z_hat = H_ * state_.x;
//    MeasurementVector y = measurement - z_hat;
//    MeasurementMatrix S = H_ * state_.P * H_.transpose() + R;
//    Eigen::MatrixXd K = state_.P * H_.transpose() * S.inverse();
//
//    state_.x = state_.x + K * y;
//    state_.P = (StateMatrix::Identity(state_size_, state_size_) - K * H_) * state_.P;
//  };
//
//  StateVector GetState() const {
//    return state_.x;
//  };
//  StateMatrix GetCov() const {
//    return state_.P;
//  };
//  void SetState(const StateVector& new_state) {
//    state_.x = new_state;
//  };
//  void SetCov(const StateMatrix& new_cov) {
//    state_.P = new_cov;
//  };
//
// private:
//  size_t state_size_;
//  kf_dynamic::State state_;
//  StateMatrix F_; // process_model_ as matrix
//  Eigen::MatrixXd H_; // measurement_model_ as matrix
//};

//} // namespace kf6



#endif //KALMAN_FILTERS_STANDARD_KF_H
