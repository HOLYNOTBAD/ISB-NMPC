#include "dc_s2ekf/log_size_filter.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

#include "dc_s2ekf/filter_common.h"

namespace dc_s2ekf {

void LogSizeFilter::initialize(const Measurement& measurement) {
  state_ = {std::log(std::max(config_.min_box_size_px, measurement[2])),
            std::log(std::max(config_.min_box_size_px, measurement[3])),
            0.0,
            0.0};
  covariance_.setZero();
  covariance_.diagonal() << 0.08 * 0.08, 0.08 * 0.08, 1.0, 1.0;
}

void LogSizeFilter::update(const Measurement& measurement, double dt,
                           double measurement_quality) {
  predict(dt);
  const double width = std::max(config_.min_box_size_px, measurement[2]);
  const double height = std::max(config_.min_box_size_px, measurement[3]);
  const double safe_quality = std::max(0.05, measurement_quality);
  const double width_std = config_.measurement_std_px / width;
  const double height_std = config_.measurement_std_px / height;
  updateScalar(0, std::log(width), width_std * width_std / safe_quality);
  updateScalar(1, std::log(height), height_std * height_std / safe_quality);
}

SizeState LogSizeFilter::predictSize(double dt) const {
  SizeState predicted = state_;
  propagateState(dt, &predicted);
  return predicted;
}

void LogSizeFilter::propagateState(double dt, SizeState* state) {
  if (state == nullptr || dt <= 0.0) {
    return;
  }
  state->head<2>().noalias() += dt * state->tail<2>();
}

void LogSizeFilter::predict(double dt) {
  if (dt <= 0.0) {
    return;
  }
  propagateState(dt, &state_);
  SizeCovariance transition = SizeCovariance::Identity();
  transition.topRightCorner<2, 2>() = dt * Eigen::Matrix2d::Identity();
  covariance_ = eigen::similarityTransform<4>(transition, covariance_);

  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;
  covariance_.topLeftCorner<2, 2>().diagonal().array() +=
      0.25 * dt4 * config_.process_noise;
  covariance_.topRightCorner<2, 2>().diagonal().array() +=
      0.5 * dt3 * config_.process_noise;
  covariance_.bottomLeftCorner<2, 2>().diagonal().array() +=
      0.5 * dt3 * config_.process_noise;
  covariance_.bottomRightCorner<2, 2>().diagonal().array() +=
      dt2 * config_.process_noise;
}

void LogSizeFilter::updateScalar(int measurement_index, double measurement,
                                 double measurement_variance) {
  const double variance = std::max(1e-6, measurement_variance);
  const double innovation_variance =
      covariance_(measurement_index, measurement_index) + variance;
  if (innovation_variance <= 1e-9) {
    return;
  }
  const SizeState gain =
      covariance_.col(measurement_index) / innovation_variance;
  state_ += gain * (measurement - state_[measurement_index]);

  Eigen::Matrix<double, 1, 4> observation =
      Eigen::Matrix<double, 1, 4>::Zero();
  observation[measurement_index] = 1.0;
  const Eigen::Matrix<double, 1, 1> noise =
      Eigen::Matrix<double, 1, 1>::Constant(variance);
  covariance_ = eigen::josephCovariance<4, 1>(
      covariance_, gain, observation, noise);
  eigen::symmetrizeAndFloor<4>(&covariance_, 1e-9);
}

}  // namespace dc_s2ekf
