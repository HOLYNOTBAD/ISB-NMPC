#include "dc_s2ekf/s2_filter.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace dc_s2ekf {

using namespace math;

namespace {

// ==========================================================================
//  S2 Geometry（S2几何）
// ==========================================================================
Vec3 logMap(const Vec3& reference, const Vec3& bearing) {
  const Vec3 normalized_reference = normalize(reference);
  const Vec3 normalized_bearing = normalize(bearing);
  Vec3 axis = cross(normalized_reference, normalized_bearing);
  const double sin_angle = clamp(norm(axis), 0.0, 1.0);
  const double cos_angle =
      clamp(dot(normalized_reference, normalized_bearing), -1.0, 1.0);
  const double angle = std::atan2(sin_angle, cos_angle);
  if (sin_angle < 1e-8 || angle < 1e-8) {
    return Vec3{};
  }
  axis = axis / sin_angle;
  return projectTangent(
      angle * cross(axis, normalized_reference), normalized_reference);
}

Vec3 expMap(const Vec3& reference, const Vec3& tangent_delta) {
  const Vec3 normalized_reference = normalize(reference);
  const Vec3 delta = projectTangent(tangent_delta, normalized_reference);
  const double theta = norm(delta);
  if (theta < 1e-8) {
    return normalized_reference;
  }
  const Vec3 direction = delta / theta;
  return normalize(
      std::cos(theta) * normalized_reference + std::sin(theta) * direction,
      normalized_reference);
}

}  // namespace

// ==========================================================================
//  Filter Lifecycle（滤波器生命周期）
// ==========================================================================
void S2Filter::configure(const S2FilterConfig& config) {
  config_ = config;
}

void S2Filter::reset() {
  state_ = S2FilterState{};
  initialized_ = false;
  last_update_time_ = ros::Time();
  track_phase_ = S2TrackPhase::INIT_BEARING;
}

void S2Filter::initialize(const Vec3& bearing, const ros::Time& stamp) {
  // 名义状态
  state_.bearing_c = normalize(bearing);
  state_.omega_c = Vec3{};
  // 切空间基底
  makeTangentBasis(state_.bearing_c, Vec3{1.0, 0.0, 0.0},
                   &state_.basis_e1_c, &state_.basis_e2_c);
  // 误差状态协方差矩阵
  state_.P.setZero();
  const double bearing_variance =
      config_.initial_bearing_std_rad * config_.initial_bearing_std_rad;
  const double omega_variance =
      config_.initial_omega_std_rad_s * config_.initial_omega_std_rad_s;
  state_.P.diagonal() << bearing_variance, bearing_variance, omega_variance,
      omega_variance;

  initialized_ = true;
  last_update_time_ = stamp;
  track_phase_ = S2TrackPhase::INIT_BEARING;
}

// ==========================================================================
//  Public Filter Operations（滤波器外部操作）
// ==========================================================================
void S2Filter::propagate(
    double dt,
    const Eigen::Matrix3d& rotation_new_camera_from_old_camera) {
  propagateState(dt, rotation_new_camera_from_old_camera, &state_);
}

void S2Filter::update(
    const Vec3& bearing_meassured, const S2MeasurementNoise& measurement_noise,
    double measurement_quality, const ros::Time& stamp) {
  // 归一化来自视觉的观测值
  const Vec3 measurement = normalize(bearing_meassured);
  track_phase_ = S2TrackPhase::TRACKING;
  // 状态校正
  correctState(measurement, measurement_noise, measurement_quality, &state_);
  last_update_time_ = stamp;
}

S2Filter::Vec3 S2Filter::predictBearing(
    double dt,
    const Eigen::Matrix3d& rotation_new_camera_from_old_camera) const {
  S2FilterState predicted = state_;
  propagateState(dt, rotation_new_camera_from_old_camera, &predicted);
  return predicted.bearing_c;
}

// ==========================================================================
//  Tangent-Space Geometry（切空间几何）
// ==========================================================================
void S2Filter::makeTangentBasis(const Vec3& bearing,
                                const Vec3& preferred_e1,
                                Vec3* e1, Vec3* e2) {
  const Vec3 b = normalize(bearing);
  Vec3 first = projectTangent(preferred_e1, b);
  if (norm(first) < 1e-6) {
    const Vec3 fallback = std::fabs(b.z) < 0.85 ? Vec3{0.0, 0.0, 1.0}
                                                : Vec3{1.0, 0.0, 0.0};
    first = projectTangent(fallback, b);
  }
  first = normalize(first, Vec3{1.0, 0.0, 0.0});
  *e1 = first;
  *e2 = normalize(cross(b, first), Vec3{0.0, 1.0, 0.0});
}

Eigen::Vector2d S2Filter::toTangentCoordinates(
    const Vec3& value, const S2FilterState& state) {
  return Eigen::Vector2d(dot(value, state.basis_e1_c),
                         dot(value, state.basis_e2_c));
}

S2Filter::Vec3 S2Filter::fromTangentCoordinates(
    double x, double y, const S2FilterState& state) {
  return x * state.basis_e1_c + y * state.basis_e2_c;
}

void S2Filter::stabilizeCovariance(S2Covariance* covariance) {
  S2ErrorState floor = S2ErrorState::Constant(1e-8);
  floor.head<2>().setConstant(1e-10);
  dc_s2ekf::eigen::symmetrizeAndFloor<4>(covariance, floor);
}

// ==========================================================================
//  State Frame and Basis Transport（状态坐标系与基底传输）
// ==========================================================================
void S2Filter::transportToBearing(const Vec3& new_bearing,
                                  S2FilterState* state) {
  Vec3 new_e1;
  Vec3 new_e2;
  makeTangentBasis(new_bearing, state->basis_e1_c, &new_e1, &new_e2);
  Eigen::Matrix2d rotation;
  rotation << dot(new_e1, state->basis_e1_c),
      dot(new_e1, state->basis_e2_c),
      dot(new_e2, state->basis_e1_c),
      dot(new_e2, state->basis_e2_c);
  S2Covariance transform = S2Covariance::Zero();
  transform.block<2, 2>(0, 0) = rotation;
  transform.block<2, 2>(2, 2) = rotation;
  state->P = dc_s2ekf::eigen::similarityTransform<4>(
      transform, state->P);
  state->bearing_c = normalize(new_bearing);
  state->basis_e1_c = new_e1;
  state->basis_e2_c = new_e2;
  stabilizeCovariance(&state->P);
}

void S2Filter::rotateStateFrame(
    const Eigen::Matrix3d& rotation_new_camera_from_old_camera,
    S2FilterState* state) {
  if (state == nullptr) {
    return;
  }
  // The nominal state and its tangent basis undergo the same rigid rotation.
  // Consequently the numeric coordinates of the local error, and therefore P,
  // are unchanged by this known control input.
  state->bearing_c =
      normalize(rotateMat(rotation_new_camera_from_old_camera,
                          state->bearing_c));
  state->omega_c =
      projectTangent(
          rotateMat(rotation_new_camera_from_old_camera, state->omega_c),
          state->bearing_c);
  state->basis_e1_c =
      normalize(rotateMat(rotation_new_camera_from_old_camera,
                          state->basis_e1_c));
  state->basis_e2_c =
      normalize(rotateMat(rotation_new_camera_from_old_camera,
                          state->basis_e2_c));
  stabilizeCovariance(&state->P);
}

// ==========================================================================
//  Model Propagation and Correction（模型传播与校正）
// ==========================================================================
void S2Filter::propagateState(
    double dt,
    const Eigen::Matrix3d& rotation_new_camera_from_old_camera,
    S2FilterState* state) const {
  if (state == nullptr) {
    return;
  }
  // Rotate the nominal state and its tangent basis into the new camera frame.
  rotateStateFrame(rotation_new_camera_from_old_camera, state);
  if (dt <= 0.0) {
    return;
  }
  S2Covariance transition = S2Covariance::Identity();
  transition.block<2, 2>(0, 2) = dt * Eigen::Matrix2d::Identity();
  S2Covariance covariance =
      dc_s2ekf::eigen::similarityTransform<4>(
          transition, state->P);

  const double sigma = std::max(1e-3, config_.process_accel_std_rad_s2);
  const double q = sigma * sigma;
  const double dt2 = dt * dt;
  covariance.block<2, 2>(0, 0).diagonal().array() +=
      0.25 * dt2 * dt2 * q;
  covariance.block<2, 2>(0, 2).diagonal().array() +=
      0.5 * dt2 * dt * q;
  covariance.block<2, 2>(2, 0).diagonal().array() +=
      0.5 * dt2 * dt * q;
  covariance.block<2, 2>(2, 2).diagonal().array() += dt2 * q;
  state->P = covariance;

  // propagate the nominal state 
  const Vec3 new_bearing =
      expMap(state->bearing_c, dt * state->omega_c);
  state->omega_c = projectTangent(state->omega_c, new_bearing);

  transportToBearing(new_bearing, state);

  const double gyro_sigma = std::max(0.0, config_.gyro_noise_std_rad_s);
  state->P.topLeftCorner<2, 2>().diagonal().array() +=
      dt * dt * gyro_sigma * gyro_sigma;
  stabilizeCovariance(&state->P);
}

void S2Filter::correctState(
    const Vec3& measurement,
    const S2MeasurementNoise& measurement_noise,
    double measurement_quality, S2FilterState* state) {
  const Vec3 innovation = logMap(state->bearing_c, measurement);
  const Eigen::Vector2d y = toTangentCoordinates(innovation, *state);
  const double quality = clamp(
      measurement_quality, config_.measurement_quality_min_weight, 1.0);
  const double sigma = std::max(1e-4, config_.sigma_bearing_meas_rad);
  const Eigen::Vector2d ju(
      dot(state->basis_e1_c, measurement_noise.pixel_u_jacobian_c),
      dot(state->basis_e2_c, measurement_noise.pixel_u_jacobian_c));
  const Eigen::Vector2d jv(
      dot(state->basis_e1_c, measurement_noise.pixel_v_jacobian_c),
      dot(state->basis_e2_c, measurement_noise.pixel_v_jacobian_c));
  Eigen::Matrix2d noise =
      (measurement_noise.variance_u_px2 * ju * ju.transpose() +
       measurement_noise.variance_v_px2 * jv * jv.transpose()).eval();
  noise.diagonal() = noise.diagonal().array().max(
      0.25 * sigma * sigma / quality);

  const Eigen::Matrix2d innovation_covariance =
      state->P.topLeftCorner<2, 2>() + noise;
  const Eigen::Matrix2d inverse = innovation_covariance.inverse();

  S2Gain gain;
  gain.noalias() = state->P.leftCols<2>() * inverse;
  const S2ErrorState delta = gain * y;
  const S2FilterState prior = *state;
  const Vec3 new_bearing = expMap(
      prior.bearing_c,
      fromTangentCoordinates(delta[0], delta[1], prior));
  state->omega_c = projectTangent(
      prior.omega_c +
          fromTangentCoordinates(delta[2], delta[3], prior),
      new_bearing);
  state->P = dc_s2ekf::eigen::josephCovariance<4, 2>(
      prior.P, gain, noise);
  state->basis_e1_c = prior.basis_e1_c;
  state->basis_e2_c = prior.basis_e2_c;
  transportToBearing(new_bearing, state);
}

}  // namespace dc_s2ekf
