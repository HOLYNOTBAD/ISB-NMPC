#pragma once

#include "dc_s2ekf/filter_common.h"

namespace dc_s2ekf {

struct S2FilterConfig {
  double measurement_quality_min_weight = 0.25;
  double sigma_bearing_meas_rad = 0.04;
  double initial_bearing_std_rad = 0.20;
  double initial_omega_std_rad_s = 3.0;
  // Standard deviation of camera angular-rate error applied as known-input
  // uncertainty during each controlled propagation.
  double gyro_noise_std_rad_s = 0.01;
  double process_accel_std_rad_s2 = 4.0;
};

class S2Filter {
 public:
  using Vec3 = math::Vec3;

  // Filter lifecycle.
  void configure(const S2FilterConfig& config);
  void reset();
  void initialize(const Vec3& bearing_camera, const ros::Time& stamp);

  // Stored-state propagation and correction.
  // Propagate a camera-frame bearing through the known camera rotation first,
  // then through the residual (target/translation) LOS motion.
  void propagate(double dt,
                 const Eigen::Matrix3d& rotation_new_camera_from_old_camera);
  void update(const Vec3& bearing_meassured,
              const S2MeasurementNoise& measurement_noise,
              double measurement_quality, const ros::Time& stamp);

  // Read-only future bearing prediction.
  Vec3 predictBearing(
      double dt,
      const Eigen::Matrix3d& rotation_new_camera_from_old_camera) const;

  // Filter status and posterior bearing.
  const Vec3& bearing() const { return state_.bearing_c; }
  bool initialized() const { return initialized_; }
  const ros::Time& lastUpdateTime() const { return last_update_time_; }
  S2TrackPhase trackPhase() const { return track_phase_; }

 private:
  static void makeTangentBasis(const Vec3& bearing,
                               const Vec3& preferred_e1,
                               Vec3* e1, Vec3* e2);
  static Eigen::Vector2d toTangentCoordinates(const Vec3& value,
                                              const S2FilterState& state);
  static Vec3 fromTangentCoordinates(double x, double y,
                                     const S2FilterState& state);
  static void stabilizeCovariance(S2Covariance* covariance);
  static void transportToBearing(const Vec3& new_bearing,
                                 S2FilterState* state);
  static void rotateStateFrame(
      const Eigen::Matrix3d& rotation_new_camera_from_old_camera,
      S2FilterState* state);
  void propagateState(
      double dt, const Eigen::Matrix3d& rotation_new_camera_from_old_camera,
      S2FilterState* state) const;
  void correctState(const Vec3& measurement,
                    const S2MeasurementNoise& measurement_noise,
                    double measurement_quality, S2FilterState* state);

  S2FilterConfig config_;
  S2FilterState state_;
  bool initialized_ = false;
  ros::Time last_update_time_;
  S2TrackPhase track_phase_ = S2TrackPhase::INIT_BEARING;
};

}  // namespace dc_s2ekf
