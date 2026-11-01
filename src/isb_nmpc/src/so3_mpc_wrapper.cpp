#include "isb_nmpc/so3_mpc_wrapper.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace isb_nmpc {

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

template <typename T>
So3MpcWrapper<T>::So3MpcWrapper() {
  std::memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
  std::memset(&acadoVariables, 0, sizeof(acadoVariables));
  acado_initializeSolver();

  Eigen::Matrix<T, kStateSize, 1> hover_state =
      Eigen::Matrix<T, kStateSize, 1>::Zero();
  hover_state(3) = 1.0;
  hover_state(kStateFocusW) = 1.0;
  hover_state(kStateRange) = 10.0;
  hover_state(kStateInterceptSpeed) = 0.0;

  acado_initial_state_ = hover_state.template cast<float>();
  acado_states_ = hover_state.replicate(1, kSamples + 1).template cast<float>();
  acado_inputs_.setZero();
  acado_inputs_.row(kThrust).setConstant(9.81f);
  fillRunningReference();
  fillTerminalReference();

  Eigen::Matrix<T, kCostSize, kCostSize> Q =
      Eigen::Matrix<T, kCostSize, kCostSize>::Identity() * T(10.0);
  Eigen::Matrix<T, kInputSize, kInputSize> R =
      Eigen::Matrix<T, kInputSize, kInputSize>::Identity() * T(0.1);
  R(kSlackU, kSlackU) = T(500.0);
  R(kSlackV, kSlackV) = T(500.0);
  initializeOnlineData();
  setCosts(Q, R, T(0.0));
  setLimits(T(0.0), T(20.0), T(3.0), T(3.0), T(1.0));

  acado_initializeNodesByForwardSimulation();
  acado_preparationStep();
  acado_is_prepared_ = true;
}

template <typename T>
So3MpcWrapper<T>::So3MpcWrapper(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R)
    : So3MpcWrapper() {
  setCosts(Q, R);
}

template <typename T>
bool So3MpcWrapper<T>::setCosts(
    const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
    const T terminal_line_cost,
    const T state_cost_scaling,
    const T input_cost_scaling) {
  if (terminal_line_cost < 0.0 || state_cost_scaling < 0.0 ||
      input_cost_scaling < 0.0) {
    ROS_ERROR("SO3 MPC: Cost scaling must be non-negative.");
    return false;
  }

  Q_ = Q;
  R_ = R;
  terminal_line_cost_ = terminal_line_cost;
  state_cost_scaling_ = state_cost_scaling;
  input_cost_scaling_ = input_cost_scaling;
  return updateCostWeights();
}

template <typename T>
bool So3MpcWrapper<T>::updateCostWeights() {
  W_.setZero();
  W_.block(0, 0, kCostSize, kCostSize) = Q_;
  W_.block(kCostSize, kCostSize, kInputSize, kInputSize) = R_;
  WN_.setZero();
  WN_(0, 0) = terminal_line_cost_;
  WN_(1, 1) = Q_(3, 3);
  WN_(2, 2) = Q_(4, 4);

  for (int i = 0; i < kSamples; ++i) {
    acado_W_.block(0, i * kRefSize, kCostSize, kCostSize) =
        Q_.template cast<float>();
    acado_W_.block(kCostSize, i * kRefSize + kCostSize,
                   kInputSize, kInputSize) =
        R_.template cast<float>();
  }
  acado_W_end_ = WN_.template cast<float>();
  return true;
}

template <typename T>
bool So3MpcWrapper<T>::setLimits(T min_thrust, T max_thrust,
                                 T max_rollrate, T max_pitchrate,
                                 T max_yawrate) {
  if (min_thrust < 0.0 || max_thrust <= min_thrust ||
      max_rollrate <= 0.0 || max_pitchrate <= 0.0 || max_yawrate <= 0.0) {
    ROS_ERROR("SO3 MPC: Invalid input limits.");
    return false;
  }

  acado_lower_bounds_.row(kThrust).setConstant(static_cast<float>(min_thrust));
  acado_upper_bounds_.row(kThrust).setConstant(static_cast<float>(max_thrust));
  acado_lower_bounds_.row(kRateX).setConstant(static_cast<float>(-max_rollrate));
  acado_upper_bounds_.row(kRateX).setConstant(static_cast<float>(max_rollrate));
  acado_lower_bounds_.row(kRateY).setConstant(static_cast<float>(-max_pitchrate));
  acado_upper_bounds_.row(kRateY).setConstant(static_cast<float>(max_pitchrate));
  acado_lower_bounds_.row(kRateZ).setConstant(static_cast<float>(-max_yawrate));
  acado_upper_bounds_.row(kRateZ).setConstant(static_cast<float>(max_yawrate));
  acado_lower_bounds_.row(kInterceptSpeedRate).setConstant(-1.0f);
  acado_upper_bounds_.row(kInterceptSpeedRate).setConstant(1.0f);
  acado_lower_bounds_.row(kSlackU).setZero();
  acado_upper_bounds_.row(kSlackU).setConstant(1000.0f);
  acado_lower_bounds_.row(kSlackV).setZero();
  acado_upper_bounds_.row(kSlackV).setConstant(1000.0f);

  acado_online_data_.row(kOdThrustLimits).setConstant(
      static_cast<float>(min_thrust));
  acado_online_data_.row(kOdThrustLimits + 1).setConstant(
      static_cast<float>(max_thrust));
  acado_online_data_.row(kOdRateLimits).setConstant(
      static_cast<float>(max_rollrate));
  acado_online_data_.row(kOdRateLimits + 1).setConstant(
      static_cast<float>(max_pitchrate));
  acado_online_data_.row(kOdRateLimits + 2).setConstant(
      static_cast<float>(max_yawrate));
  return true;
}

template <typename T>
bool So3MpcWrapper<T>::setMaxRollAngle(T max_roll_angle_rad) {
  constexpr T kHalfPi = T(1.57079632679489661923);
  if (!std::isfinite(max_roll_angle_rad) || max_roll_angle_rad <= T(0.0) ||
      max_roll_angle_rad >= kHalfPi) {
    ROS_ERROR("SO3 MPC: max roll angle must be finite and in (0, pi/2). Got %.6f rad.",
              static_cast<double>(max_roll_angle_rad));
    return false;
  }
  acado_online_data_.row(kOdTanMaxRoll).setConstant(
      static_cast<float>(std::tan(max_roll_angle_rad)));
  return true;
}

template <typename T>
bool So3MpcWrapper<T>::setCameraParameters(const Eigen::Quaternion<T>& q_B_C) {
  Eigen::Quaternion<T> q = q_B_C;
  if (q.norm() > T(1e-6)) {
    q.normalize();
  } else {
    q = Eigen::Quaternion<T>::Identity();
  }
  const Eigen::Matrix<T, 4, 1> q_vec(q.w(), q.x(), q.y(), q.z());
  acado_online_data_.block(kOdCameraQuaternion, 0, 4, kSamples + 1) =
      q_vec.replicate(1, kSamples + 1).template cast<float>();
  return true;
}

template <typename T>
bool So3MpcWrapper<T>::setTargetVelocity(
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& velocity) {
  acado_online_data_.block(kOdTargetVelocity, 0, 3, kSamples + 1) =
      velocity.replicate(1, kSamples + 1).template cast<float>();
  return true;
}

template <typename T>
bool So3MpcWrapper<T>::setInterceptSpeedLimits(
    T max_intercept_speed, T min_intercept_speed,
    T los_speed_schedule_kh, T los_speed_schedule_kv,
    T max_delta_intercept_speed) {
  if (max_intercept_speed < T(0.0) ||
      min_intercept_speed < T(0.0) ||
      min_intercept_speed > max_intercept_speed ||
      los_speed_schedule_kh < T(0.0) ||
      los_speed_schedule_kv < T(0.0) ||
      max_delta_intercept_speed <= T(0.0)) {
    ROS_ERROR("SO3 MPC: intercept speed limits or LOS scheduling gains are "
              "invalid.");
    return false;
  }
  max_intercept_speed_ = max_intercept_speed;
  min_intercept_speed_ = min_intercept_speed;
  los_speed_schedule_kh_ = los_speed_schedule_kh;
  los_speed_schedule_kv_ = los_speed_schedule_kv;
  max_delta_intercept_speed_ = max_delta_intercept_speed;
  acado_online_data_.row(kOdMaxInterceptSpeed).setConstant(
      static_cast<float>(max_intercept_speed));
  scheduled_intercept_speed_ = max_intercept_speed_;
  acado_reference_.row(kRefInterceptSpeed).setConstant(
      static_cast<float>(scheduled_intercept_speed_));
  return updateInterceptSpeedRateBounds();
}

template <typename T>
bool So3MpcWrapper<T>::setInterceptSpeedReference(
    const Eigen::Quaternion<T>& q_image) {
  Eigen::Quaternion<T> focus = q_image;
  if (focus.norm() > T(1e-6)) {
    focus.normalize();
  } else {
    focus = Eigen::Quaternion<T>::Identity();
  }

  const T nx = T(2.0) *
      (focus.w() * focus.y() + focus.x() * focus.z());
  const T ny = T(2.0) *
      (focus.y() * focus.z() - focus.w() * focus.x());
  const T nz = T(1.0) -
      T(2.0) * (focus.x() * focus.x() + focus.y() * focus.y());
  const T epsilon = T(1e-2);
  const T horizontal_error =
      nx / std::sqrt(nx * nx + nz * nz + epsilon * epsilon);
  const T vertical_error =
      ny / std::sqrt(ny * ny + nz * nz + epsilon * epsilon);
  const T alpha =
      std::exp(-(los_speed_schedule_kh_ * horizontal_error * horizontal_error +
                 los_speed_schedule_kv_ * vertical_error * vertical_error));
  scheduled_intercept_speed_ =
      min_intercept_speed_ +
      alpha * (max_intercept_speed_ - min_intercept_speed_);
  acado_reference_.row(kRefInterceptSpeed).setConstant(
      static_cast<float>(scheduled_intercept_speed_));
  return true;
}

template <typename T>
bool So3MpcWrapper<T>::setImagePlaneParameters(T center_u, T center_v,
                                               T radius_u, T radius_v) {
  if (!std::isfinite(static_cast<double>(center_u)) ||
      !std::isfinite(static_cast<double>(center_v)) ||
      !std::isfinite(static_cast<double>(radius_u)) ||
      !std::isfinite(static_cast<double>(radius_v)) ||
      radius_u <= T(0.0) || radius_v <= T(0.0)) {
    ROS_ERROR("SO3 MPC: invalid normalized image-plane parameters.");
    return false;
  }

  Eigen::Matrix<T, 2, 1> center(center_u, center_v);
  Eigen::Matrix<T, 2, 1> radius(radius_u, radius_v);
  acado_online_data_.block(kOdImageCenter, 0, 2, kSamples + 1) =
      center.replicate(1, kSamples + 1).template cast<float>();
  acado_online_data_.block(kOdImageRadius, 0, 2, kSamples + 1) =
      radius.replicate(1, kSamples + 1).template cast<float>();
  return true;
}

template <typename T>
bool So3MpcWrapper<T>::setTerminalLine(
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& point,
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& direction) {
  Eigen::Matrix<T, 3, 1> unit_direction = direction;
  if (unit_direction.norm() > T(1e-6)) {
    unit_direction.normalize();
  } else {
    unit_direction << T(1.0), T(0.0), T(0.0);
  }
  acado_online_data_.block(kOdTerminalLinePoint, 0, 3, kSamples + 1) =
      point.replicate(1, kSamples + 1).template cast<float>();
  acado_online_data_.block(kOdTerminalLineDirection, 0, 3, kSamples + 1) =
      unit_direction.replicate(1, kSamples + 1).template cast<float>();
  return true;
}

template <typename T>
bool So3MpcWrapper<T>::setTimeScale(T time_scale) {
  if (!std::isfinite(static_cast<double>(time_scale)) || time_scale <= T(0.0)) {
    ROS_ERROR("SO3 MPC: time_scale must be positive and finite.");
    return false;
  }
  acado_online_data_.row(kOdTimeScale).setConstant(
      static_cast<float>(time_scale));
  return true;
}

template <typename T>
bool So3MpcWrapper<T>::setReferencePose(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state) {
  acado_states_ = state.replicate(1, kSamples + 1).template cast<float>();
  acado_inputs_.setZero();
  acado_inputs_.row(kThrust).setConstant(9.81f);
  acado_inputs_.row(kInterceptSpeedRate).setZero();
  return true;
}

template <typename T>
bool So3MpcWrapper<T>::setTrajectory(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples + 1>> inputs) {
  acado_states_ = states.template cast<float>();
  acado_inputs_ =
      inputs.template block<kInputSize, kSamples>(0, 0).template cast<float>();
  fillRunningReference();
  fillTerminalReference();
  return true;
}

template <typename T>
bool So3MpcWrapper<T>::setWarmStart(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs) {
  if (!states.allFinite() || !inputs.allFinite()) {
    return false;
  }
  acado_states_ = states.template cast<float>();
  acado_inputs_ = inputs.template cast<float>();
  acado_is_prepared_ = prepare();
  return acado_is_prepared_;
}

template <typename T>
void So3MpcWrapper<T>::resetSolverWorkspace() {
  // acado_initializeSolver() clears ACADOworkspace and restores the generated
  // constraint defaults.  It intentionally leaves references, weights and
  // online data in ACADOvariables untouched; the controller reapplies all
  // configurable limits immediately after this call.
  acado_initializeSolver();
  acado_is_prepared_ = false;
  last_feedback_status_ = 0;
  last_preparation_ok_ = true;
  last_kkt_ = T(0.0);
}

template <typename T>
bool So3MpcWrapper<T>::solve(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state) {
  acado_states_ = state.replicate(1, kSamples + 1).template cast<float>();
  acado_inputs_.setZero();
  acado_inputs_.row(kThrust).setConstant(9.81f);
  acado_initializeNodesByForwardSimulation();
  if (!prepare()) {
    return false;
  }
  return update(state);
}

template <typename T>
bool So3MpcWrapper<T>::update(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    bool do_preparation) {
  if (!acado_is_prepared_) {
    ROS_WARN("SO3 MPC: Solver was triggered without preparation, abort.");
    // MPC Debug Logging （MPC调试日志）
    last_feedback_status_ = -1;
    last_preparation_ok_ = false;
    // MPC Debug Logging End
    return false;
  }

  acado_initial_state_ = state.template cast<float>();
  if (acado_initial_state_.segment(3, 4).dot(acado_states_.col(0).segment(3, 4)) <
      T(0.0)) {
    acado_initial_state_.segment(3, 4) = -acado_initial_state_.segment(3, 4);
  }

  const int feedback_status = acado_feedbackStep();
  // MPC Debug Logging （MPC调试日志）
  last_feedback_status_ = feedback_status;
  last_kkt_ = acado_getKKT();
  // MPC Debug Logging End
  acado_is_prepared_ = false;
  if (feedback_status != 0) {
    ROS_WARN_THROTTLE(1.0, "SO3 MPC: feedback step returned %d.",
                      feedback_status);
  }

  if (do_preparation) {
    // MPC Debug Logging （MPC调试日志）
    last_preparation_ok_ = prepare();
    return feedback_status == 0 && last_preparation_ok_;
    // MPC Debug Logging End
  }
  // MPC Debug Logging （MPC调试日志）
  last_preparation_ok_ = true;
  // MPC Debug Logging End
  return feedback_status == 0;
}

template <typename T>
bool So3MpcWrapper<T>::prepare() {
  const int status = acado_preparationStep();
  acado_is_prepared_ = status == 0;
  // MPC Debug Logging （MPC调试日志）
  last_preparation_ok_ = acado_is_prepared_;
  // MPC Debug Logging End
  if (status != 0) {
    ROS_WARN_THROTTLE(1.0, "SO3 MPC: preparation step returned %d.", status);
  }
  return acado_is_prepared_;
}

template <typename T>
void So3MpcWrapper<T>::getState(
    int node_index, Eigen::Ref<Eigen::Matrix<T, kStateSize, 1>> return_state) {
  const int bounded_index = std::max(0, std::min(kSamples, node_index));
  return_state = acado_states_.col(bounded_index).template cast<T>();
}

template <typename T>
void So3MpcWrapper<T>::getStates(
    Eigen::Ref<Eigen::Matrix<T, kStateSize, kSamples + 1>> return_states) {
  return_states = acado_states_.template cast<T>();
}

template <typename T>
void So3MpcWrapper<T>::getInput(
    int node_index, Eigen::Ref<Eigen::Matrix<T, kInputSize, 1>> return_input) {
  const int bounded_index = std::max(0, std::min(kSamples - 1, node_index));
  return_input = acado_inputs_.col(bounded_index).template cast<T>();
}

template <typename T>
void So3MpcWrapper<T>::getInputs(
    Eigen::Ref<Eigen::Matrix<T, kInputSize, kSamples>> return_inputs) {
  return_inputs = acado_inputs_.template cast<T>();
}

template <typename T>
void So3MpcWrapper<T>::fillRunningReference() {
  acado_reference_.setZero();
  acado_reference_.row(kRefInterceptSpeed).setConstant(
      static_cast<float>(scheduled_intercept_speed_));
}

template <typename T>
void So3MpcWrapper<T>::fillTerminalReference() {
  acado_reference_end_.setZero();
}

template <typename T>
void So3MpcWrapper<T>::initializeOnlineData() {
  acado_online_data_.setZero();

  setCameraParameters(Eigen::Quaternion<T>::Identity());
  setTargetVelocity(Eigen::Matrix<T, 3, 1>::Zero());
  setInterceptSpeedLimits(T(2.0), T(0.0), T(0.0), T(0.0), T(1.0));
  setImagePlaneParameters(T(0.0), T(0.0),
                          T(0.5) * T(640.0) / T(115.0) - T(0.3),
                          T(0.5) * T(480.0) / T(115.0) - T(0.3));
  setTerminalLine(Eigen::Matrix<T, 3, 1>::Zero(),
                  Eigen::Matrix<T, 3, 1>::UnitX());
  setTimeScale(T(1.0));
  setMaxRollAngle(T(55.0 * 3.14159265358979323846 / 180.0));
}

template <typename T>
bool So3MpcWrapper<T>::updateInterceptSpeedRateBounds() {
  const T effective_delta = max_delta_intercept_speed_;
  acado_online_data_.row(kOdMaxDeltaInterceptSpeed).setConstant(
      static_cast<float>(effective_delta));
  acado_lower_bounds_.row(kInterceptSpeedRate).setConstant(
      static_cast<float>(-effective_delta));
  acado_upper_bounds_.row(kInterceptSpeedRate).setConstant(
      static_cast<float>(effective_delta));
  return true;
}

template class So3MpcWrapper<float>;
template class So3MpcWrapper<double>;

}  // namespace isb_nmpc
