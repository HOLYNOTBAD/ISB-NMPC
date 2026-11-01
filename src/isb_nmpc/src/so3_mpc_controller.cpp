#include "isb_nmpc/so3_mpc_controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <utility>

namespace isb_nmpc {

// ============================================================================
// 1. Utility （工具函数）
// ============================================================================
namespace {
// Used for generating log file names.（用于生成日志文件名）
std::string makeTimestampString() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm time_info{};
  localtime_r(&now_time, &time_info);

  std::ostringstream stream;
  stream << std::put_time(&time_info, "%Y%m%d_%H%M%S");
  return stream.str();
}

template <typename T>
Eigen::Quaternion<T> makeLevelOrientationPreservingYaw(
    const Eigen::Quaternion<T>& orientation) {
  Eigen::Quaternion<T> q = orientation;
  if (q.norm() > T(1e-6)) {
    q.normalize();
  } else {
    q = Eigen::Quaternion<T>::Identity();
  }
  const Eigen::Matrix<T, 3, 3> rotation = q.toRotationMatrix();
  const T yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  return Eigen::Quaternion<T>(Eigen::AngleAxis<T>(yaw, Eigen::Matrix<T, 3, 1>::UnitZ()));
}

}  // namespace

// ============================================================================
// 2. Constructor （构造函数）
// ============================================================================
template <typename T>
So3MpcController<T>::So3MpcController(
    const ros::NodeHandle& nh, const ros::NodeHandle& pnh,
    const std::string& topic)
    : nh_(nh),
      pnh_(pnh),
      est_state_(Eigen::Matrix<T, kStateSize, 1>::Zero()),
      predicted_states_(Eigen::Matrix<T, kStateSize, kSamples + 1>::Zero()),
      predicted_inputs_(Eigen::Matrix<T, kInputSize, kSamples>::Zero()) {
  est_state_(kOriW) = 1.0;
  est_state_(kFocusW) = 1.0;
  est_state_(kRange) = 10.0;
  est_state_(kInterceptSpeed) = 0.0;
  last_valid_states_.setZero();
  last_valid_inputs_.setZero();
  last_valid_command_.zero();
  pnh_.param("max_invalid_retries", max_invalid_retries_, max_invalid_retries_);
  max_invalid_retries_ = std::max(0, max_invalid_retries_);
  pnh_.param("max_zero_control_retries", max_zero_control_retries_,
             max_zero_control_retries_);
  max_zero_control_retries_ = std::max(0, max_zero_control_retries_);

  // ---------------------------------------------------------------------------
//  MPC Debug Logging （MPC调试日志）
// ---------------------------------------------------------------------------
  bool enable_log = true;
  std::string debug_log_dir =
      (std::filesystem::temp_directory_path() / "isb_nmpc" / "interception").string();
  pnh_.param("enable_log", enable_log, enable_log);
  pnh_.param<std::string>("debug_log_dir", debug_log_dir, debug_log_dir);
  if (enable_log) {
    std::error_code error;
    std::filesystem::create_directories(debug_log_dir, error);
    const std::filesystem::path log_path =
        std::filesystem::path(debug_log_dir) /
        ("isb_nmpc_" + makeTimestampString() + ".csv");
    debug_log_file_.open(log_path, std::ios::out | std::ios::trunc);
    if (debug_log_file_.is_open()) {
      debug_log_file_ << std::setprecision(9);
      debug_log_file_
          << "seq,stamp,"
          << "feedback_status,preparation_ok,kkt,has_target,target_in_main_view,"
          << "range,current_intercept_speed,scheduled_intercept_speed,max_intercept_speed,"
          << "max_delta_intercept_speed,img_err_u,img_err_v,"
          << "uav_px,uav_py,uav_pz,uav_qw,uav_qx,uav_qy,uav_qz,"
          << "uav_vx,uav_vy,uav_vz,"
          << "target_px,target_py,target_pz,target_vx,target_vy,target_vz,"
          << "target_dir_x,target_dir_y,target_dir_z,"
          << "target_unit_x,target_unit_y,target_unit_z,"
          << "q_image_w,q_image_x,q_image_y,q_image_z,"
          << "focus_w,focus_x,focus_y,focus_z,"
          << "x1_range,x1_intercept_speed,"
          << "u0_T,u0_wx,u0_wy,u0_wz,u0_ds,u0_sv1,u0_sv2,"
          << "cmd_T,cmd_wx,cmd_wy,cmd_wz,"
          << "xN_px,xN_py,xN_pz,xN_qw,xN_qx,xN_qy,xN_qz,"
          << "xN_vx,xN_vy,xN_vz,"
          << "xN_focus_w,xN_focus_x,xN_focus_y,xN_focus_z,"
          << "xN_range,xN_intercept_speed,xN_img_err_u,xN_img_err_v,"
          << "current_roll_deg,predicted_max_abs_roll_deg,"
          << "min_roll_constraint_margin\n";
      ROS_INFO("SO3 MPC: writing debug log to %s.",
               log_path.string().c_str());
    } else {
      ROS_WARN("SO3 MPC: failed to open debug log file '%s'.",
               log_path.string().c_str());
    }
  }
  // ---------------------------------------------------------------------------

  pub_predicted_trajectory_ = nh_.advertise<nav_msgs::Path>(topic, 1);
  std::string failure_topic = "/nmpc/failure";
  pnh_.param<std::string>("failure_topic", failure_topic, failure_topic);
  failure_pub_ = nh_.advertise<std_msgs::Bool>(failure_topic, 1);
  std::string solve_latency_topic = "/isb_nmpc/solve_latency_ms";
  pnh_.param<std::string>("solve_latency_topic", solve_latency_topic,
                          solve_latency_topic);
  solve_latency_pub_ = nh_.advertise<std_msgs::Float32>(solve_latency_topic, 1);

  if (!params_.loadParameters(pnh_)) {
    ROS_ERROR("[%s] Could not load SO3 MPC parameters.",
              pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
  setNewParams(params_);
}

// ============================================================================
// 3. Activation & Reset （激活/重置）
// ============================================================================
template <typename T>
void So3MpcController<T>::resetForActivation() {
  // Every trial is an independent optimization problem.  Reset ACADO's
  // workspace so a terminal/failed QP from a previous interception cannot
  // contaminate the next trial's cold start, then restore configured values.
  mpc_wrapper_.resetSolverWorkspace();
  setNewParams(params_);
  solve_from_scratch_ = true;
  initialize_intercept_speed_from_state_ = true;
  // Each interception is a new engagement.  In bearing-only mode there is no
  // target velocity state from which to reinitialize this value, so retaining
  // it would start the next trial at the previous terminal speed (typically
  // near the 9.5 m/s limit) and immediately saturate the pitch-rate command.
  current_intercept_speed_ = T(0.0);
  has_last_valid_solution_ = false;
  has_last_valid_command_ = false;
  consecutive_invalid_outputs_ = 0;
  consecutive_zero_control_outputs_ = 0;
  mpc_wrapper_.setTimeScale(T(1.0));
  mpc_wrapper_.setInterceptSpeedLimits(params_.intercept_speed_,
                                       params_.min_intercept_speed_,
                                       params_.los_speed_schedule_kh_,
                                       params_.los_speed_schedule_kv_,
                                       params_.max_delta_intercept_speed_);
  mpc_wrapper_.setImagePlaneParameters(params_.image_center_u_,
                                       params_.image_center_v_,
                                       params_.image_radius_u_,
                                       params_.image_radius_v_);
  publishFailure(false);
}

// ============================================================================
// 4. Core MPC Run （核心MPC求解）
// ============================================================================
template <typename T>
quadrotor_common::ControlCommand So3MpcController<T>::run(
    const quadrotor_common::QuadStateEstimate& state_estimate,
    const So3MpcParams<T>& params,
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& target_position,
    const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& target_velocity,
    const Eigen::Quaternion<T>& q_image,
    bool target_in_main_view) {
  ros::Time call_time = ros::Time::now();
  const clock_t start = clock();

  // Param hot-reload.（参数热加载）
  if (params.changed_) {
    params_ = params;
    setNewParams(params_);
  }

  mpc_wrapper_.setTimeScale(T(1.0));

  // Fill est_state_ from odometry + target + q_image.（填充MPC状态向量）
  const bool has_target_state = target_position.allFinite() &&
                                target_velocity.allFinite();
  setStateEstimate(state_estimate, target_position, target_velocity,
                   q_image, has_target_state);
  // Solve MPC — cold start on first iteration, warm start thereafter.（求解MPC：首次冷启动，后续warm start）
  bool solve_ok = false;
  if (solve_from_scratch_) {
    solve_ok = mpc_wrapper_.solve(est_state_);
    solve_from_scratch_ = false;
  } else {
    solve_ok = mpc_wrapper_.update(est_state_);
  }

  // Validate solution.（校验求解结果）
  mpc_wrapper_.getStates(predicted_states_);
  mpc_wrapper_.getInputs(predicted_inputs_);
  const bool states_finite = predicted_states_.allFinite();
  const bool inputs_finite = predicted_inputs_.allFinite();
  if (!solve_ok || !states_finite || !inputs_finite) {
    ++consecutive_invalid_outputs_;
    ROS_WARN_THROTTLE(
        0.5,
        "SO3 MPC: solver validation failed: solve_ok=%s states_finite=%s inputs_finite=%s feedback_status=%d preparation_ok=%s kkt=%.6g.",
        solve_ok ? "true" : "false", states_finite ? "true" : "false",
        inputs_finite ? "true" : "false",
        mpc_wrapper_.getLastFeedbackStatus(),
        mpc_wrapper_.getLastPreparationOk() ? "true" : "false",
        static_cast<double>(mpc_wrapper_.getLastKkt()));
    if (has_last_valid_solution_ && has_last_valid_command_ &&
        consecutive_invalid_outputs_ <= max_invalid_retries_ &&
        mpc_wrapper_.setWarmStart(last_valid_states_, last_valid_inputs_)) {
      // A transient invalid solve is recoverable while a previously validated
      // command is available.  Do not signal a terminal NMPC failure: both the
      // FSM and loop runner intentionally stop the trial on failure=true.
      publishFailure(false);
      ROS_WARN_THROTTLE(
          0.5,
          "SO3 MPC: invalid solver output, restoring last valid warm start and retrying (%d/%d).",
          consecutive_invalid_outputs_, max_invalid_retries_);
      quadrotor_common::ControlCommand command = last_valid_command_;
      command.timestamp = call_time;
      command.expected_execution_time = call_time;
      return command;
    }

    solve_from_scratch_ = true;
    current_intercept_speed_ = T(0.0);
    has_last_valid_solution_ = false;
    has_last_valid_command_ = false;
    // A failed cold start may leave the generated solver workspace unusable.
    // Reset it before the next recovery tick and restore runtime parameters.
    mpc_wrapper_.resetSolverWorkspace();
    setNewParams(params_);
    ++consecutive_zero_control_outputs_;
    if (consecutive_zero_control_outputs_ <= max_zero_control_retries_) {
      // The first cold-start solve has no prior solution to restore.  Give it
      // its own zero-control recovery budget instead of subtracting the unused
      // warm-start budget (which previously produced counters such as -4/5).
      publishFailure(false);
      ROS_WARN_THROTTLE(
          0.5,
          "SO3 MPC: invalid solver output, publishing zero body-rate recovery command (%d/%d) and resetting warm start.",
          consecutive_zero_control_outputs_,
          max_zero_control_retries_);
      return makeZeroControlRecoveryCommand(call_time);
    }

    // Recovery is exhausted.  Only this terminal condition is exposed as an
    // NMPC failure so downstream state machines do not abort on a single bad
    // solver tick.
    publishFailure(true);
    ROS_WARN_THROTTLE(
        0.5,
        "SO3 MPC: invalid solver output after %d consecutive output(s) and %d/%d zero-control recovery attempt(s), switching to hover command.",
        consecutive_invalid_outputs_, consecutive_zero_control_outputs_,
        max_zero_control_retries_);
    return makeHoverCommand(
        call_time,
        makeLevelOrientationPreservingYaw(
            state_estimate.orientation.template cast<T>()));  // Fallback to hover.（降级悬停）
  }

  // Extract intercept speed and publish predicted trajectory.（提取拦截速度，发布预测轨迹）
  current_intercept_speed_ =
      std::max<T>(T(0.0), predicted_states_(kInterceptSpeed,
                                            std::min(1, kSamples)));
  publishPrediction(predicted_states_, call_time);

  // Reuse the controller's existing smoothed computation timing for both the
  // terminal log and the FPV telemetry topic.
  const clock_t end = clock();
  timing_feedback_ = 0.9 * timing_feedback_ +
                     0.1 * static_cast<T>(end - start) / CLOCKS_PER_SEC;
  std_msgs::Float32 solve_latency;
  solve_latency.data = static_cast<float>(timing_feedback_ * T(1000.0));
  solve_latency_pub_.publish(solve_latency);
  if (params_.print_info_) {
    ROS_INFO_THROTTLE(1.0, "SO3 MPC Timing: Latency: %1.1f ms",
                      timing_feedback_ * 1000);
  }

  // Convert MPC output to ControlCommand.（MPC输出→飞控指令）
  const int command_state_index = kSamples > 0 ? 1 : 0;
  const quadrotor_common::ControlCommand command =
      updateControlCommand(predicted_states_.col(command_state_index),
                           predicted_inputs_.col(0), call_time);
  consecutive_invalid_outputs_ = 0;
  consecutive_zero_control_outputs_ = 0;
  publishFailure(false);
  has_last_valid_solution_ = true;
  has_last_valid_command_ = true;
  last_valid_states_ = predicted_states_;
  last_valid_inputs_ = predicted_inputs_;
  last_valid_command_ = command;
  // ---------------------------------------------------------------------------
//  MPC Debug Logging （MPC调试日志）
// ---------------------------------------------------------------------------
  if (debug_log_file_.is_open()) {
    const auto& first_input = predicted_inputs_.col(0);
    const auto& command_state = predicted_states_.col(command_state_index);
    const auto& terminal_state = predicted_states_.col(kSamples);
    const Eigen::Matrix<T, 3, 1> target_direction =
        target_position - state_estimate.position.template cast<T>();
    Eigen::Matrix<T, 3, 1> target_unit =
        has_target_state ? target_direction : Eigen::Matrix<T, 3, 1>::UnitX();
    if (target_unit.norm() > T(1e-6)) {
      target_unit.normalize();
    } else {
      target_unit = Eigen::Matrix<T, 3, 1>::UnitX();
    }
    const T focus_w = est_state_(kFocusW);
    const T focus_x = est_state_(kFocusX);
    const T focus_y = est_state_(kFocusY);
    const T focus_z = est_state_(kFocusZ);
    const T image_nx = T(2.0) * focus_w * focus_y +
                       T(2.0) * focus_x * focus_z;
    const T image_ny = T(2.0) * focus_y * focus_z -
                       T(2.0) * focus_w * focus_x;
    const T image_nz = T(1.0) - T(2.0) * focus_x * focus_x -
                       T(2.0) * focus_y * focus_y;
    const T image_u = image_nx / (image_nz + T(1e-2));
    const T image_v = image_ny / (image_nz + T(1e-2));
    const T image_err_u =
        (image_u - params_.image_center_u_) / params_.image_radius_u_;
    const T image_err_v =
        (image_v - params_.image_center_v_) / params_.image_radius_v_;
    const T terminal_focus_w = terminal_state(kFocusW);
    const T terminal_focus_x = terminal_state(kFocusX);
    const T terminal_focus_y = terminal_state(kFocusY);
    const T terminal_focus_z = terminal_state(kFocusZ);
    const T terminal_nx = T(2.0) * terminal_focus_w * terminal_focus_y +
                          T(2.0) * terminal_focus_x * terminal_focus_z;
    const T terminal_ny = T(2.0) * terminal_focus_y * terminal_focus_z -
                          T(2.0) * terminal_focus_w * terminal_focus_x;
    const T terminal_nz = T(1.0) -
                          T(2.0) * terminal_focus_x * terminal_focus_x -
                          T(2.0) * terminal_focus_y * terminal_focus_y;
    const T terminal_image_u = terminal_nx / (terminal_nz + T(1e-2));
    const T terminal_image_v = terminal_ny / (terminal_nz + T(1e-2));
    const T terminal_img_err_u =
        (terminal_image_u - params_.image_center_u_) / params_.image_radius_u_;
    const T terminal_img_err_v =
        (terminal_image_v - params_.image_center_v_) / params_.image_radius_v_;
    constexpr T kRadToDeg = T(180.0 / 3.14159265358979323846);
    const auto rollMetrics = [](const auto& state) {
      const T qw = state(kOriW);
      const T qx = state(kOriX);
      const T qy = state(kOriY);
      const T qz = state(kOriZ);
      const T roll_s = T(2.0) * (qy * qz + qw * qx);
      const T roll_c = T(1.0) - T(2.0) * qx * qx - T(2.0) * qy * qy;
      return std::make_pair(std::atan2(roll_s, roll_c),
                            std::make_pair(roll_s, roll_c));
    };
    const T current_roll = rollMetrics(est_state_).first;
    T predicted_max_abs_roll = T(0.0);
    T min_roll_constraint_margin = std::numeric_limits<T>::infinity();
    const T tan_max_roll = std::tan(params_.max_roll_angle_rad_);
    for (int i = 0; i <= kSamples; ++i) {
      const auto metrics = rollMetrics(predicted_states_.col(i));
      predicted_max_abs_roll =
          std::max(predicted_max_abs_roll, std::abs(metrics.first));
      const T roll_s = metrics.second.first;
      const T roll_c = metrics.second.second;
      min_roll_constraint_margin = std::min(
          min_roll_constraint_margin,
          tan_max_roll * roll_c - std::abs(roll_s));
    }

    debug_log_file_
        << debug_log_sequence_++ << ','
        << call_time.toSec() << ','
        << mpc_wrapper_.getLastFeedbackStatus() << ','
        << (mpc_wrapper_.getLastPreparationOk() ? 1 : 0) << ','
        << mpc_wrapper_.getLastKkt() << ','
        << (has_target_state ? 1 : 0) << ','
        << (target_in_main_view ? 1 : 0) << ','
        << est_state_(kRange) << ','
        << current_intercept_speed_ << ','
        << mpc_wrapper_.getScheduledInterceptSpeed() << ','
        << params_.intercept_speed_ << ','
        << params_.max_delta_intercept_speed_ << ','
        << image_err_u << ','
        << image_err_v << ','
        << state_estimate.position.x() << ','
        << state_estimate.position.y() << ','
        << state_estimate.position.z() << ','
        << state_estimate.orientation.w() << ','
        << state_estimate.orientation.x() << ','
        << state_estimate.orientation.y() << ','
        << state_estimate.orientation.z() << ','
        << state_estimate.velocity.x() << ','
        << state_estimate.velocity.y() << ','
        << state_estimate.velocity.z() << ','
        << target_position.x() << ','
        << target_position.y() << ','
        << target_position.z() << ','
        << target_velocity.x() << ','
        << target_velocity.y() << ','
        << target_velocity.z() << ','
        << target_direction.x() << ','
        << target_direction.y() << ','
        << target_direction.z() << ','
        << target_unit.x() << ','
        << target_unit.y() << ','
        << target_unit.z() << ','
        << q_image.w() << ','
        << q_image.x() << ','
        << q_image.y() << ','
        << q_image.z() << ','
        << est_state_(kFocusW) << ','
        << est_state_(kFocusX) << ','
        << est_state_(kFocusY) << ','
        << est_state_(kFocusZ) << ','
        << command_state(kRange) << ','
        << command_state(kInterceptSpeed) << ','
        << first_input(kThrust) << ','
        << first_input(kRateX) << ','
        << first_input(kRateY) << ','
        << first_input(kRateZ) << ','
        << first_input(kInterceptSpeedRate) << ','
        << first_input(kSlackU) << ','
        << first_input(kSlackV) << ','
        << command.collective_thrust << ','
        << command.bodyrates.x() << ','
        << command.bodyrates.y() << ','
        << command.bodyrates.z() << ','
        << prediction_origin_.x() + terminal_state(kPosX) << ','
        << prediction_origin_.y() + terminal_state(kPosY) << ','
        << prediction_origin_.z() + terminal_state(kPosZ) << ','
        << terminal_state(kOriW) << ','
        << terminal_state(kOriX) << ','
        << terminal_state(kOriY) << ','
        << terminal_state(kOriZ) << ','
        << terminal_state(kVelX) << ','
        << terminal_state(kVelY) << ','
        << terminal_state(kVelZ) << ','
        << terminal_state(kFocusW) << ','
        << terminal_state(kFocusX) << ','
        << terminal_state(kFocusY) << ','
        << terminal_state(kFocusZ) << ','
        << terminal_state(kRange) << ','
        << terminal_state(kInterceptSpeed) << ','
        << terminal_img_err_u << ','
        << terminal_img_err_v << ','
        << current_roll * kRadToDeg << ','
        << predicted_max_abs_roll * kRadToDeg << ','
        << min_roll_constraint_margin << '\n';
    debug_log_file_.flush();
  }
  // ---------------------------------------------------------------------------
  return command;
}

// ============================================================================
// 5. Hover Command （悬停指令）
// ============================================================================
template <typename T>
quadrotor_common::ControlCommand So3MpcController<T>::makeHoverCommand(
    const ros::Time& time,
    const Eigen::Quaternion<T>& orientation) const {
  quadrotor_common::ControlCommand command;
  command.zero();
  command.timestamp = time;
  command.expected_execution_time = time;
  command.armed = true;
  command.control_mode = quadrotor_common::ControlMode::ATTITUDE;
  command.collective_thrust = params_.hover_acceleration_;
  command.bodyrates.setZero();
  Eigen::Quaternion<T> q = orientation;
  if (q.norm() > T(1e-6)) {
    q.normalize();
  } else {
    q = Eigen::Quaternion<T>::Identity();
  }
  command.orientation = q.template cast<double>();
  return command;
}

template <typename T>
quadrotor_common::ControlCommand
So3MpcController<T>::makeZeroControlRecoveryCommand(
    const ros::Time& time) const {
  quadrotor_common::ControlCommand command;
  command.zero();
  command.timestamp = time;
  command.expected_execution_time = time;
  command.armed = true;
  command.control_mode = quadrotor_common::ControlMode::BODY_RATES;
  command.bodyrates.setZero();
  command.collective_thrust = params_.hover_acceleration_;
  return command;
}

// ============================================================================
// 6. State Estimate Setup （状态估计填充）
// ============================================================================
template <typename T>
void So3MpcController<T>::setStateEstimate(
    const quadrotor_common::QuadStateEstimate& state_estimate,
    const Eigen::Matrix<T, 3, 1>& target_position,
    const Eigen::Matrix<T, 3, 1>& target_velocity,
    const Eigen::Quaternion<T>& q_image,
    bool has_target_state) {
  // Rebuild all 18 states on every solve.  In particular, the two generated
  // slack-accumulator states are not measured and must start from zero rather
  // than inherit any value from an earlier engagement.
  est_state_.setZero();
  prediction_origin_ = state_estimate.position.template cast<T>();
  est_state_(kPosX) = T(0.0);
  est_state_(kPosY) = T(0.0);
  est_state_(kPosZ) = T(0.0);

  // q and -q encode the same attitude but lead to different nonlinear solver
  // linearization points.  PX4 may switch quaternion hemisphere between
  // trials, so normalize to a stable w >= 0 representation before solving.
  Eigen::Quaternion<T> q_world_body =
      state_estimate.orientation.template cast<T>();
  if (q_world_body.norm() > T(1e-6)) {
    q_world_body.normalize();
  } else {
    q_world_body = Eigen::Quaternion<T>::Identity();
  }
  if (q_world_body.w() < T(0.0)) {
    q_world_body.coeffs() *= T(-1.0);
  }
  est_state_(kOriW) = q_world_body.w();
  est_state_(kOriX) = q_world_body.x();
  est_state_(kOriY) = q_world_body.y();
  est_state_(kOriZ) = q_world_body.z();
  est_state_(kVelX) = state_estimate.velocity.x();
  est_state_(kVelY) = state_estimate.velocity.y();
  est_state_(kVelZ) = state_estimate.velocity.z();

  Eigen::Quaternion<T> q_image_normalized = q_image;
  if (q_image_normalized.norm() > T(1e-6)) {
    q_image_normalized.normalize();
  } else {
    q_image_normalized = Eigen::Quaternion<T>::Identity();
  }
  if (q_image_normalized.w() < T(0.0)) {
    q_image_normalized.coeffs() *= T(-1.0);
  }
  // Freeze the speed schedule from the current observed LOS for this solve.
  // Predicted LOS states can no longer lower their own speed reference.
  mpc_wrapper_.setInterceptSpeedReference(q_image_normalized);
  if (initialize_intercept_speed_from_state_ && has_target_state) {
    const Eigen::Matrix<T, 3, 1> target_position_delta =
        target_position - prediction_origin_;
    if (target_position_delta.norm() > T(1e-6)) {
      const Eigen::Matrix<T, 3, 1> line_of_sight =
          target_position_delta.normalized();
      const Eigen::Matrix<T, 3, 1> uav_velocity =
          state_estimate.velocity.template cast<T>();
      const T closing_speed =
          (uav_velocity - target_velocity).dot(line_of_sight);
      const T initial_speed_limit =
          std::min(params_.intercept_speed_,
                   params_.max_initial_intercept_speed_);
      current_intercept_speed_ =
          std::max<T>(T(0.0), std::min(initial_speed_limit, closing_speed));
      ROS_INFO(
          "SO3 MPC: initialized intercept speed from current closing speed: %.3f m/s (raw %.3f, limit %.3f).",
          static_cast<double>(current_intercept_speed_),
          static_cast<double>(closing_speed),
          static_cast<double>(initial_speed_limit));
    }
    initialize_intercept_speed_from_state_ = false;
  }

  current_intercept_speed_ =
      std::max<T>(T(0.0),
                  std::min(params_.intercept_speed_,
                           current_intercept_speed_));
  est_state_(kInterceptSpeed) = current_intercept_speed_;
  setFocusStateFromQImage(est_state_, q_image_normalized);

  if (has_target_state) {
    const Eigen::Matrix<T, 3, 1> uav_position =
        est_state_.template segment<3>(kPosX);
    const Eigen::Matrix<T, 3, 1> target_position_delta =
        target_position - prediction_origin_;
    est_state_(kRange) =
        std::max<T>(target_position_delta.norm(), params_.min_solver_range_);

    mpc_wrapper_.setTargetVelocity(target_velocity);

    Eigen::Quaternion<T> q_body_camera = params_.q_B_C_;
    if (q_body_camera.norm() > T(1e-6)) {
      q_body_camera.normalize();
    } else {
      q_body_camera = Eigen::Quaternion<T>::Identity();
    }
    Eigen::Matrix<T, 3, 1> line_of_sight =
        q_world_body * (q_body_camera * (q_image_normalized * Eigen::Matrix<T, 3, 1>::UnitZ()));
    if (line_of_sight.norm() > T(1e-6)) {
      line_of_sight.normalize();
    } else {
      line_of_sight = Eigen::Matrix<T, 3, 1>::UnitX();
    }

    mpc_wrapper_.setTerminalLine(uav_position, line_of_sight);
  } else {
    est_state_(kRange) =
        std::max<T>(params_.nominal_target_range_, params_.min_solver_range_);
    mpc_wrapper_.setTargetVelocity(Eigen::Matrix<T, 3, 1>::Zero());

    Eigen::Quaternion<T> q_body_camera = params_.q_B_C_;
    if (q_body_camera.norm() > T(1e-6)) {
      q_body_camera.normalize();
    } else {
      q_body_camera = Eigen::Quaternion<T>::Identity();
    }
    Eigen::Matrix<T, 3, 1> line_of_sight =
        q_world_body *
        (q_body_camera *
         (q_image_normalized * Eigen::Matrix<T, 3, 1>::UnitZ()));
    if (line_of_sight.norm() > T(1e-6)) {
      line_of_sight.normalize();
    } else {
      line_of_sight = Eigen::Matrix<T, 3, 1>::UnitX();
    }
    mpc_wrapper_.setTerminalLine(est_state_.template segment<3>(kPosX),
                                 line_of_sight);
  }
}

template <typename T>
void So3MpcController<T>::setFocusStateFromQImage(
    Eigen::Ref<Eigen::Matrix<T, kStateSize, 1>> state,
    const Eigen::Quaternion<T>& q_image) const {
  state(kFocusW) = q_image.w();
  state(kFocusX) = q_image.x();
  state(kFocusY) = q_image.y();
  state(kFocusZ) = q_image.z();
  state(kRange) = std::max<T>(state(kRange), params_.min_solver_range_);
}

// ============================================================================
// 7. Control Command Conversion （控制指令转换）
// ============================================================================
template <typename T>
quadrotor_common::ControlCommand So3MpcController<T>::updateControlCommand(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
    const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
    const ros::Time& time) {
  // run() validates the complete predicted state and input matrices before
  // selecting these columns, so repeating the same finite check here added no
  // independent failure coverage.
  const T command_thrust = std::max(
      params_.min_thrust_, std::min(params_.max_thrust_, input(kThrust)));
  T command_rate_x = std::max(
      -params_.max_bodyrate_x_,
      std::min(params_.max_bodyrate_x_, input(kRateX)));
  const T command_rate_y = std::max(
      -params_.max_bodyrate_y_,
      std::min(params_.max_bodyrate_y_, input(kRateY)));
  const T command_rate_z = std::max(
      -params_.max_bodyrate_z_,
      std::min(params_.max_bodyrate_z_, input(kRateZ)));

  // The plant can overshoot the predicted roll because PX4 body-rate
  // tracking and state-estimate latency are not states in the NMPC model.
  // Shape only the outward Euler-roll rate in a fixed 12-degree boundary
  // layer. This keeps max_roll_angle_deg as the sole user-facing safety
  // setting while preserving pitch authority.
  Eigen::Quaternion<T> q_current(est_state_(kOriW), est_state_(kOriX),
                                 est_state_(kOriY), est_state_(kOriZ));
  if (q_current.norm() > T(1e-6)) {
    q_current.normalize();
    const T qw = q_current.w();
    const T qx = q_current.x();
    const T qy = q_current.y();
    const T qz = q_current.z();
    const T roll = std::atan2(T(2) * (qw * qx + qy * qz),
                              T(1) - T(2) * (qx * qx + qy * qy));
    const T sin_pitch = std::max(
        T(-0.98), std::min(T(0.98), T(2) * (qw * qy - qz * qx)));
    const T tan_pitch =
        sin_pitch / std::sqrt(std::max(T(1e-4), T(1) - sin_pitch * sin_pitch));
    const T coupling = std::sin(roll) * tan_pitch * command_rate_y +
                       std::cos(roll) * tan_pitch * command_rate_z;
    const T roll_rate = command_rate_x + coupling;
    const T roll_sign = roll >= T(0) ? T(1) : T(-1);
    const T outward_rate = roll_sign * roll_rate;
    constexpr T kRollBoundaryLayer =
        T(12.0 * 3.14159265358979323846 / 180.0);
    const T remaining = params_.max_roll_angle_rad_ - std::abs(roll);
    T allowed_outward_rate = params_.max_bodyrate_x_;
    if (remaining < kRollBoundaryLayer) {
      allowed_outward_rate = params_.max_bodyrate_x_ *
          std::max(T(0), remaining / kRollBoundaryLayer);
    }
    if (remaining <= T(0)) {
      // Once outside the configured boundary, require a bounded inward roll
      // rate instead of merely preventing further outward motion.
      allowed_outward_rate = -std::min(
          params_.max_bodyrate_x_,
          T(0.25) + T(2.0) * std::abs(remaining));
    }
    if (outward_rate > allowed_outward_rate) {
      const T guarded_roll_rate = roll_sign * allowed_outward_rate;
      command_rate_x = std::max(
          -params_.max_bodyrate_x_,
          std::min(params_.max_bodyrate_x_, guarded_roll_rate - coupling));
      ROS_WARN_THROTTLE(
          0.5,
          "SO3 MPC: roll-rate safety layer active: roll=%.1f deg, p %.2f -> %.2f rad/s.",
          static_cast<double>(roll * T(180.0 / 3.14159265358979323846)),
          static_cast<double>(input(kRateX)),
          static_cast<double>(command_rate_x));
    }
  }

  quadrotor_common::ControlCommand command;
  command.timestamp = time;
  command.armed = true;
  command.control_mode = quadrotor_common::ControlMode::BODY_RATES;
  command.expected_execution_time = time;
  command.collective_thrust = command_thrust;
  command.bodyrates.x() = command_rate_x;
  command.bodyrates.y() = command_rate_y;
  command.bodyrates.z() = command_rate_z;
  command.orientation.w() = state(kOriW);
  command.orientation.x() = state(kOriX);
  command.orientation.y() = state(kOriY);
  command.orientation.z() = state(kOriZ);
  return command;
}

// ============================================================================
// 8. Prediction Publishing （预测轨迹发布）
// ============================================================================
template <typename T>
void So3MpcController<T>::publishPrediction(
    const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
    const ros::Time& time) {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = time;
  path_msg.header.frame_id = "map";
  const T dt = mpc_wrapper_.getTimestep();

  for (int i = 0; i <= kSamples; ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = time + ros::Duration(i * dt);
    pose.header.seq = i;
    pose.pose.position.x = prediction_origin_.x() + states(kPosX, i);
    pose.pose.position.y = prediction_origin_.y() + states(kPosY, i);
    pose.pose.position.z = prediction_origin_.z() + states(kPosZ, i);
    pose.pose.orientation.w = states(kOriW, i);
    pose.pose.orientation.x = states(kOriX, i);
    pose.pose.orientation.y = states(kOriY, i);
    pose.pose.orientation.z = states(kOriZ, i);
    path_msg.poses.push_back(pose);
  }

  pub_predicted_trajectory_.publish(path_msg);
}

template <typename T>
void So3MpcController<T>::publishFailure(bool failed) const {
  if (!failure_pub_) {
    return;
  }
  std_msgs::Bool msg;
  msg.data = failed;
  failure_pub_.publish(msg);
}

// ============================================================================
// 9. Parameter Management （参数下发）
// ============================================================================
template <typename T>
void So3MpcController<T>::setNewParams(So3MpcParams<T>& params) {
  mpc_wrapper_.setCosts(params.Q_, params.R_, params.Q_terminal_line_,
                        params.state_cost_exponential_,
                        params.input_cost_exponential_);
  mpc_wrapper_.setLimits(params.min_thrust_, params.max_thrust_,
                         params.max_bodyrate_x_, params.max_bodyrate_y_,
                         params.max_bodyrate_z_);
  // Keep the nonlinear program feasible when the measured plant briefly
  // overshoots the command boundary. The tighter user limit is enforced by
  // the output safety layer above; this envelope still excludes inversion and
  // the alternate Euler branch.
  constexpr T kRollFeasibilityMargin =
      T(10.0 * 3.14159265358979323846 / 180.0);
  constexpr T kMaxFeasibleRoll =
      T(89.0 * 3.14159265358979323846 / 180.0);
  const T solver_roll_envelope = std::min(
      kMaxFeasibleRoll,
      params.max_roll_angle_rad_ + kRollFeasibilityMargin);
  mpc_wrapper_.setMaxRollAngle(solver_roll_envelope);
  mpc_wrapper_.setInterceptSpeedLimits(params.intercept_speed_,
                                       params.min_intercept_speed_,
                                       params.los_speed_schedule_kh_,
                                       params.los_speed_schedule_kv_,
                                       params.max_delta_intercept_speed_);
  mpc_wrapper_.setImagePlaneParameters(params.image_center_u_,
                                       params.image_center_v_,
                                       params.image_radius_u_,
                                       params.image_radius_v_);
  mpc_wrapper_.setCameraParameters(params.q_B_C_);
  params.changed_ = false;
}

// ============================================================================
// 10. Template Instantiation （模板实例化）
// ============================================================================
template class So3MpcController<float>;
template class So3MpcController<double>;

}  // namespace isb_nmpc
