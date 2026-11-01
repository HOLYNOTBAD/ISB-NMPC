#include "plan_manage/interception_fsm.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace plan_manage {

namespace {

constexpr double kInterceptionFsmBasePeriodSec = 0.005;  // 200 Hz base timer.（200 Hz基准定时器）
constexpr int kInterceptionStatePublishDivider = 2;      // 100 Hz state/POI publish.（100 Hz状态发布）
constexpr int kInterceptionExecDivider = 10;             // 20 Hz FSM logic.（20 Hz FSM逻辑执行）

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double pointDistance(const geometry_msgs::Point& lhs,
                     const geometry_msgs::Point& rhs) {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

}  // namespace

const char* InterceptionFSM::stateName(FSMExecState state) const {
  switch (state) {
    case IDLE:
      return "IDLE";
    case WAIT_TARGET:
      return "WAIT_TARGET";
    case EXEC_INTERCEPTION:
      return "EXEC_INTERCEPTION";
    case HOLD:
      return "HOLD";
    case FINISH_HOLD:
      return "FINISH_HOLD";
    case RETURN_TO_START:
      return "RETURN_TO_START";
  }
  return "UNKNOWN";
}

void InterceptionFSM::init(ros::NodeHandle& nh) {
  nh_ = nh;
  exec_state_ = IDLE;
  has_local_pose_ = false;
  has_local_velocity_ = false;
  has_object_target_ = false;
  intercepted_ = false;
  trajectory_fsm_.reset();

  loadCommonParams(nh_, "interception");
  std::string start_mode;
  nh_.param<std::string>("interception/start_mode", start_mode, "ground_takeoff");
  if (nh_.hasParam("interception/manual_offboard_takeover")) {
    ROS_FATAL("[fsm] manual_offboard_takeover was removed; configure start_mode instead.");
    ros::shutdown();
    return;
  }
  if (start_mode == "ground_takeoff") {
    start_mode_ = StartMode::GROUND_TAKEOFF;
  } else if (start_mode == "airborne_takeover") {
    start_mode_ = StartMode::AIRBORNE_TAKEOVER;
  } else {
    ROS_FATAL("[fsm] invalid start_mode '%s': use ground_takeoff or airborne_takeover.", start_mode.c_str());
    ros::shutdown();
    return;
  }
  px4_com_.deferLocalFrameCapture();
  nh_.param("interception/initialization/settle_sec", initialization_settle_sec_, 0.5);
  nh_.param("interception/initialization/max_speed", initialization_max_speed_, 0.3);
  nh_.param("interception/initialization/max_position_change", initialization_max_position_change_, 0.2);
  double max_yaw_deg;
  nh_.param("interception/initialization/max_yaw_change_deg", max_yaw_deg, 5.0);
  if (!std::isfinite(initialization_settle_sec_) || initialization_settle_sec_ <= 0.0 ||
      !std::isfinite(initialization_max_speed_) || initialization_max_speed_ <= 0.0 ||
      !std::isfinite(initialization_max_position_change_) || initialization_max_position_change_ <= 0.0 ||
      !std::isfinite(max_yaw_deg) || max_yaw_deg <= 0.0 || max_yaw_deg >= 180.0) {
    ROS_FATAL("[fsm] invalid initialization stability thresholds.");
    ros::shutdown();
    return;
  }
  initialization_max_yaw_change_rad_ = max_yaw_deg * M_PI / 180.0;
  nh_.param("interception/offboard_takeover_hold_sec",
            offboard_takeover_hold_sec_, offboard_takeover_hold_sec_);
  if (!std::isfinite(offboard_takeover_hold_sec_) ||
      offboard_takeover_hold_sec_ < 0.0) {
    ROS_FATAL("[fsm] offboard_takeover_hold_sec must be finite and non-negative.");
    ros::shutdown();
    return;
  }
  nh_.param("interception/idle/position_tolerance", position_tolerance_, 0.3);
  nh_.param("interception/return_to_start/restore_start_altitude",
            return_to_start_altitude_, false);
  nh_.param("interception/hold/brake_speed_threshold",
            hold_brake_speed_threshold_, hold_brake_speed_threshold_);
  nh_.param("interception/hold/brake_settle_time",
            hold_brake_settle_time_, hold_brake_settle_time_);
  nh_.param("interception/hold/brake_timeout",
            hold_brake_timeout_, hold_brake_timeout_);
  hold_brake_speed_threshold_ = std::max(0.0, hold_brake_speed_threshold_);
  hold_brake_settle_time_ = std::max(0.0, hold_brake_settle_time_);
  hold_brake_timeout_ = std::max(0.0, hold_brake_timeout_);
  nh_.param("interception/finish_hold/brake_speed_threshold",
            finish_brake_speed_threshold_, finish_brake_speed_threshold_);
  nh_.param("interception/finish_hold/brake_settle_time",
            finish_brake_settle_time_, finish_brake_settle_time_);
  nh_.param("interception/finish_hold/brake_timeout",
            finish_brake_timeout_, finish_brake_timeout_);
  finish_brake_speed_threshold_ =
      std::max(0.0, finish_brake_speed_threshold_);
  finish_brake_settle_time_ = std::max(0.0, finish_brake_settle_time_);
  finish_brake_timeout_ = std::max(0.0, finish_brake_timeout_);
  double max_abs_roll_deg =
      failsafe_max_abs_roll_rad_ * 180.0 / M_PI;
  nh_.param("interception/failsafe/max_abs_roll_deg",
            max_abs_roll_deg, max_abs_roll_deg);
  if (!std::isfinite(max_abs_roll_deg) || max_abs_roll_deg <= 0.0 ||
      max_abs_roll_deg >= 180.0) {
    ROS_FATAL("[fsm] invalid interception failsafe parameters.");
    ros::shutdown();
    return;
  }
  failsafe_max_abs_roll_rad_ = max_abs_roll_deg * M_PI / 180.0;
  nh_.param("interception/target_state_estimate/timeout",
            target_state_estimate_timeout_, target_state_estimate_timeout_);
  nh_.param("manager/has_target_pos", has_target_pos_, has_target_pos_);
  nh_.param("estimator/q_image_timeout", q_image_timeout_, q_image_timeout_);
  nh_.param<std::string>("estimator/target_in_main_view_topic",
                         target_in_main_view_topic_,
                         target_in_main_view_topic_);
  nh_.param<std::string>("interception/exec/detection_mode_topic",
                         detection_mode_topic_, detection_mode_topic_);
  nh_.param("interception/exec/terminal_lost_timeout",
            terminal_lost_timeout_, terminal_lost_timeout_);
  nh_.param("interception/exec/terminal_mode_memory",
            terminal_mode_memory_, terminal_mode_memory_);
  terminal_lost_timeout_ = std::max(0.0, terminal_lost_timeout_);
  terminal_mode_memory_ = std::max(0.0, terminal_mode_memory_);
  nh_.param<std::string>("interception/exec/motion_trigger_topic",
                         motion_trigger_topic_, motion_trigger_topic_);
  nh_.param<std::string>("interception/exec/intercepted_topic",
                         intercepted_topic_, intercepted_topic_);
  nh_.param<std::string>("interception/exec/nmpc_failure_topic",
                         nmpc_failure_topic_, nmpc_failure_topic_);

  start_state_.header.frame_id = idle_frame_id_;
  start_state_.child_frame_id = "base_link";
  start_state_.pose.pose.position.x = start_x_;
  start_state_.pose.pose.position.y = start_y_;
  start_state_.pose.pose.position.z = start_z_;
  start_state_.pose.pose.orientation.w = 1.0;

  uav_state_.header.frame_id = idle_frame_id_;
  uav_state_.child_frame_id = "base_link";

  initCommonRosInterfaces(nh_);
  target_state_estimate_sub_ =
      nh_.subscribe(target_state_estimate_topic_, 10,
                    &InterceptionFSM::targetStateEstimateCallback, this);
  target_in_main_view_sub_ =
      nh_.subscribe(target_in_main_view_topic_, 10,
                    &InterceptionFSM::targetInMainViewCallback, this);
  detection_mode_sub_ =
      nh_.subscribe(detection_mode_topic_, 10,
                    &InterceptionFSM::detectionModeCallback, this);
  intercepted_sub_ =
      nh_.subscribe(intercepted_topic_, 10,
                    &InterceptionFSM::interceptedCallback, this);
  nmpc_failure_sub_ =
      nh_.subscribe(nmpc_failure_topic_, 10,
                    &InterceptionFSM::nmpcFailureCallback, this);
  balloon_motion_trigger_pub_ =
      nh_.advertise<std_msgs::Bool>(motion_trigger_topic_, 1, true);
  if (!airborneTakeover()) {
    reset_trial_srv_ =
        nh_.advertiseService("/fsm/reset_trial",
                             &InterceptionFSM::resetTrialCallback, this);
  }
  exec_timer_ = nh_.createTimer(ros::Duration(kInterceptionFsmBasePeriodSec),
                                &InterceptionFSM::execFSMCallback, this);
  publishBalloonMotionTrigger(false);

  startInputThread(requireEnterConfirmation() && !airborneTakeover());

  ROS_INFO(
      "[fsm] InterceptionFSM initialized. start=[%.2f %.2f %.2f], "
      "start_mode=%s, takeover_hold=%.2f s, "
      "require_enter_confirmation=%s, has_target_pos=%s, intercepted_topic=%s, nmpc_failure_topic=%s, "
      "detection_mode_topic=%s, terminal_lost_timeout=%.3f s, terminal_mode_memory=%.3f s, "
      "hold_brake=[speed<%.2f m/s for %.2f s, timeout %.2f s], "
      "finish_brake=[speed<%.2f m/s for %.2f s, timeout %.2f s], "
      "failsafe=[|roll|>%.1f deg; latched failures remain in STABILIZED for pilot intervention].",
      start_state_.pose.pose.position.x, start_state_.pose.pose.position.y,
      start_state_.pose.pose.position.z,
      start_mode.c_str(),
      offboard_takeover_hold_sec_,
      requireEnterConfirmation() ? "true" : "false",
      has_target_pos_ ? "true" : "false",
      intercepted_topic_.c_str(), nmpc_failure_topic_.c_str(),
      detection_mode_topic_.c_str(), terminal_lost_timeout_,
      terminal_mode_memory_, hold_brake_speed_threshold_,
      hold_brake_settle_time_, hold_brake_timeout_, finish_brake_speed_threshold_,
      finish_brake_settle_time_, finish_brake_timeout_,
      max_abs_roll_deg);
}

void InterceptionFSM::targetInMainViewCallback(
    const std_msgs::Bool::ConstPtr& msg) {
  target_in_main_view_ = msg->data;
  if (msg->data) {
    last_target_in_view_time_ = ros::Time::now();
  }
}

void InterceptionFSM::detectionModeCallback(
    const std_msgs::String::ConstPtr& msg) {
  if (exec_state_ == EXEC_INTERCEPTION && msg->data == "TRACK_LR") {
    last_track_lr_time_ = ros::Time::now();
  }
}

void InterceptionFSM::interceptedCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (exec_state_ != EXEC_INTERCEPTION) {
    return;
  }
  if (msg->data && !intercepted_) {
    ROS_INFO("[fsm] InterceptionFSM received intercepted=true.");
  }
  intercepted_ = intercepted_ || msg->data;
}

void InterceptionFSM::nmpcFailureCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data) {
    nmpc_failure_received_ = true;
  }
}

bool InterceptionFSM::normalizedAttitude(
    geometry_msgs::Quaternion* attitude) const {
  if (attitude == nullptr) {
    return false;
  }
  *attitude = uav_state_.pose.pose.orientation;
  const double norm_squared =
      attitude->w * attitude->w + attitude->x * attitude->x +
      attitude->y * attitude->y + attitude->z * attitude->z;
  if (!std::isfinite(norm_squared) || norm_squared <= 1e-12) {
    return false;
  }
  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  attitude->w *= inverse_norm;
  attitude->x *= inverse_norm;
  attitude->y *= inverse_norm;
  attitude->z *= inverse_norm;
  return true;
}

bool InterceptionFSM::localStateFinite() const {
  if (!has_local_pose_ || !has_local_velocity_) {
    return false;
  }
  const auto& position = uav_state_.pose.pose.position;
  const auto& linear_velocity = uav_state_.twist.twist.linear;
  const auto& angular_velocity = uav_state_.twist.twist.angular;
  geometry_msgs::Quaternion attitude;
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z) &&
         std::isfinite(linear_velocity.x) &&
         std::isfinite(linear_velocity.y) &&
         std::isfinite(linear_velocity.z) &&
         std::isfinite(angular_velocity.x) &&
         std::isfinite(angular_velocity.y) &&
         std::isfinite(angular_velocity.z) && normalizedAttitude(&attitude);
}

double InterceptionFSM::rollAngleRad(
    const geometry_msgs::Quaternion& q) const {
  const double roll_s = 2.0 * (q.w * q.x + q.y * q.z);
  const double roll_c = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  return std::atan2(roll_s, roll_c);
}

void InterceptionFSM::beginStabilizedFailsafe(const char* reason) {
  if (failsafe_latched_) {
    return;
  }
  const std::string interrupted_state = stateName(exec_state_);
  failsafe_latched_ = true;
  failsafe_offboard_exit_observed_ = false;
  failsafe_reason_ = reason != nullptr ? reason : "unknown state failure";
  publishBalloonMotionTrigger(false);
  publishEmptyHorizonTraj();
  setExecPermissionGranted(false);
  px4_com_.resetHoverSetpoint();
  px4_com_.resetIdleVelocitySetpoint();
  // FINISH_HOLD is an existing mission state that makes every downstream
  // controller leave EXEC_INTERCEPTION. The latched failsafe below owns the
  // callback, so normal FINISH_HOLD position control is never executed.
  exec_state_ = FINISH_HOLD;
  publishFsmState(stateName(exec_state_));
  px4_com_.requestStabilizedMode();
  ROS_ERROR(
      "[fsm] failsafe latched (%s) while in %s: PX4 STABILIZED requested; "
      "remaining latched for pilot intervention; the FSM will not disarm.",
      failsafe_reason_.c_str(), interrupted_state.c_str());
}

void InterceptionFSM::runLatchedFailsafe() {
  const bool has_state = px4_com_.mavrosStateFresh();
  const bool stabilized =
      has_state && px4_com_.mavrosState().mode == "STABILIZED";
  const bool offboard =
      has_state && px4_com_.mavrosState().mode == "OFFBOARD";
  if (has_state && !offboard) {
    failsafe_offboard_exit_observed_ = true;
  }

  const auto& position = uav_state_.pose.pose.position;
  const bool ready_for_pilot_offboard =
      airborneTakeover() && has_state && px4_com_.mavrosState().armed &&
      px4_com_.localPositionEstimateReliable() && localStateFinite();

  // PX4 needs a current setpoint stream before it accepts a new OFFBOARD
  // entry. While STABILIZED, stream the measured pose only after recovery.
  if (ready_for_pilot_offboard && !offboard) {
    px4_com_.publishPositionSetpoint(
        position, yawFromQuaternion(uav_state_.pose.pose.orientation));
  }

  // A fresh pilot OFFBOARD entry after leaving it is the real-flight reset
  // gesture. Never release the latch while the position estimate is invalid.
  if (ready_for_pilot_offboard && failsafe_offboard_exit_observed_ &&
      offboard) {
    resetFailsafe();
    clearTrialState();
    px4_com_.resetConfirmationGate();
    exec_state_ = IDLE;
    trial_frame_locked_ = false;
    resetInitializationWindow();
    publishFsmState(stateName(exec_state_));
    px4_com_.publishPositionSetpoint(
        position, yawFromQuaternion(uav_state_.pose.pose.orientation));
    ROS_WARN("[fsm] pilot re-entered OFFBOARD with reliable position; "
             "failsafe cleared and FSM returned to IDLE for a new trial.");
    return;
  }

  if (!stabilized) {
    px4_com_.requestStabilizedMode();
  }
  ROS_ERROR_THROTTLE(
      1.0,
      "[fsm] failsafe (%s) latched; holding STABILIZED. Re-enter OFFBOARD "
      "after position recovers to start a new trial. No disarm command will be sent.",
      failsafe_reason_.c_str());
}

bool InterceptionFSM::runFailsafe() {
  if (failsafe_latched_) {
    runLatchedFailsafe();
    return true;
  }

  const bool flight_active = exec_state_ != IDLE ||
      (trial_frame_locked_ && px4_com_.offboardEntryRequested());
  if (flight_active &&
      !px4_com_.mavrosStateFresh()) {
    beginStabilizedFailsafe("MAVROS state missing or stale");
    return true;
  }
  const bool armed =
      px4_com_.mavrosStateFresh() && px4_com_.mavrosState().armed;
  if (armed && flight_active &&
      !px4_com_.localPositionEstimateReliable()) {
    beginStabilizedFailsafe("local position estimate invalid or stale");
    return true;
  }

  if (armed && flight_active && has_local_pose_ &&
      has_local_velocity_) {
    geometry_msgs::Quaternion attitude;
    if (!localStateFinite() || !normalizedAttitude(&attitude)) {
      beginStabilizedFailsafe("non-finite state or invalid quaternion");
      return true;
    }
    const double roll = rollAngleRad(attitude);
    if (std::abs(roll) > failsafe_max_abs_roll_rad_ &&
        exec_state_ != IDLE) {
      ROS_ERROR(
          "[fsm] failsafe roll limit exceeded: roll=%.1f deg, limit=%.1f "
          "deg; requesting STABILIZED.",
          roll * 180.0 / M_PI,
          failsafe_max_abs_roll_rad_ * 180.0 / M_PI);
      beginStabilizedFailsafe("measured roll limit exceeded");
      return true;
    }
  }

  if (nmpc_failure_received_) {
    nmpc_failure_received_ = false;
    if (exec_state_ == EXEC_INTERCEPTION) {
      ROS_ERROR(
          "[fsm] NMPC failure=true received; switching to FINISH_HOLD.");
      beginFinishHold("NMPC failure");
      return true;
    }
  }
  return false;
}

void InterceptionFSM::resetFailsafe() {
  nmpc_failure_received_ = false;
  failsafe_latched_ = false;
  failsafe_offboard_exit_observed_ = false;
  failsafe_reason_.clear();
}

void InterceptionFSM::clearTrialState() {
  intercepted_ = false;
  nmpc_failure_received_ = false;
  has_object_target_ = false;
  target_in_main_view_ = false;
  target_state_ = nav_msgs::Odometry();
  intercept_state_.clear();
  last_target_state_estimate_time_ = ros::Time();
  last_target_in_view_time_ = ros::Time();
  last_track_lr_time_ = ros::Time();
  hold_brake_start_time_ = ros::Time();
  hold_brake_below_speed_since_ = ros::Time();
  hold_phase_ = HOLD_BRAKE;
  finish_brake_start_time_ = ros::Time();
  finish_brake_below_speed_since_ = ros::Time();
  finish_hold_phase_ = FINISH_BRAKE;
  offboard_takeover_pending_ = false;
  offboard_takeover_start_time_ = ros::Time();
  return_cruise_position_ = geometry_msgs::Point();
  hold_position_ = geometry_msgs::Point();
  hold_yaw_ = 0.0;
  trajectory_fsm_.reset();
  publishEmptyHorizonTraj();
  publishBalloonMotionTrigger(false);
  px4_com_.resetHoverSetpoint();
  px4_com_.resetIdleVelocitySetpoint();
}

bool InterceptionFSM::resetTrialCallback(std_srvs::Trigger::Request&,
                                         std_srvs::Trigger::Response& resp) {
  resetFailsafe();
  clearTrialState();
  setExecPermissionGranted(!requireEnterConfirmation());
  px4_com_.resetConfirmationGate();

  if (!has_local_pose_) {
    exec_state_ = IDLE;
    resp.message = "FSM reset to IDLE; waiting for local pose.";
  } else if (!px4_com_.hasMavrosState() || !px4_com_.mavrosState().armed) {
    exec_state_ = IDLE;
    resp.message = "FSM reset to IDLE; vehicle is not armed yet.";
  } else if (airborneTakeover() &&
             px4_com_.mavrosState().mode != "OFFBOARD") {
    exec_state_ = IDLE;
    resp.message = "FSM reset to IDLE; waiting for pilot OFFBOARD entry.";
  } else {
    beginReturnToStart();
    resp.message = "FSM reset to RETURN_TO_START.";
  }
  resp.success = true;
  ROS_INFO_STREAM("[fsm] reset_trial: " << resp.message);
  return true;
}

void InterceptionFSM::onEnterConfirmation() {
  // The terminal thread only records the event. Frame mutation belongs to
  // the ROS callback thread and occurs only after initialization validation.
  last_enter_time_.store(ros::Time::now().toSec());
}

void InterceptionFSM::resetInitializationWindow() {
  initialization_stable_since_ = ros::Time();
  initialization_ready_ = false;
}

bool InterceptionFSM::initializationReady() {
  geometry_msgs::Quaternion attitude;
  const bool basic = px4_com_.mavrosStateFresh() &&
      px4_com_.mavrosState().connected && px4_com_.localPositionEstimateReliable() &&
      localStateFinite() && normalizedAttitude(&attitude);
  const auto& velocity = uav_state_.twist.twist.linear;
  const double speed = std::hypot(velocity.x, velocity.y, velocity.z);
  if (!basic || speed > initialization_max_speed_) {
    resetInitializationWindow();
    return false;
  }
  const auto& position = uav_state_.pose.pose.position;
  const double yaw = yawFromQuaternion(attitude);
  const double yaw_change = std::abs(std::atan2(std::sin(yaw - initialization_yaw_),
                                               std::cos(yaw - initialization_yaw_)));
  if (initialization_stable_since_.isZero() ||
      pointDistance(position, initialization_anchor_) > initialization_max_position_change_ ||
      yaw_change > initialization_max_yaw_change_rad_) {
    initialization_stable_since_ = ros::Time::now();
    initialization_anchor_ = position;
    initialization_yaw_ = yaw;
  }
  initialization_ready_ = (ros::Time::now() - initialization_stable_since_).toSec() >=
                          initialization_settle_sec_;
  return initialization_ready_;
}

void InterceptionFSM::beginReturnToStart() {
  return_cruise_position_ = start_state_.pose.pose.position;
  if (has_local_pose_ && !return_to_start_altitude_) {
    return_cruise_position_.z = uav_state_.pose.pose.position.z;
  }
  exec_state_ = RETURN_TO_START;

  ROS_INFO(
      "[fsm] RETURN_TO_START initialized: cruise directly to start XY "
      "[%.2f %.2f] at target z=%.2f.",
      return_cruise_position_.x, return_cruise_position_.y,
      return_cruise_position_.z);
}

void InterceptionFSM::targetStateEstimateCallback(
    const nav_msgs::Odometry::ConstPtr& msg) {
  target_state_ = *msg;
  if (target_state_.header.frame_id.empty()) {
    target_state_.header.frame_id = idle_frame_id_;
  }
  if (target_state_.child_frame_id.empty()) {
    target_state_.child_frame_id = "target";
  }
  if (target_state_.pose.pose.orientation.w == 0.0 &&
      target_state_.pose.pose.orientation.x == 0.0 &&
      target_state_.pose.pose.orientation.y == 0.0 &&
      target_state_.pose.pose.orientation.z == 0.0) {
    target_state_.pose.pose.orientation.w = 1.0;
  }

  intercept_state_.clear();
  intercept_state_.push_back(target_state_);
  has_object_target_ = true;
  last_target_state_estimate_time_ = ros::Time::now();
}

double InterceptionFSM::distanceToFirstTarget() const {
  if (!has_local_pose_ || intercept_state_.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  return pointDistance(uav_state_.pose.pose.position,
                       intercept_state_.front().pose.pose.position);
}

bool InterceptionFSM::targetAvailable(bool allow_visual_grace,
                                      double visual_timeout) const {
  if (has_target_pos_) {
    return has_object_target_ && !intercept_state_.empty();
  }
  if (last_target_in_view_time_.isZero()) {
    return false;
  }
  const bool recently_seen =
      (ros::Time::now() - last_target_in_view_time_).toSec() <=
      (visual_timeout >= 0.0 ? visual_timeout : q_image_timeout_);
  return recently_seen && (target_in_main_view_ || allow_visual_grace);
}

void InterceptionFSM::publishBalloonMotionTrigger(bool trigger) {
  std_msgs::Bool msg;
  msg.data = trigger;
  balloon_motion_trigger_pub_.publish(msg);
  if (trigger != last_balloon_motion_trigger_) {
    ROS_INFO("[fsm] InterceptionFSM balloon motion trigger -> %s.",
             trigger ? "true" : "false");
    last_balloon_motion_trigger_ = trigger;
  }
}

void InterceptionFSM::beginHold(const char* reason) {
  publishBalloonMotionTrigger(false);
  publishEmptyHorizonTraj();
  setExecPermissionGranted(false);

  hold_phase_ = HOLD_BRAKE;
  hold_brake_start_time_ = ros::Time::now();
  hold_brake_below_speed_since_ = ros::Time();
  px4_com_.resetHoverSetpoint();
  px4_com_.resetIdleVelocitySetpoint();
  exec_state_ = HOLD;

  // Transfer control ownership immediately. The zero-velocity command and the
  // HOLD state are published in the same callback, so the attitude controller
  // cannot leave a low-thrust or highly tilted command active until the next
  // 20 Hz FSM execution slot.
  publishFsmState(stateName(exec_state_));
  geometry_msgs::Vector3 zero_velocity;
  px4_com_.publishVelSetpoint(zero_velocity);
  ROS_WARN(
      "[fsm] HOLD braking started (%s): commanding zero velocity; "
      "threshold %.2f m/s for %.2f s.",
      reason, hold_brake_speed_threshold_, hold_brake_settle_time_);
}

void InterceptionFSM::runHold() {
  if (hold_phase_ == HOLD_BRAKE) {
    geometry_msgs::Vector3 zero_velocity;
    px4_com_.publishVelSetpoint(zero_velocity);

    const ros::Time now = ros::Time::now();
    const double speed = has_local_velocity_
                             ? std::hypot(
                                   std::hypot(uav_state_.twist.twist.linear.x,
                                              uav_state_.twist.twist.linear.y),
                                   uav_state_.twist.twist.linear.z)
                             : std::numeric_limits<double>::infinity();
    if (speed <= hold_brake_speed_threshold_) {
      if (hold_brake_below_speed_since_.isZero()) {
        hold_brake_below_speed_since_ = now;
      }
    } else {
      hold_brake_below_speed_since_ = ros::Time();
    }

    const bool settled = !hold_brake_below_speed_since_.isZero() &&
                         (now - hold_brake_below_speed_since_).toSec() >=
                             hold_brake_settle_time_;
    const bool timed_out = !hold_brake_start_time_.isZero() &&
                           hold_brake_timeout_ > 0.0 &&
                           (now - hold_brake_start_time_).toSec() >=
                               hold_brake_timeout_;
    if (!settled) {
      if (timed_out) {
        ROS_ERROR_THROTTLE(1.0,
            "[fsm] HOLD braking exceeded %.2f s at %.2f m/s; "
            "continuing zero-velocity braking until settled or pilot takeover.",
            hold_brake_timeout_, speed);
      } else {
        ROS_INFO_THROTTLE(0.5,
            "[fsm] HOLD braking: speed %.2f m/s, waiting for <= %.2f m/s.",
            speed, hold_brake_speed_threshold_);
      }
      return;
    }
    ROS_INFO("[fsm] HOLD braking complete at %.2f m/s; locking hold position.",
             speed);

    // Lock the actual stopping point instead of pulling the vehicle back to
    // the point where target loss was detected.
    if (has_local_pose_) {
      hold_position_ = uav_state_.pose.pose.position;
      hold_yaw_ = yawFromQuaternion(uav_state_.pose.pose.orientation);
    } else {
      hold_position_ = start_state_.pose.pose.position;
      hold_yaw_ = yawFromQuaternion(start_state_.pose.pose.orientation);
    }
    px4_com_.resetHoverSetpoint();
    px4_com_.resetIdleVelocitySetpoint();
    hold_phase_ = HOLD_POSITION_HOLD;
    setExecPermissionGranted(false);
  }

  px4_com_.publishHoverSetpoint(hold_position_, hold_yaw_);
}

void InterceptionFSM::beginFinishHold(const char* reason) {
  has_object_target_ = false;
  intercept_state_.clear();
  publishBalloonMotionTrigger(false);
  publishEmptyHorizonTraj();
  setExecPermissionGranted(false);

  finish_hold_phase_ = FINISH_BRAKE;
  finish_brake_start_time_ = ros::Time::now();
  finish_brake_below_speed_since_ = ros::Time();
  px4_com_.resetHoverSetpoint();
  px4_com_.resetIdleVelocitySetpoint();
  exec_state_ = FINISH_HOLD;

  // Do not wait for the next 20 Hz FSM execution slot.  Publish both the
  // ownership change and the first zero-velocity command immediately so the
  // attitude controller cannot keep accelerating through the transition.
  publishFsmState(stateName(exec_state_));
  geometry_msgs::Vector3 zero_velocity;
  px4_com_.publishVelSetpoint(zero_velocity);
  ROS_INFO(
      "[fsm] FINISH_HOLD braking started (%s): commanding zero velocity; "
      "threshold %.2f m/s for %.2f s.",
      reason, finish_brake_speed_threshold_, finish_brake_settle_time_);
}

void InterceptionFSM::runFinishHold() {
  if (finish_hold_phase_ == FINISH_BRAKE) {
    geometry_msgs::Vector3 zero_velocity;
    px4_com_.publishVelSetpoint(zero_velocity);

    const ros::Time now = ros::Time::now();
    const double speed = has_local_velocity_
                             ? std::hypot(
                                   std::hypot(uav_state_.twist.twist.linear.x,
                                              uav_state_.twist.twist.linear.y),
                                   uav_state_.twist.twist.linear.z)
                             : std::numeric_limits<double>::infinity();
    if (speed <= finish_brake_speed_threshold_) {
      if (finish_brake_below_speed_since_.isZero()) {
        finish_brake_below_speed_since_ = now;
      }
    } else {
      finish_brake_below_speed_since_ = ros::Time();
    }

    const bool settled = !finish_brake_below_speed_since_.isZero() &&
                         (now - finish_brake_below_speed_since_).toSec() >=
                             finish_brake_settle_time_;
    const bool timed_out = !finish_brake_start_time_.isZero() &&
                           finish_brake_timeout_ > 0.0 &&
                           (now - finish_brake_start_time_).toSec() >=
                               finish_brake_timeout_;
    if (!settled) {
      if (timed_out) {
        ROS_ERROR_THROTTLE(1.0,
            "[fsm] FINISH_HOLD braking exceeded %.2f s at %.2f m/s; "
            "continuing zero-velocity braking until settled or pilot takeover.",
            finish_brake_timeout_, speed);
      } else {
        ROS_INFO_THROTTLE(0.5,
            "[fsm] FINISH_HOLD braking: speed %.2f m/s, waiting "
            "for <= %.2f m/s.",
            speed, finish_brake_speed_threshold_);
      }
      return;
    }
    ROS_INFO("[fsm] FINISH_HOLD braking complete at %.2f m/s; locking hold position.",
             speed);

    // Capture the actual stopping point, not the interception point.  This
    // prevents the position controller from reversing back along the approach
    // path after it has removed the vehicle's momentum.
    px4_com_.resetHoverSetpoint();
    px4_com_.resetIdleVelocitySetpoint();
    finish_hold_phase_ = FINISH_POSITION_HOLD;
    setExecPermissionGranted(false);
  }

  if (has_local_pose_) {
    px4_com_.publishHoverSetpoint(
        uav_state_.pose.pose.position,
        yawFromQuaternion(uav_state_.pose.pose.orientation));
  } else {
    px4_com_.publishHoverSetpoint(
        start_state_.pose.pose.position,
        yawFromQuaternion(start_state_.pose.pose.orientation));
  }
}

void InterceptionFSM::execFSMCallback(const ros::TimerEvent&) {
  static int fsm_num = 0;
  ++fsm_num;

  if (trial_frame_locked_ && px4_com_.mavrosStateFresh()) {
    if (px4_com_.mavrosState().armed) trial_was_armed_ = true;
    else if (trial_was_armed_) {
      clearTrialState();
      resetFailsafe();
      px4_com_.resetConfirmationGate();
      trial_frame_locked_ = false;
      trial_was_armed_ = false;
      resetInitializationWindow();
      exec_state_ = IDLE;
      ROS_INFO("[fsm] vehicle disarmed; next trial requires a new origin lock.");
    }
  }
  if (exec_state_ == IDLE) {
    initializationReady();
    // Existing unattended simulation profiles explicitly disable the Enter
    // gate. They still wait for the same validated initialization window.
    if (!airborneTakeover() && !requireEnterConfirmation() &&
        !trial_frame_locked_ && initialization_ready_) {
      setExecPermissionGranted(true);
    }
    if (!airborneTakeover() && !trial_frame_locked_ && execPermissionGranted()) {
      const bool disarmed = px4_com_.mavrosStateFresh() && !px4_com_.mavrosState().armed;
      const bool timely_enter = !requireEnterConfirmation() ||
          last_enter_time_.load() >= initialization_stable_since_.toSec() + initialization_settle_sec_;
      if (!initialization_ready_ || !disarmed || !timely_enter) {
        setExecPermissionGranted(false);
        ROS_WARN("[fsm] Enter rejected: ground_takeoff requires disarmed vehicle and stable valid localization; press Enter again when ready.");
      } else if (px4_com_.resetLocalOriginToLatestPose()) {
        clearTrialState();
        trial_frame_locked_ = true;
        resetInitializationWindow();
        ROS_INFO("[fsm] ground_takeoff origin locked; preparing automatic takeoff.");
      }
    }
  }

  // A pilot mode change ends the current manual-takeover trial immediately.
  // Keep an emergency STABILIZED latch intact; a fresh pilot OFFBOARD entry
  // after position recovery releases it in runLatchedFailsafe().
  if (airborneTakeover() && !failsafe_latched_ && exec_state_ != IDLE &&
      px4_com_.mavrosStateFresh() &&
      px4_com_.mavrosState().mode != "OFFBOARD") {
    const std::string previous_state = stateName(exec_state_);
    clearTrialState();
    px4_com_.resetConfirmationGate();
    exec_state_ = IDLE;
    trial_frame_locked_ = false;
    resetInitializationWindow();
    publishFsmState(stateName(exec_state_));
    ROS_WARN("[fsm] left OFFBOARD during %s; trial cleared and FSM returned "
             "to IDLE. Next pilot OFFBOARD entry starts a new trial.",
             previous_state.c_str());
    return;
  }

  if (fsm_num % kInterceptionStatePublishDivider == 0) {
    publishUavStateEstimate();
  }
  publishFsmState(stateName(exec_state_));

  // Safety evaluation runs at the full 200 Hz base timer, before the normal
  // 20 Hz mission-state divider. Emergency states own the callback completely
  // so no position/NMPC command can be published after they are latched.
  if (runFailsafe()) {
    if (fsm_num >= 1000) {
      fsm_num = 0;
    }
    return;
  }

  if (fsm_num % kInterceptionExecDivider != 0) {
    // Keep the braking command at the 200 Hz base rate.  The normal FSM logic
    // runs at 20 Hz, which is sufficient for state transitions but leaves a
    // needlessly sparse setpoint stream during a high-speed stop.
    if ((exec_state_ == HOLD && hold_phase_ == HOLD_BRAKE) ||
        (exec_state_ == FINISH_HOLD &&
         finish_hold_phase_ == FINISH_BRAKE)) {
      geometry_msgs::Vector3 zero_velocity;
      px4_com_.publishVelSetpoint(zero_velocity);
    }
    if (fsm_num >= 1000) {
      fsm_num = 0;
    }
    return;
  }

  if (fsm_num >= 1000) {
    fsm_num = 0;
  }

  if (has_target_pos_ && has_object_target_ &&
      !last_target_state_estimate_time_.isZero() &&
      (ros::Time::now() - last_target_state_estimate_time_).toSec() >
          target_state_estimate_timeout_) {
    ROS_WARN_THROTTLE(
        1.0,
        "[fsm] target state estimate timed out for %.2f s, treating target as lost.",
        (ros::Time::now() - last_target_state_estimate_time_).toSec());
    has_object_target_ = false;
    intercept_state_.clear();
  }

  publishBalloonMotionTrigger(exec_state_ == EXEC_INTERCEPTION);

  if (intercepted_ && exec_state_ == EXEC_INTERCEPTION) {
    ROS_INFO("[fsm] InterceptionFSM intercepted=true, switching to FINISH_HOLD.");
    beginFinishHold("intercepted signal");
  }

  switch (exec_state_) {
    // FSM logic 
    case IDLE: {

      // stable local pose (EKF的本地位置稳定)
      if (!has_local_pose_) {
        ROS_INFO_THROTTLE(1.0, "[fsm] IDLE waiting for local pose.");
        break;
      }

      if (airborneTakeover()) {
        if (!px4_com_.localPositionEstimateReliable() || !localStateFinite() ||
            !px4_com_.mavrosStateFresh() || !px4_com_.mavrosState().connected) {
          ROS_WARN_THROTTLE(1.0, "[fsm] airborne_takeover waiting for valid fresh localization and FCU state.");
          break;
        }
        // Convert the latest raw measurement through the current transform and
        // back on publication. The old trial's altitude must not gate this stream.
        px4_com_.publishMeasuredPositionSetpoint();
        if (px4_com_.mavrosState().mode != "OFFBOARD") {
          ROS_INFO_THROTTLE(1.0, "[fsm] airborne_takeover %s; waiting for pilot OFFBOARD (mode=%s).",
              initialization_ready_ ? "READY" : "waiting for stable low-speed state",
              px4_com_.mavrosState().mode.c_str());
          break;
        }
        if (!px4_com_.mavrosState().armed || !initialization_ready_) {
          ROS_WARN_THROTTLE(1.0, "[fsm] OFFBOARD takeover rejected: vehicle must be armed and stably stopped; returning to POSCTL.");
          px4_com_.requestPositionMode();
          break;
        }
        if (!trial_frame_locked_) {
          if (!px4_com_.resetLocalOriginToLatestPose()) break;
          clearTrialState();
          trial_frame_locked_ = true;
          resetInitializationWindow();
          ROS_INFO("[fsm] airborne_takeover origin locked at pilot handover.");
        }
        start_state_ = uav_state_;
        start_state_.header.frame_id = idle_frame_id_;
        start_state_.child_frame_id = "base_link";
        offboard_takeover_start_time_ = ros::Time::now();
        offboard_takeover_pending_ = true;
        exec_state_ = WAIT_TARGET;
        ROS_INFO("[fsm] pilot OFFBOARD takeover accepted at [%.2f %.2f %.2f]; holding for %.2f s.",
                 start_state_.pose.pose.position.x,
                 start_state_.pose.pose.position.y,
                 start_state_.pose.pose.position.z,
                 offboard_takeover_hold_sec_);
        break;
      }

      if (!trial_frame_locked_) {
        ROS_INFO_THROTTLE(1.0, "[fsm] ground_takeoff %s; press Enter while disarmed to lock origin.",
                          initialization_ready_ ? "READY" : "waiting for stable valid localization");
        break;
      }
      if (!px4_com_.localPositionEstimateReliable()) {
        ROS_WARN_THROTTLE(1.0, "[fsm] takeoff paused: position estimate invalid or stale.");
        break;
      }

      if (!px4_com_.hasMavrosState() ||
          px4_com_.mavrosState().mode != "OFFBOARD") {
        px4_com_.publishWarmUpPositionSetpoint();
      }

      const double position_error =
          pointDistance(uav_state_.pose.pose.position,
                        start_state_.pose.pose.position);

      if (!px4_com_.offboardEntryRequested() && !execPermissionGranted()) {
        px4_com_.publishWaitingEnterPrompt("IDLE");
      }
      if (!px4_com_.offboardEntryRequested()) {
        if (!execPermissionGranted()) {
          break;
        }
        setExecPermissionGranted(false);
        px4_com_.setOffboardEntryRequested(true);
        ROS_INFO("[fsm] IDLE received Enter confirmation, requesting OFFBOARD.");
      }

      if (!px4_com_.hasMavrosState() ||
          px4_com_.mavrosState().mode != "OFFBOARD") {
        px4_com_.requestPreparedOffboardMode();
        ROS_INFO_THROTTLE(
            1.0, "[fsm] waiting for OFFBOARD. current mode=%s.",
            px4_com_.hasMavrosState() ? px4_com_.mavrosState().mode.c_str()
                                      : "unknown");
        break;
      }

      if (!px4_com_.mavrosState().armed) {
        px4_com_.armVehicle();
        px4_com_.publishWarmUpPositionSetpoint();
        ROS_INFO_THROTTLE(1.0, "[fsm] IDLE waiting for vehicle to arm.");
        break;
      }

      // Let PX4's position controller hold the takeoff XY anchor.  A velocity
      // setpoint here allowed the vehicle to build substantial horizontal
      // displacement during the high-thrust lift-off transient before the
      // outer position loop could recover it.
      px4_com_.publishPositionSetpoint(
          start_state_.pose.pose.position,
          yawFromQuaternion(start_state_.pose.pose.orientation));
      if (position_error > position_tolerance_) {
        ROS_INFO_THROTTLE(
            1.0,
            "[fsm] IDLE guiding vehicle to start [%.2f %.2f %.2f], current "
            "[%.2f %.2f %.2f], error %.2f m.",
            start_state_.pose.pose.position.x, start_state_.pose.pose.position.y,
            start_state_.pose.pose.position.z, uav_state_.pose.pose.position.x,
            uav_state_.pose.pose.position.y, uav_state_.pose.pose.position.z,
            position_error);
        break;
      }

      exec_state_ = WAIT_TARGET;
      break;
    }

    case WAIT_TARGET: {
      px4_com_.publishHoverSetpoint(start_state_.pose.pose.position,
                                    yawFromQuaternion(start_state_.pose.pose.orientation));

      if (offboard_takeover_pending_ &&
          (ros::Time::now() - offboard_takeover_start_time_).toSec() <
              offboard_takeover_hold_sec_) {
        ROS_INFO_THROTTLE(0.5, "[fsm] OFFBOARD takeover hold %.2f / %.2f s.",
                          (ros::Time::now() - offboard_takeover_start_time_).toSec(),
                          offboard_takeover_hold_sec_);
        break;
      }
      offboard_takeover_pending_ = false;

      if (!px4_com_.ensureLocalPoseReceived("WAIT_TARGET")) {
        break;
      }
      if (!px4_com_.ensureLocalVelocityReceived("WAIT_TARGET")) {
        break;
      }
      if (!targetAvailable()) {
        ROS_WARN_THROTTLE(1.0,
                          has_target_pos_
                              ? "[fsm] WAIT_TARGET waiting for target state estimate."
                              : "[fsm] WAIT_TARGET waiting for target in main view.");
        break;
      }

      if (requireEnterConfirmation() && !airborneTakeover()) {
        px4_com_.publishWaitingEnterPrompt("WAIT_TARGET");
        if (!execPermissionGranted()) {
          break;
        }
        setExecPermissionGranted(false);
      }

      px4_com_.resetHoverSetpoint();
      last_track_lr_time_ = ros::Time();
      exec_state_ = EXEC_INTERCEPTION;
      publishBalloonMotionTrigger(true);
      break;
    }

    case EXEC_INTERCEPTION: {
      const bool recently_in_track_lr =
          !has_target_pos_ && !last_track_lr_time_.isZero() &&
          (ros::Time::now() - last_track_lr_time_).toSec() <=
              terminal_mode_memory_;
      const double lost_timeout = recently_in_track_lr
                                      ? terminal_lost_timeout_
                                      : q_image_timeout_;
      if (!targetAvailable(true, lost_timeout)) {
        if (recently_in_track_lr) {
          ROS_INFO(
              "[fsm] EXEC_INTERCEPTION lost target %.3f s after TRACK_LR; "
              "assuming terminal interception and switching to FINISH_HOLD.",
              (ros::Time::now() - last_target_in_view_time_).toSec());
          has_object_target_ = false;
          beginFinishHold("terminal TRACK_LR target loss");
          break;
        }
        ROS_WARN_THROTTLE(1.0,
                          has_target_pos_
                              ? "[fsm] EXEC_INTERCEPTION lost target, switching to HOLD."
                              : "[fsm] EXEC_INTERCEPTION lost visual target, switching to HOLD.");
        beginHold(has_target_pos_ ? "target state lost" : "visual target lost");
        break;
      }
      if (!px4_com_.ensureLocalPoseReceived("EXEC_INTERCEPTION")) {
        break;
      }
      if (!px4_com_.ensureLocalVelocityReceived("EXEC_INTERCEPTION")) {
        break;
      }
      break;
    }

    case HOLD: {
      runHold();
      publishBalloonMotionTrigger(false);

      if (targetAvailable()) {
        ROS_INFO("[fsm] HOLD reacquired target, switching to EXEC_INTERCEPTION.");
        setExecPermissionGranted(false);
        px4_com_.resetHoverSetpoint();
        exec_state_ = EXEC_INTERCEPTION;
        publishBalloonMotionTrigger(true);
        break;
      }

      if (hold_phase_ == HOLD_BRAKE) {
        // Do not let an Enter pressed during braking initiate a return while
        // the vehicle still has interception momentum.
        setExecPermissionGranted(false);
        break;
      }

      if (requireEnterConfirmation()) {
        ROS_INFO_THROTTLE(
            2.0,
            "[fsm] HOLD waiting for target. Press Enter to RETURN_TO_START.");
        if (!execPermissionGranted()) {
          break;
        }
        setExecPermissionGranted(false);
        ROS_INFO("[fsm] HOLD received Enter, switching to RETURN_TO_START.");
        px4_com_.resetHoverSetpoint();
        beginReturnToStart();
      }
      break;
    }

    case FINISH_HOLD: {
      runFinishHold();
      if (finish_hold_phase_ == FINISH_BRAKE) {
        // Ignore/consume confirmations made while the aircraft is still
        // braking.  Return-to-start is only armed after a stable hold exists.
        setExecPermissionGranted(false);
        break;
      }

      if (requireEnterConfirmation()) {
        px4_com_.publishWaitingEnterPrompt("FINISH_HOLD");
        if (!execPermissionGranted()) {
          ROS_INFO_THROTTLE(
              1.0,
              "[fsm] FINISH_HOLD waiting for Enter before RETURN_TO_START.");
          break;
        }
        setExecPermissionGranted(false);
      }

      px4_com_.resetHoverSetpoint();
      // RETURN_TO_START stays armed in OFFBOARD.  Do not clear the OFFBOARD
      // entry flag here: doing so makes the next trial look like a fresh
      // takeoff and resets the local origin on every cycle.
      px4_com_.resetIdleVelocitySetpoint();
      beginReturnToStart();
      break;
    }

    case RETURN_TO_START: {
      px4_com_.resetHoverSetpoint();

      if (!has_local_pose_) {
        ROS_INFO_THROTTLE(1.0, "[fsm] RETURN_TO_START waiting for local pose.");
        break;
      }

      const double start_yaw =
          yawFromQuaternion(start_state_.pose.pose.orientation);
      px4_com_.publishPoseSetpoint(return_cruise_position_, start_yaw);
      const double horizontal_error = std::hypot(
          uav_state_.pose.pose.position.x - return_cruise_position_.x,
          uav_state_.pose.pose.position.y - return_cruise_position_.y);
      const double return_error = return_to_start_altitude_
          ? pointDistance(uav_state_.pose.pose.position, return_cruise_position_)
          : horizontal_error;
      if (return_error > position_tolerance_) {
        ROS_INFO_THROTTLE(
            1.0,
            "[fsm] RETURN_TO_START cruising to start XY [%.2f %.2f] "
            "at z=%.2f, return position error %.2f m.",
            return_cruise_position_.x, return_cruise_position_.y,
            return_cruise_position_.z, return_error);
        break;
      }

      px4_com_.publishHoverSetpoint(return_cruise_position_,
                                    start_yaw);
      // Keep the next IDLE hover at this altitude as well; otherwise the
      // non-manual path would command a vertical move right after the return.
      start_state_.pose.pose.position.z = return_cruise_position_.z;
      // A completed return terminates the current trial.
      clearTrialState();
      // The vehicle remains armed in OFFBOARD at the original start pose.
      // Clear only the per-trial Enter permission: resetting the whole gate
      // would mark OFFBOARD entry as pending again, consume the next Enter in
      // IDLE, and reset the local origin as if this were another takeoff.
      setExecPermissionGranted(!requireEnterConfirmation());
      exec_state_ = IDLE;
      ROS_INFO(
          "[fsm] RETURN_TO_START complete; preserving the original local "
          "origin and switching through IDLE to WAIT_TARGET.");
      break;
    }

  }
}

}  // namespace plan_manage
