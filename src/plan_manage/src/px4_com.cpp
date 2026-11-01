#include "plan_manage/px4_com.h"
#include <std_msgs/Float64.h>

#include <algorithm>
#include <cmath>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

namespace plan_manage {

namespace {

// ============================================================================
//  Local Utilities （局部工具函数）
// ============================================================================

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

geometry_msgs::Quaternion quaternionFromYaw(double yaw) {
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

geometry_msgs::Quaternion multiplyQuaternions(
    const geometry_msgs::Quaternion& lhs,
    const geometry_msgs::Quaternion& rhs) {
  geometry_msgs::Quaternion result;
  result.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
  result.x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y;
  result.y = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
  result.z = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w;
  return result;
}

geometry_msgs::Vector3 rotateVectorAroundZ(
    const geometry_msgs::Vector3& vector, double yaw) {
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  geometry_msgs::Vector3 result;
  result.x = cos_yaw * vector.x - sin_yaw * vector.y;
  result.y = sin_yaw * vector.x + cos_yaw * vector.y;
  result.z = vector.z;
  return result;
}

void normalizeQuaternionHemisphere(geometry_msgs::Quaternion* q) {
  if (q == nullptr) {
    return;
  }
  if (q->w < 0.0) {
    q->w = -q->w;
    q->x = -q->x;
    q->y = -q->y;
    q->z = -q->z;
  }
}

void publishRawPoseSetpoint(ros::Publisher* position_setpoint_pub,
                            const std::string* frame_id,
                            const geometry_msgs::Point& position, double yaw) {
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.stamp = ros::Time::now();
  if (frame_id != nullptr) {
    target_pose.header.frame_id = *frame_id;
  }
  target_pose.pose.position = position;
  target_pose.pose.orientation = quaternionFromYaw(yaw);
  position_setpoint_pub->publish(target_pose);
}

void publishRawVelocitySetpoint(ros::Publisher* velocity_setpoint_pub,
                                const std::string* frame_id,
                                const geometry_msgs::Vector3& velocity,
                                double yaw_rate = 0.0) {
  geometry_msgs::TwistStamped target_twist;
  target_twist.header.stamp = ros::Time::now();
  if (frame_id != nullptr) {
    target_twist.header.frame_id = *frame_id;
  }
  target_twist.twist.linear = velocity;
  target_twist.twist.angular = geometry_msgs::Vector3();
  target_twist.twist.angular.z = yaw_rate;
  velocity_setpoint_pub->publish(target_twist);
}

double normalizeAngle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

}  // namespace

// ============================================================================
//  Lifecycle and Configuration （生命周期与参数配置）
// ============================================================================

Px4Com::~Px4Com() = default;

void Px4Com::bindContext(const Context& context) { context_ = context; }

void Px4Com::loadParams(ros::NodeHandle& nh, const std::string& prefix) {
  nh.param(prefix + "/failsafe/mavros_state_timeout",
           mavros_state_timeout_, 2.5);
  if (!std::isfinite(mavros_state_timeout_) || mavros_state_timeout_ <= 0.0) {
    ROS_WARN("[px4_com] invalid mavros_state_timeout; using 2.5 s.");
    mavros_state_timeout_ = 2.5;
  }
  nh.param(prefix + "/offboard/warmup_setpoints", warmup_setpoints_required_, 50);
  nh.param(prefix + "/offboard/mode_request_interval", mode_request_interval_, 2.0);
  nh.param(prefix + "/offboard/arm_request_interval", arm_request_interval_, 2.0);
  nh.param(prefix + "/idle/vertical_speed", idle_vertical_speed_, 0.6);
  nh.param(prefix + "/idle/horizontal_speed", idle_horizontal_speed_, 1.0);
  nh.param(prefix + "/idle/yaw_rate", idle_yaw_rate_, 0.5);
  nh.param(prefix + "/idle/horizontal_kp", idle_horizontal_kp_, 1.0);
  nh.param(prefix + "/idle/horizontal_kd", idle_horizontal_kd_, 0.4);
  nh.param(prefix + "/idle/horizontal_ki", idle_horizontal_ki_, 0.0);
  nh.param(prefix + "/idle/horizontal_i_limit", idle_horizontal_i_limit_, 1.0);
  nh.param(prefix + "/idle/vertical_kp", idle_vertical_kp_, 1.0);
  nh.param(prefix + "/idle/vertical_kd", idle_vertical_kd_, 0.4);
  nh.param(prefix + "/idle/yaw_kp", idle_yaw_kp_, 1.0);
  nh.param(prefix + "/idle/horizontal_accel", idle_horizontal_accel_, 2.0);
  nh.param(prefix + "/idle/vertical_accel", idle_vertical_accel_, 1.5);
  nh.param(prefix + "/idle/velocity_bias_x", idle_velocity_bias_x_, 0.0);
  nh.param(prefix + "/idle/velocity_bias_y", idle_velocity_bias_y_, 0.0);
  nh.param(prefix + "/local_frame/use_first_pose_as_origin",
           use_first_pose_as_origin_, false);
  nh.param(prefix + "/local_frame/rotate_by_initial_yaw",
           rotate_by_initial_yaw_, false);
  nh.param(prefix + "/failsafe/estimator_status_timeout",
           estimator_status_timeout_, 2.5);
  if (!std::isfinite(estimator_status_timeout_) ||
      estimator_status_timeout_ <= 0.0) {
    ROS_WARN("[px4_com] invalid estimator_status_timeout; using 2.5 s.");
    estimator_status_timeout_ = 2.5;
  }
  nh.param(prefix + "/failsafe/local_state_timeout", local_state_timeout_,
           0.5);
  if (!std::isfinite(local_state_timeout_) || local_state_timeout_ <= 0.0) {
    ROS_WARN("[px4_com] invalid local_state_timeout; using 0.5 s.");
    local_state_timeout_ = 0.5;
  }
}

// ============================================================================
//  ROS Interface （ROS接口）
// ============================================================================

void Px4Com::initRosInterfaces(ros::NodeHandle& nh) {
  local_yaw_pub_ = nh.advertise<std_msgs::Float64>("/local_frame/initial_yaw", 1, true);
  local_origin_pub_ =
      nh.advertise<geometry_msgs::PointStamped>("/local_frame/origin", 1, true);
  mavros_state_sub_ = nh.subscribe<mavros_msgs::State>(
      "/mavros/state", 10,
      [this](const mavros_msgs::State::ConstPtr& msg) { mavrosStateCallback(msg); });
  estimator_status_sub_ = nh.subscribe<mavros_msgs::EstimatorStatus>(
      "/mavros/estimator_status", 10,
      [this](const mavros_msgs::EstimatorStatus::ConstPtr& msg) {
        estimatorStatusCallback(msg);
      });
  local_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
      "/mavros/local_position/pose", 10,
      [this](const geometry_msgs::PoseStamped::ConstPtr& msg) { localPoseCallback(msg); });
  local_velocity_sub_ = nh.subscribe<geometry_msgs::TwistStamped>(
      "/mavros/local_position/velocity_local", 10,
      [this](const geometry_msgs::TwistStamped::ConstPtr& msg) { localVelocityCallback(msg); });

  position_setpoint_pub_ =
      nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  velocity_setpoint_pub_ =
      nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
  raw_local_setpoint_pub_ =
      nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
  arming_client_ =
      nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  set_mode_client_ =
      nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}

void Px4Com::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg) {
  updateMavrosState(*msg);
}

void Px4Com::estimatorStatusCallback(
    const mavros_msgs::EstimatorStatus::ConstPtr& msg) {
  estimator_status_ = *msg;
  has_estimator_status_ = true;
  last_estimator_status_time_ = ros::Time::now();
}

void Px4Com::localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  updateLocalPose(*msg);
  if (context_.uav_state != nullptr) {
    geometry_msgs::PoseStamped control_pose;
    control_pose.header = context_.uav_state->header;
    control_pose.pose = context_.uav_state->pose.pose;
    captureTakeoffAnchorIfUnset(control_pose);
  } else {
    captureTakeoffAnchorIfUnset(*msg);
  }
}

void Px4Com::localVelocityCallback(
    const geometry_msgs::TwistStamped::ConstPtr& msg) {
  updateLocalVelocity(*msg);
}

// ============================================================================
//  State Update （状态更新）
// ============================================================================

void Px4Com::updateMavrosState(const mavros_msgs::State& msg) {
  mavros_state_ = msg;
  has_mavros_state_ = true;
  last_mavros_state_time_ = ros::Time::now();
}

void Px4Com::updateLocalPose(const geometry_msgs::PoseStamped& msg,
                             const std::string& child_frame_id) {
  latest_local_pose_ = msg;
  last_local_pose_time_ = ros::Time::now();

  if (!defer_local_frame_capture_) captureInitialYawIfNeeded(msg);

  if (!defer_local_frame_capture_ && use_first_pose_as_origin_ && !has_local_origin_) {
    local_origin_ = msg.pose.position;
    has_local_origin_ = true;
    publishLocalOrigin();
    ROS_INFO("[px4_com] local_frame origin captured from first local pose: [%.3f %.3f %.3f].",
             local_origin_.x, local_origin_.y, local_origin_.z);
  }

  if (context_.uav_state != nullptr) {
    context_.uav_state->header = msg.header;
    context_.uav_state->child_frame_id = child_frame_id;
    context_.uav_state->pose.pose = msg.pose;
    context_.uav_state->pose.pose.position = toControlFrame(msg.pose.position);
    context_.uav_state->pose.pose.orientation =
        toControlFrame(msg.pose.orientation);
    normalizeQuaternionHemisphere(&context_.uav_state->pose.pose.orientation);
  }
  if (context_.has_local_pose != nullptr) {
    *context_.has_local_pose = true;
  }
}

void Px4Com::updateLocalVelocity(const geometry_msgs::TwistStamped& msg) {
  latest_local_velocity_ = msg;
  last_local_velocity_time_ = ros::Time::now();
  if (context_.uav_state != nullptr) {
    if (context_.uav_state->header.frame_id.empty() && context_.idle_frame_id != nullptr) {
      context_.uav_state->header.frame_id = *context_.idle_frame_id;
    }
    context_.uav_state->header.stamp = msg.header.stamp;
    context_.uav_state->twist.twist = msg.twist;
    context_.uav_state->twist.twist.linear =
        toControlFrame(msg.twist.linear);
  }
  if (context_.has_local_velocity != nullptr) {
    *context_.has_local_velocity = true;
  }
}

// ============================================================================
//  Local Frame and Origin （本地坐标系与原点）
// ============================================================================

bool Px4Com::resetLocalOriginToLatestPose() {
  if (!latest_local_pose_.has_value()) {
    ROS_WARN("[px4_com] cannot reset local_frame origin: no /mavros/local_position/pose received yet.");
    return false;
  }

  local_origin_ = latest_local_pose_->pose.position;
  has_local_origin_ = true;

  if (rotate_by_initial_yaw_) {
    initial_yaw_ = yawFromQuaternion(latest_local_pose_->pose.orientation);
    has_initial_yaw_ = true;
  }

  if (context_.uav_state != nullptr) {
    context_.uav_state->header = latest_local_pose_->header;
    context_.uav_state->child_frame_id = "base_link";
    context_.uav_state->pose.pose = latest_local_pose_->pose;
    context_.uav_state->pose.pose.position =
        toControlFrame(latest_local_pose_->pose.position);
    context_.uav_state->pose.pose.orientation =
        toControlFrame(latest_local_pose_->pose.orientation);
    normalizeQuaternionHemisphere(&context_.uav_state->pose.pose.orientation);
    if (latest_local_velocity_) {
      context_.uav_state->twist.twist = latest_local_velocity_->twist;
      context_.uav_state->twist.twist.linear =
          toControlFrame(latest_local_velocity_->twist.linear);
    }
    takeoff_anchor_ = context_.uav_state->pose.pose.position;
  } else {
    geometry_msgs::Point zero;
    takeoff_anchor_ = zero;
  }

  has_takeoff_xy_anchor_ = true;
  warmup_setpoint_count_ = 0;
  last_mode_request_ = ros::Time();
  publishLocalOrigin();
  resetHoverSetpoint();
  ROS_INFO(
      "[px4_com] local_frame explicitly locked: origin "
      "[%.3f %.3f %.3f], initial yaw %.3f rad, yaw rotation %s.",
      local_origin_.x, local_origin_.y, local_origin_.z, initial_yaw_,
      rotate_by_initial_yaw_ ? "enabled" : "disabled");
  return true;
}

void Px4Com::deferLocalFrameCapture() {
  defer_local_frame_capture_ = true;
  use_first_pose_as_origin_ = true;
  rotate_by_initial_yaw_ = true;
}

void Px4Com::publishMeasuredPositionSetpoint() {
  if (!latest_local_pose_) return;
  publishPositionSetpoint(toControlFrame(latest_local_pose_->pose.position),
      yawFromQuaternion(toControlFrame(latest_local_pose_->pose.orientation)));
}

void Px4Com::publishLocalOrigin() const {
  if (local_yaw_pub_) {
    std_msgs::Float64 yaw;
    yaw.data = has_initial_yaw_ ? initial_yaw_ : 0.0;
    local_yaw_pub_.publish(yaw);
  }
  if (!local_origin_pub_) {
    return;
  }
  geometry_msgs::PointStamped origin;
  origin.header.stamp = ros::Time::now();
  origin.header.frame_id =
      context_.idle_frame_id != nullptr ? *context_.idle_frame_id : "map";
  origin.point = local_origin_;
  local_origin_pub_.publish(origin);
}

geometry_msgs::Point Px4Com::toControlFrame(
    const geometry_msgs::Point& mavros_local_position) const {
  geometry_msgs::Point position = mavros_local_position;
  if (use_first_pose_as_origin_ && has_local_origin_) {
    position.x -= local_origin_.x;
    position.y -= local_origin_.y;
    position.z -= local_origin_.z;
  }
  if (rotate_by_initial_yaw_ && has_initial_yaw_) {
    geometry_msgs::Vector3 vector;
    vector.x = position.x;
    vector.y = position.y;
    vector.z = position.z;
    const geometry_msgs::Vector3 rotated =
        rotateVectorAroundZ(vector, -initial_yaw_);
    position.x = rotated.x;
    position.y = rotated.y;
    position.z = rotated.z;
  }
  return position;
}

geometry_msgs::Point Px4Com::toMavrosLocalFrame(
    const geometry_msgs::Point& control_frame_position) const {
  geometry_msgs::Point position = control_frame_position;
  if (rotate_by_initial_yaw_ && has_initial_yaw_) {
    geometry_msgs::Vector3 vector;
    vector.x = position.x;
    vector.y = position.y;
    vector.z = position.z;
    const geometry_msgs::Vector3 rotated =
        rotateVectorAroundZ(vector, initial_yaw_);
    position.x = rotated.x;
    position.y = rotated.y;
    position.z = rotated.z;
  }
  if (use_first_pose_as_origin_ && has_local_origin_) {
    position.x += local_origin_.x;
    position.y += local_origin_.y;
    position.z += local_origin_.z;
  }
  return position;
}

geometry_msgs::Vector3 Px4Com::toControlFrame(
    const geometry_msgs::Vector3& mavros_local_vector) const {
  if (!rotate_by_initial_yaw_ || !has_initial_yaw_) {
    return mavros_local_vector;
  }
  return rotateVectorAroundZ(mavros_local_vector, -initial_yaw_);
}

geometry_msgs::Vector3 Px4Com::toMavrosLocalFrame(
    const geometry_msgs::Vector3& control_frame_vector) const {
  if (!rotate_by_initial_yaw_ || !has_initial_yaw_) {
    return control_frame_vector;
  }
  return rotateVectorAroundZ(control_frame_vector, initial_yaw_);
}

geometry_msgs::Quaternion Px4Com::toControlFrame(
    const geometry_msgs::Quaternion& mavros_local_orientation) const {
  if (!rotate_by_initial_yaw_ || !has_initial_yaw_) {
    return mavros_local_orientation;
  }
  return multiplyQuaternions(quaternionFromYaw(-initial_yaw_),
                             mavros_local_orientation);
}

double Px4Com::toMavrosLocalYaw(double control_frame_yaw) const {
  if (!rotate_by_initial_yaw_ || !has_initial_yaw_) {
    return control_frame_yaw;
  }
  return normalizeAngle(control_frame_yaw + initial_yaw_);
}

void Px4Com::captureInitialYawIfNeeded(
    const geometry_msgs::PoseStamped& msg) {
  if (!rotate_by_initial_yaw_ || has_initial_yaw_) {
    return;
  }

  initial_yaw_ = yawFromQuaternion(msg.pose.orientation);
  has_initial_yaw_ = true;
  if (local_yaw_pub_) {
    std_msgs::Float64 yaw;
    yaw.data = initial_yaw_;
    local_yaw_pub_.publish(yaw);
  }
  ROS_INFO("[px4_com] local_frame initial yaw captured: %.3f rad.",
           initial_yaw_);
}

// ============================================================================
//  PX4 Mode and Arming （PX4模式与解锁）
// ============================================================================

bool Px4Com::requestPreparedOffboardMode() {
  if (!warmupSetpointsReady() ||
      (!last_mode_request_.isZero() &&
       (ros::Time::now() - last_mode_request_).toSec() < mode_request_interval_)) return false;
  return requestOffboardMode();
}

bool Px4Com::requestOffboardMode() {
  mavros_msgs::SetMode mode_cmd;
  mode_cmd.request.custom_mode = "OFFBOARD";
  const bool sent = set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent;
  if (sent) {
    ROS_INFO("[px4_com] OFFBOARD mode request sent.");
  } else {
    ROS_WARN("[px4_com] failed to send OFFBOARD mode request.");
  }
  last_mode_request_ = ros::Time::now();
  return sent;
}

bool Px4Com::requestStabilizedMode() {
  const ros::Time now = ros::Time::now();
  constexpr double kEmergencyModeRetrySec = 0.2;
  if (!last_mode_request_.isZero() &&
      (now - last_mode_request_).toSec() < kEmergencyModeRetrySec) {
    return false;
  }

  mavros_msgs::SetMode mode_cmd;
  mode_cmd.request.custom_mode = "STABILIZED";
  const bool sent =
      set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent;
  if (sent) {
    ROS_WARN_THROTTLE(1.0, "[px4_com] STABILIZED mode request sent.");
  } else {
    ROS_ERROR_THROTTLE(1.0,
                       "[px4_com] failed to send STABILIZED mode request.");
  }
  last_mode_request_ = now;
  return sent;
}

bool Px4Com::requestPositionMode() {
  const ros::Time now = ros::Time::now();
  constexpr double kEmergencyModeRetrySec = 0.2;
  if (!last_mode_request_.isZero() &&
      (now - last_mode_request_).toSec() < kEmergencyModeRetrySec) {
    return false;
  }

  mavros_msgs::SetMode mode_cmd;
  mode_cmd.request.custom_mode = "POSCTL";
  const bool sent =
      set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent;
  if (sent) {
    ROS_WARN_THROTTLE(1.0, "[px4_com] POSCTL mode request sent.");
  } else {
    ROS_ERROR_THROTTLE(1.0,
                       "[px4_com] failed to send POSCTL mode request.");
  }
  last_mode_request_ = now;
  return sent;
}

bool Px4Com::requestAutoLandMode() {
  const ros::Time now = ros::Time::now();
  if (!last_mode_request_.isZero() &&
      (now - last_mode_request_).toSec() < mode_request_interval_) {
    return false;
  }
  mavros_msgs::SetMode mode_cmd;
  mode_cmd.request.custom_mode = "AUTO.LAND";
  const bool sent = set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent;
  if (sent) {
    ROS_INFO("[px4_com] AUTO.LAND mode request sent.");
  } else {
    ROS_WARN_THROTTLE(mode_request_interval_,
                      "[px4_com] failed to send AUTO.LAND mode request.");
  }
  last_mode_request_ = now;
  return sent;
}

bool Px4Com::armVehicle() {
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  const bool sent = arming_client_.call(arm_cmd) && arm_cmd.response.success;
  if (sent) {
    ROS_INFO_THROTTLE(arm_request_interval_,
                      "[px4_com] vehicle arm request sent.");
  } else {
    ROS_WARN_THROTTLE(arm_request_interval_,
                      "[px4_com] failed to arm vehicle.");
  }
  last_arm_request_ = ros::Time::now();
  return sent;
}

void Px4Com::captureTakeoffAnchorIfUnset(const geometry_msgs::PoseStamped& msg) {
  if (has_takeoff_xy_anchor_) {
    return;
  }
  takeoff_anchor_ = msg.pose.position;
  has_takeoff_xy_anchor_ = true;
}

bool Px4Com::hasMavrosState() const { return has_mavros_state_; }

bool Px4Com::mavrosStateFresh() const {
  return has_mavros_state_ && !last_mavros_state_time_.isZero() &&
         (ros::Time::now() - last_mavros_state_time_).toSec() <=
             mavros_state_timeout_;
}

const mavros_msgs::State& Px4Com::mavrosState() const { return mavros_state_; }

bool Px4Com::localPositionEstimateReliable() const {
  const ros::Time now = ros::Time::now();
  if (!has_estimator_status_ || last_estimator_status_time_.isZero() ||
      estimator_status_timeout_ <= 0.0 ||
      (now - last_estimator_status_time_).toSec() >
          estimator_status_timeout_ ||
      last_local_pose_time_.isZero() || last_local_velocity_time_.isZero() ||
      (now - last_local_pose_time_).toSec() > local_state_timeout_ ||
      (now - last_local_velocity_time_).toSec() > local_state_timeout_) {
    return false;
  }

  const bool horizontal_position_valid =
      estimator_status_.pos_horiz_rel_status_flag ||
      estimator_status_.pos_horiz_abs_status_flag;
  return horizontal_position_valid &&
         estimator_status_.pos_vert_abs_status_flag;
}

bool Px4Com::offboardEntryRequested() const {
  return offboard_entry_requested_;
}

void Px4Com::setOffboardEntryRequested(bool requested) {
  offboard_entry_requested_ = requested;
}

void Px4Com::resetIdleVelocitySetpoint() {
  last_idle_velocity_setpoint_.reset();
  last_idle_velocity_setpoint_time_ = ros::Time();
  idle_position_i_error_ = geometry_msgs::Vector3();
}

// ============================================================================
//  Setpoint Publishing （Setpoint发布）
// ============================================================================

void Px4Com::publishPoseSetpoint(const geometry_msgs::Point& position, double yaw,
                                 bool count_warmup) {
  resetHoverSetpoint();

  geometry_msgs::Vector3 velocity;
  double yaw_rate = 0.0;
  const ros::Time now = ros::Time::now();
  geometry_msgs::Vector3 previous_velocity;
  double dt = 0.05;
  if (last_idle_velocity_setpoint_.has_value() &&
      !last_idle_velocity_setpoint_time_.isZero()) {
    previous_velocity = *last_idle_velocity_setpoint_;
    dt = clamp((now - last_idle_velocity_setpoint_time_).toSec(), 1e-3, 0.2);
  }

  if (context_.has_local_pose == nullptr || !*context_.has_local_pose ||
      context_.uav_state == nullptr) {
    ROS_WARN_THROTTLE(
        1.0,
        "[px4_com] cannot guide to pose with velocity setpoint: no local pose.");
  } else {
    const geometry_msgs::Point& current_position =
        context_.uav_state->pose.pose.position;
    const geometry_msgs::Vector3& current_velocity =
        context_.uav_state->twist.twist.linear;
    const double error_x = position.x - current_position.x;
    const double error_y = position.y - current_position.y;
    const double error_z = position.z - current_position.z;
    const double position_error =
        std::hypot(std::hypot(error_x, error_y), error_z);
    const double tolerance =
        context_.position_tolerance != nullptr ? *context_.position_tolerance : 0.0;

    if (idle_horizontal_ki_ > 1e-6) {
      idle_position_i_error_.x += error_x * dt;
      idle_position_i_error_.y += error_y * dt;
      const double i_norm =
          std::hypot(idle_position_i_error_.x, idle_position_i_error_.y);
      if (i_norm > idle_horizontal_i_limit_ && i_norm > 1e-6) {
        const double scale = idle_horizontal_i_limit_ / i_norm;
        idle_position_i_error_.x *= scale;
        idle_position_i_error_.y *= scale;
      }
    } else {
      idle_position_i_error_ = geometry_msgs::Vector3();
    }

    velocity.x = idle_velocity_bias_x_ +
                 idle_horizontal_kp_ * error_x -
                 idle_horizontal_kd_ * current_velocity.x +
                 idle_horizontal_ki_ * idle_position_i_error_.x;
    velocity.y = idle_velocity_bias_y_ +
                 idle_horizontal_kp_ * error_y -
                 idle_horizontal_kd_ * current_velocity.y +
                 idle_horizontal_ki_ * idle_position_i_error_.y;

    const double horizontal_speed = std::hypot(velocity.x, velocity.y);
    const double horizontal_distance = std::hypot(error_x, error_y);
    const double current_horizontal_speed =
        std::hypot(current_velocity.x, current_velocity.y);
    const double horizontal_compensation_speed = std::hypot(
        idle_velocity_bias_x_ + idle_horizontal_ki_ * idle_position_i_error_.x,
        idle_velocity_bias_y_ + idle_horizontal_ki_ * idle_position_i_error_.y);
    double horizontal_speed_limit = idle_horizontal_speed_;
    if (idle_horizontal_accel_ > 1e-6) {
      const double distance_limited_speed =
          std::sqrt(std::max(0.0, 2.0 * idle_horizontal_accel_ *
                                      horizontal_distance));
      horizontal_speed_limit =
          std::min(horizontal_speed_limit,
                   std::max(distance_limited_speed,
                            std::min(idle_horizontal_speed_,
                                     std::max(current_horizontal_speed,
                                              horizontal_compensation_speed))));
    }
    if (horizontal_speed > horizontal_speed_limit &&
        horizontal_speed > 1e-6) {
      const double scale = horizontal_speed_limit / horizontal_speed;
      velocity.x *= scale;
      velocity.y *= scale;
    }

    if (tolerance <= 0.0 || position_error > tolerance) {
      double vertical_speed_limit = idle_vertical_speed_;
      if (idle_vertical_accel_ > 1e-6) {
        const double distance_limited_speed =
            std::sqrt(std::max(0.0, 2.0 * idle_vertical_accel_ *
                                        std::abs(error_z)));
        vertical_speed_limit =
            std::min(vertical_speed_limit,
                     std::max(distance_limited_speed,
                              std::min(idle_vertical_speed_,
                                       std::abs(current_velocity.z))));
      }
      velocity.z = clamp(idle_vertical_kp_ * error_z -
                             idle_vertical_kd_ * current_velocity.z,
                         -vertical_speed_limit, vertical_speed_limit);
    }

    const double current_yaw =
        yawFromQuaternion(context_.uav_state->pose.pose.orientation);
    yaw_rate = clamp(idle_yaw_kp_ * normalizeAngle(yaw - current_yaw),
                     -idle_yaw_rate_, idle_yaw_rate_);
  }

  if (idle_horizontal_accel_ > 1e-6) {
    const double max_delta = idle_horizontal_accel_ * dt;
    double delta_x = velocity.x - previous_velocity.x;
    double delta_y = velocity.y - previous_velocity.y;
    const double delta_norm = std::hypot(delta_x, delta_y);
    if (delta_norm > max_delta && delta_norm > 1e-6) {
      const double scale = max_delta / delta_norm;
      delta_x *= scale;
      delta_y *= scale;
    }
    velocity.x = previous_velocity.x + delta_x;
    velocity.y = previous_velocity.y + delta_y;
  }
  if (idle_vertical_accel_ > 1e-6) {
    const double max_delta = idle_vertical_accel_ * dt;
    velocity.z = previous_velocity.z +
                 clamp(velocity.z - previous_velocity.z,
                       -max_delta, max_delta);
  }

  last_idle_velocity_setpoint_ = velocity;
  last_idle_velocity_setpoint_time_ = now;

  publishRawVelocitySetpoint(
      &velocity_setpoint_pub_, context_.idle_frame_id,
      toMavrosLocalFrame(velocity), yaw_rate);
  if (count_warmup) {
    ++warmup_setpoint_count_;
  }
}

void Px4Com::publishPositionSetpoint(const geometry_msgs::Point& position,
                                     double yaw) {
  resetHoverSetpoint();
  resetIdleVelocitySetpoint();
  publishRawPoseSetpoint(&position_setpoint_pub_, context_.idle_frame_id,
                         toMavrosLocalFrame(position),
                         toMavrosLocalYaw(yaw));
}

void Px4Com::publishPVASetpoint(
    const quadrotor_msgs::TrajectoryPoint& point) {
  resetHoverSetpoint();
  resetIdleVelocitySetpoint();

  mavros_msgs::PositionTarget target;
  target.header.stamp = ros::Time::now();
  target.header.frame_id =
      context_.idle_frame_id != nullptr ? *context_.idle_frame_id : "map";
  // MAVROS accepts ROS ENU values and performs the ENU-to-NED conversion
  // before sending SET_POSITION_TARGET_LOCAL_NED to PX4.
  target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  target.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  target.position = toMavrosLocalFrame(point.pose.position);
  target.velocity = toMavrosLocalFrame(point.velocity.linear);
  target.acceleration_or_force =
      toMavrosLocalFrame(point.acceleration.linear);
  target.yaw = toMavrosLocalYaw(point.heading);
  raw_local_setpoint_pub_.publish(target);
}

void Px4Com::publishVelSetpoint(const geometry_msgs::Vector3& velocity) {
  resetHoverSetpoint();
  resetIdleVelocitySetpoint();
  publishRawVelocitySetpoint(
      &velocity_setpoint_pub_, context_.idle_frame_id,
      toMavrosLocalFrame(velocity));
}

void Px4Com::publishHoverSetpoint(const geometry_msgs::Point& position, double yaw) {
  resetIdleVelocitySetpoint();
  if (!hover_setpoint_position_.has_value() || !hover_setpoint_yaw_.has_value()) {
    hover_setpoint_position_ = position;
    hover_setpoint_yaw_ = yaw;
  }

  publishRawPoseSetpoint(&position_setpoint_pub_, context_.idle_frame_id,
                         toMavrosLocalFrame(*hover_setpoint_position_),
                         toMavrosLocalYaw(*hover_setpoint_yaw_));
}

void Px4Com::resetHoverSetpoint() {
  hover_setpoint_position_.reset();
  hover_setpoint_yaw_.reset();
}

void Px4Com::publishWarmUpPositionSetpoint() {
  if (context_.has_local_pose != nullptr && *context_.has_local_pose &&
      context_.uav_state != nullptr) {
    publishPositionSetpoint(
        context_.uav_state->pose.pose.position,
        yawFromQuaternion(context_.uav_state->pose.pose.orientation));
    ++warmup_setpoint_count_;
  } else if (context_.start_state != nullptr) {
    publishPositionSetpoint(
        context_.start_state->pose.pose.position,
        yawFromQuaternion(context_.start_state->pose.pose.orientation));
    ++warmup_setpoint_count_;
  }
}

bool Px4Com::warmupSetpointsReady() const {
  return warmup_setpoint_count_ >= warmup_setpoints_required_;
}

// ============================================================================
//  Readiness and Confirmation Helpers （就绪检查与确认辅助）
// ============================================================================

bool Px4Com::ensureLocalPoseReceived(const char* wait_context) const {
  if (context_.has_local_pose != nullptr && *context_.has_local_pose) {
    return true;
  }

  if (wait_context != nullptr && wait_context[0] != '\0') {
    ROS_INFO_THROTTLE(
        1.0, "[px4_com] %s waiting for /mavros/local_position/pose.",
        wait_context);
  } else {
    ROS_INFO_THROTTLE(
        1.0, "[px4_com] waiting for /mavros/local_position/pose.");
  }
  return false;
}

bool Px4Com::ensureLocalVelocityReceived(const char* wait_context) const {
  if (context_.has_local_velocity != nullptr && *context_.has_local_velocity) {
    return true;
  }

  if (wait_context != nullptr && wait_context[0] != '\0') {
    ROS_WARN_THROTTLE(1.0, "[px4_com] %s waiting for local velocity.",
                      wait_context);
  } else {
    ROS_WARN_THROTTLE(1.0, "[px4_com] waiting for local velocity.");
  }
  return false;
}

void Px4Com::publishWaitingEnterPrompt(const char* state_name) const {
  if (context_.require_enter_confirmation == nullptr ||
      !*context_.require_enter_confirmation) {
    return;
  }
  ROS_INFO_THROTTLE(2.0, "[px4_com] %s  <<Press Enter>> to continue.",
                    state_name);
}

void Px4Com::resetConfirmationGate() {
  resetIdleVelocitySetpoint();
  if (context_.exec_permission_granted != nullptr &&
      context_.require_enter_confirmation != nullptr) {
    *context_.exec_permission_granted = !*context_.require_enter_confirmation;
  }
  offboard_entry_requested_ = false;
}

}  // namespace plan_manage
