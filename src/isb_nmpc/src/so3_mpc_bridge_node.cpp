#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/QuaternionStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/quad_state_estimate.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <quadrotor_msgs/FsmState.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <swarm_msgs/MassPoints.h>

#include "isb_nmpc/so3_mpc_controller.h"
#include "isb_nmpc/so3_mpc_params.h"

namespace isb_nmpc {

class So3MpcBridgeNode {
 public:
  // ==========================================================================
  // 1. Constructor （构造函数）
  // ==========================================================================
  So3MpcBridgeNode()
      : nh_(),
        pnh_("~"),
        controller_(nh_, pnh_, "isb_nmpc/predicted_trajectory_vis") {
    if (!params_.loadParameters(pnh_)) {
      ROS_ERROR("ISB-NMPC bridge: failed to load parameters.");
      ros::shutdown();
      return;
    }

    pnh_.param("control_rate", control_rate_hz_, 50.0);
    pnh_.param("hover_thrust", hover_thrust_, 0.5);
    pnh_.param("hover_acceleration", hover_acceleration_, 9.81);
    pnh_.param("activation_hover_sec", activation_hover_sec_, 0.25);
    pnh_.param<std::string>(
        "attitude_target_topic", attitude_target_topic_, attitude_target_topic_);
    pnh_.param<std::string>(
        "control_command_topic", control_command_topic_, control_command_topic_);
    pnh_.param("dry_run", dry_run_, dry_run_);
    pnh_.param("restart_after_interception", restart_after_interception_,
               restart_after_interception_);
    pnh_.param<std::string>(
        "state_estimate_topic", uav_state_estimate_topic_, uav_state_estimate_topic_);
    pnh_.param<std::string>(
        "target_state_estimate_topic", target_state_estimate_topic_,
        target_state_estimate_topic_);
    pnh_.param<std::string>("q_image_topic", q_image_topic_, q_image_topic_);
    pnh_.param<std::string>("target_in_main_view_topic",
                            target_in_main_view_topic_,
                            target_in_main_view_topic_);
    pnh_.param("manager/has_target_pos", has_target_pos_, has_target_pos_);
    pnh_.param("estimator/q_image_timeout", q_image_timeout_, q_image_timeout_);
    pnh_.param("estimator/require_target_in_view", require_target_in_view_,
               require_target_in_view_);
    pnh_.param("target_dropout_hold_sec", target_dropout_hold_sec_,
               target_dropout_hold_sec_);
    target_dropout_hold_sec_ = std::max(0.0, target_dropout_hold_sec_);
    pnh_.param("fsm_state_timeout", fsm_state_timeout_, 0.3);
    pnh_.param("uav_state_timeout", uav_state_timeout_, 0.3);
    if (!std::isfinite(fsm_state_timeout_) || fsm_state_timeout_ <= 0.0 ||
        !std::isfinite(uav_state_timeout_) || uav_state_timeout_ <= 0.0) {
      ROS_ERROR("ISB-NMPC bridge: watchdog timeouts must be finite and positive.");
      ros::shutdown();
      return;
    }
    pnh_.param<std::string>("fsm_state_topic", fsm_state_topic_, fsm_state_topic_);
    pnh_.param<std::string>("current_target_topic", current_target_topic_,
                            current_target_topic_);
    pnh_.param<std::string>(
        "reload_params_topic", reload_params_topic_, reload_params_topic_);

    if (hover_acceleration_ <= 0.0) {
      ROS_ERROR("ISB-NMPC bridge: hover_acceleration must be positive.");
      ros::shutdown();
      return;
    }
    if (hover_thrust_ <= 0.0 || hover_thrust_ > 1.0) {
      ROS_ERROR("ISB-NMPC bridge: hover_thrust must be in (0, 1]. Got %.3f.",
                hover_thrust_);
      ros::shutdown();
      return;
    }

    local_frame_yaw_sub_ = nh_.subscribe<std_msgs::Float64>(
        "/local_frame/initial_yaw", 1, [this](const std_msgs::Float64::ConstPtr& msg) {
          if (std::isfinite(msg->data)) control_frame_yaw_ = msg->data;
        });
    uav_state_estimate_sub_ = nh_.subscribe(
        uav_state_estimate_topic_, 10,
        &So3MpcBridgeNode::uavStateEstimateCallback, this);
    target_state_estimate_sub_ = nh_.subscribe(
        target_state_estimate_topic_, 10,
        &So3MpcBridgeNode::targetStateEstimateCallback, this);
    q_image_sub_ = nh_.subscribe(
        q_image_topic_, 10, &So3MpcBridgeNode::qImageCallback, this);
    target_in_main_view_sub_ = nh_.subscribe(
        target_in_main_view_topic_, 10,
        &So3MpcBridgeNode::targetInMainViewCallback, this);
    current_target_sub_ = nh_.subscribe(
        current_target_topic_, 10, &So3MpcBridgeNode::currentTargetCallback,
        this);
    fsm_state_sub_ = nh_.subscribe(
        fsm_state_topic_, 10, &So3MpcBridgeNode::fsmStateCallback, this);
    mavros_state_sub_ = nh_.subscribe(
        "/mavros/state", 10, &So3MpcBridgeNode::mavrosStateCallback, this);
    reload_params_sub_ = nh_.subscribe(
        reload_params_topic_, 1, &So3MpcBridgeNode::reloadParamsCallback, this);

    control_cmd_pub_ =
        nh_.advertise<quadrotor_msgs::ControlCommand>(control_command_topic_, 10);
    attitude_target_pub_ =
        nh_.advertise<mavros_msgs::AttitudeTarget>(attitude_target_topic_, 10);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    startup_time_ = ros::Time::now();

    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, control_rate_hz_)),
        &So3MpcBridgeNode::controlLoop, this);

    if (dry_run_) {
      interception_active_ = true;
      activation_time_ = ros::Time::now();
      controller_.resetForActivation();
      ROS_WARN("ISB-NMPC bridge: dry_run enabled; MPC visualization runs, "
               "but MAVROS attitude setpoints are suppressed.");
    }
  }

 private:
  // ==========================================================================
  // 2. Subscriber Callbacks （ROS订阅回调）
  // ==========================================================================
  void uavStateEstimateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    last_uav_state_time_ = ros::Time::now();
    latest_state_estimate_.timestamp = msg->header.stamp;
    latest_state_estimate_.coordinate_frame =
        quadrotor_common::QuadStateEstimate::CoordinateFrame::WORLD;
    latest_state_estimate_.position.x() = msg->pose.pose.position.x;
    latest_state_estimate_.position.y() = msg->pose.pose.position.y;
    latest_state_estimate_.position.z() = msg->pose.pose.position.z;
    latest_state_estimate_.velocity.x() = msg->twist.twist.linear.x;
    latest_state_estimate_.velocity.y() = msg->twist.twist.linear.y;
    latest_state_estimate_.velocity.z() = msg->twist.twist.linear.z;
    latest_state_estimate_.orientation = Eigen::Quaterniond(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    latest_state_estimate_.bodyrates.x() = msg->twist.twist.angular.x;
    latest_state_estimate_.bodyrates.y() = msg->twist.twist.angular.y;
    latest_state_estimate_.bodyrates.z() = msg->twist.twist.angular.z;
    has_state_estimate_ = true;
  }

  void targetStateEstimateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!targetStateMatchesCurrent(*msg)) {
      ROS_WARN_THROTTLE(
          1.0,
          "ISB-NMPC bridge: ignoring target estimate %s while current target id=%ld.",
          msg->child_frame_id.c_str(), static_cast<long>(current_target_id_));
      return;
    }
    latest_target_position_ = Eigen::Vector3f(
        static_cast<float>(msg->pose.pose.position.x),
        static_cast<float>(msg->pose.pose.position.y),
        static_cast<float>(msg->pose.pose.position.z));
    latest_target_velocity_ = Eigen::Vector3f(
        static_cast<float>(msg->twist.twist.linear.x),
        static_cast<float>(msg->twist.twist.linear.y),
        static_cast<float>(msg->twist.twist.linear.z));
    has_target_state_estimate_ = true;
  }

  void fsmStateCallback(const quadrotor_msgs::FsmState::ConstPtr& msg) {
    if (dry_run_) {
      return;
    }

    last_fsm_state_time_ = ros::Time::now();
    if (msg->state_name != "EXEC_INTERCEPTION") {
      watchdog_latched_ = false;
    }

    const bool was_interception_active = interception_active_;
    interception_active_ = !watchdog_latched_ &&
                           msg->state_name == "EXEC_INTERCEPTION";
    if (interception_active_ && !was_interception_active) {
      activation_time_ = ros::Time::now();
      has_last_active_command_ = false;
      controller_.resetForActivation();
      ROS_INFO("ISB-NMPC bridge: EXEC_INTERCEPTION received, enabling MPC.");
    } else if (!interception_active_ && was_interception_active) {
      has_last_active_command_ = false;
      ROS_INFO("ISB-NMPC bridge: left EXEC_INTERCEPTION, publishing safe empty commands.");
      if (restart_after_interception_) {
        ROS_INFO("ISB-NMPC bridge: restarting after interception to reset the generated solver.");
        ros::shutdown();
      }
    }
  }

  void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg) {
    mavros_state_ = *msg;
    has_mavros_state_ = true;
    last_mavros_state_time_ = ros::Time::now();
  }

  void requestWatchdogStabilized(const ros::Time& now) {
    const bool state_fresh = has_mavros_state_ &&
        (now - last_mavros_state_time_).toSec() <= 0.5;
    if ((state_fresh &&
         (!mavros_state_.armed || mavros_state_.mode != "OFFBOARD")) ||
        (!last_watchdog_mode_request_.isZero() &&
         (now - last_watchdog_mode_request_).toSec() < 0.2)) {
      return;
    }
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = "STABILIZED";
    const bool sent = set_mode_client_.call(mode_cmd) &&
                      mode_cmd.response.mode_sent;
    last_watchdog_mode_request_ = now;
    if (!sent) {
      ROS_ERROR_THROTTLE(1.0,
                         "ISB-NMPC bridge: watchdog STABILIZED request failed.");
    }
  }

  void reloadParamsCallback(const std_msgs::Empty::ConstPtr&) {
    if (params_.loadParameters(pnh_)) {
      ROS_INFO("So3MpcBridgeNode: SO3 MPC params reloaded.");
    } else {
      ROS_WARN("So3MpcBridgeNode: failed to reload SO3 MPC params.");
    }
  }

  void qImageCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg) {
    if (!qImageMatchesCurrent(*msg)) {
      ROS_WARN_THROTTLE(
          1.0,
          "ISB-NMPC bridge: ignoring q_image frame=%s while current target id=%ld.",
          msg->header.frame_id.c_str(), static_cast<long>(current_target_id_));
      return;
    }
    latest_q_image_ = Eigen::Quaternionf(
        static_cast<float>(msg->quaternion.w),
        static_cast<float>(msg->quaternion.x),
        static_cast<float>(msg->quaternion.y),
        static_cast<float>(msg->quaternion.z));
    latest_q_image_stamp_ =
        msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    has_q_image_ = true;
  }

  void targetInMainViewCallback(const std_msgs::Bool::ConstPtr& msg) {
    target_in_main_view_ = msg->data;
    if (msg->data) {
      last_target_in_view_time_ = ros::Time::now();
    }
  }

  void currentTargetCallback(const swarm_msgs::MassPoints::ConstPtr& msg) {
    // In bearing-only mode target_manager intentionally publishes an empty
    // active-target list at 10 Hz. It is not a visual target-loss signal and
    // must not clear q_image between camera/filter updates.
    if (!has_target_pos_) {
      return;
    }
    if (msg->points.empty()) {
      if (current_target_id_ >= 0) {
        ROS_INFO("ISB-NMPC bridge: current target cleared.");
      }
      current_target_id_ = -1;
      has_last_active_command_ = false;
      resetTargetInputs();
      return;
    }

    const int64_t target_id = msg->points.front().volume;
    if (target_id == current_target_id_) {
      return;
    }
    current_target_id_ = target_id;
    has_last_active_command_ = false;
    resetTargetInputs();
    ROS_INFO("ISB-NMPC bridge: current target switched to id=%ld.",
             static_cast<long>(current_target_id_));
  }

  // ==========================================================================
  // 3. Publishers （发布/输出）
  // ==========================================================================
  void publishAttitudeTarget(const quadrotor_common::ControlCommand& command) {
    mavros_msgs::AttitudeTarget target;
    target.header.stamp = command.timestamp;
    target.header.frame_id = "base_link";
    if (command.control_mode == quadrotor_common::ControlMode::ATTITUDE) {
      target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                         mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                         mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
      Eigen::Quaterniond q = Eigen::Quaterniond(
          Eigen::AngleAxisd(control_frame_yaw_, Eigen::Vector3d::UnitZ())) * command.orientation;
      if (q.norm() > 1e-6) {
        q.normalize();
      } else {
        q = Eigen::Quaterniond::Identity();
      }
      target.orientation.w = q.w();
      target.orientation.x = q.x();
      target.orientation.y = q.y();
      target.orientation.z = q.z();
      target.body_rate.x = 0.0;
      target.body_rate.y = 0.0;
      target.body_rate.z = 0.0;
    } else {
      target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
      target.orientation.w = 1.0;
      target.orientation.x = 0.0;
      target.orientation.y = 0.0;
      target.orientation.z = 0.0;
      target.body_rate.x = command.bodyrates.x();
      target.body_rate.y = command.bodyrates.y();
      target.body_rate.z = command.bodyrates.z();
    }

    const double normalized_thrust =
        (command.collective_thrust / hover_acceleration_) * hover_thrust_;
    target.thrust = std::max(0.0, std::min(1.0, normalized_thrust));
    attitude_target_pub_.publish(target);
  }

  void publishHoverAttitudeTarget(const ros::Time& timestamp) {
    quadrotor_common::ControlCommand command;
    command.zero();
    command.timestamp = timestamp;
    command.expected_execution_time = timestamp;
    command.armed = true;
    command.control_mode = quadrotor_common::ControlMode::ATTITUDE;
    command.collective_thrust = hover_acceleration_;
    command.orientation = makeLevelOrientationPreservingYaw(
        latest_state_estimate_.orientation);
    control_cmd_pub_.publish(command.toRosMessage());
    if (!dry_run_) {
      publishAttitudeTarget(command);
    }
  }

  void publishSafeEmptyCommand(const ros::Time& timestamp) {
    quadrotor_common::ControlCommand command;
    command.zero();
    command.timestamp = timestamp;
    command.expected_execution_time = timestamp;
    control_cmd_pub_.publish(command.toRosMessage());
    // When not in EXEC_INTERCEPTION, suppress attitude setpoints so the FCU
    // setpoint stream is owned by the FSM position interface (IDLE takeoff, hover, etc.).
    // （非EXEC_INTERCEPTION时抑制attitude setpoint，将FCU的setpoint流让给FSM的position接口）
  }

  // Keep the FCU attitude stream continuous across a brief visual dropout.
  // A longer loss falls back to a level hover command rather than an empty
  // command that leaves the FCU without a fresh setpoint.
  void publishActiveFallback(const ros::Time& timestamp) {
    if (has_last_active_command_ &&
        (timestamp - last_active_command_time_).toSec() <=
            target_dropout_hold_sec_) {
      quadrotor_common::ControlCommand command = last_active_command_;
      command.timestamp = timestamp;
      command.expected_execution_time = timestamp;
      control_cmd_pub_.publish(command.toRosMessage());
      if (!dry_run_) {
        publishAttitudeTarget(command);
      }
      return;
    }
    publishHoverAttitudeTarget(timestamp);
  }

  // ==========================================================================
  // 4. Core Control Loop （核心控制逻辑）
  // ==========================================================================
  // Validate target vectors only in position-based interception mode.
  //（仅在基于位置的拦截模式中校验目标向量）
  static bool isFiniteVector(const Eigen::Vector3f& vector) {
    return std::isfinite(vector.x()) && std::isfinite(vector.y()) &&
           std::isfinite(vector.z());
  }

  static Eigen::Quaterniond makeLevelOrientationPreservingYaw(
      const Eigen::Quaterniond& orientation) {
    Eigen::Quaterniond q = orientation;
    if (q.norm() > 1e-6) {
      q.normalize();
    } else {
      q = Eigen::Quaterniond::Identity();
    }
    const Eigen::Matrix3d rotation = q.toRotationMatrix();
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  }

  // Reject stale target IDs only in position-based interception mode.
  //（仅在基于位置的拦截模式中过滤旧目标ID）
  bool targetStateMatchesCurrent(const nav_msgs::Odometry& msg) const {
    if (!has_target_pos_) {
      return true;
    }
    if (current_target_id_ < 0 || msg.child_frame_id.empty()) {
      return true;
    }
    return msg.child_frame_id ==
           ("target_" + std::to_string(current_target_id_));
  }

  // Enforce target-tagged image rays only in position-based interception mode.
  //（仅在基于位置的拦截模式中校验带目标标识的图像视线）
  bool qImageMatchesCurrent(const geometry_msgs::QuaternionStamped& msg) const {
    if (!has_target_pos_) {
      return true;
    }
    if (current_target_id_ < 0 || msg.header.frame_id.empty() ||
        msg.header.frame_id == "base_link") {
      return true;
    }
    return msg.header.frame_id ==
           ("target_" + std::to_string(current_target_id_));
  }

  void resetTargetInputs() {
    has_target_state_estimate_ = false;
    target_in_main_view_ = false;
    last_target_in_view_time_ = ros::Time();
    latest_target_position_.setZero();
    latest_target_velocity_.setZero();
    latest_q_image_ = Eigen::Quaternionf::Identity();
    latest_q_image_stamp_ = ros::Time();
    has_q_image_ = false;
  }

  bool hasFreshQImage(const ros::Time& now) const {
    return has_q_image_ && !latest_q_image_stamp_.isZero() &&
           (now - latest_q_image_stamp_).toSec() <= q_image_timeout_;
  }

  void controlLoop(const ros::TimerEvent&) {
    const ros::Time command_time = ros::Time::now();
    const bool fsm_stale = last_fsm_state_time_.isZero()
                               ? (command_time - startup_time_).toSec() >
                                     fsm_state_timeout_
                               : (command_time - last_fsm_state_time_).toSec() >
                                     fsm_state_timeout_;
    // Give a new EXEC activation one watchdog interval for the first state
    // callback; ROS topics have no cross-topic delivery order.
    const ros::Time uav_watchdog_anchor =
        last_uav_state_time_ > activation_time_ ? last_uav_state_time_
                                                 : activation_time_;
    const bool uav_stale = !uav_watchdog_anchor.isZero() &&
                           (command_time - uav_watchdog_anchor).toSec() >
                               uav_state_timeout_;
    if (!dry_run_ &&
        (watchdog_latched_ ||
         (fsm_stale &&
          (interception_active_ ||
           (has_mavros_state_ &&
            (command_time - last_mavros_state_time_).toSec() <= 0.5 &&
            mavros_state_.armed &&
            mavros_state_.mode == "OFFBOARD"))) ||
         (interception_active_ && uav_stale))) {
      if (!watchdog_latched_) {
        ROS_ERROR("ISB-NMPC bridge: %s stale; suppressing attitude output "
                  "and requesting STABILIZED.",
                  fsm_stale ? "FSM state" : "UAV state");
      }
      watchdog_latched_ = true;
      interception_active_ = false;
      has_last_active_command_ = false;
      publishSafeEmptyCommand(command_time);
      requestWatchdogStabilized(command_time);
      return;
    }
    if (!has_state_estimate_) {
      return;
    }

    quadrotor_common::QuadStateEstimate state = latest_state_estimate_;
    if (!state.isValid()) {
      if (!dry_run_ && interception_active_) {
        watchdog_latched_ = true;
        interception_active_ = false;
        has_last_active_command_ = false;
        publishSafeEmptyCommand(command_time);
        requestWatchdogStabilized(command_time);
        ROS_ERROR("ISB-NMPC bridge: invalid state estimate; "
                  "attitude output stopped and STABILIZED requested.");
      } else {
        ROS_WARN_THROTTLE(1.0, "ISB-NMPC bridge: invalid state estimate.");
      }
      return;
    }

    if (!interception_active_) {
      ROS_INFO_THROTTLE(
          1.0,
          "ISB-NMPC bridge: waiting for EXEC_INTERCEPTION, publishing safe empty command.");
      publishSafeEmptyCommand(command_time);
      return;
    }

    if (has_target_pos_ && !has_target_state_estimate_) {
      ROS_WARN_THROTTLE(
          1.0,
          "ISB-NMPC bridge: waiting for target state estimate.");
      publishActiveFallback(command_time);
      return;
    }

    if (!has_target_pos_ && !hasFreshQImage(command_time)) {
      ROS_WARN_THROTTLE(
          1.0,
          "ISB-NMPC bridge: waiting for fresh q_image in bearing-only mode.");
      publishActiveFallback(command_time);
      return;
    }

    const bool target_seen_recently =
        !last_target_in_view_time_.isZero() &&
        (command_time - last_target_in_view_time_).toSec() <= q_image_timeout_;
    if (!has_target_pos_ && require_target_in_view_ &&
        !target_seen_recently) {
      ROS_WARN_THROTTLE(
          1.0,
          "ISB-NMPC bridge: target is not in main view, holding bearing-only control.");
      publishHoverAttitudeTarget(command_time);
      return;
    }

    if (has_target_pos_ && (!isFiniteVector(latest_target_position_) ||
                            !isFiniteVector(latest_target_velocity_))) {
      ROS_WARN_THROTTLE(1.0,
                        "ISB-NMPC bridge: invalid target estimate, skipping MPC.");
      publishActiveFallback(command_time);
      return;
    }

    const double activation_age =
        activation_time_.isZero()
            ? std::numeric_limits<double>::infinity()
            : (command_time - activation_time_).toSec();
    if (activation_age < activation_hover_sec_) {
      publishHoverAttitudeTarget(command_time);
      return;
    }

    const Eigen::Vector3f target_position =
        has_target_pos_
            ? latest_target_position_
            : Eigen::Vector3f::Constant(
                  std::numeric_limits<float>::quiet_NaN());
    const Eigen::Vector3f target_velocity =
        has_target_pos_
            ? latest_target_velocity_
            : Eigen::Vector3f::Constant(
                  std::numeric_limits<float>::quiet_NaN());
    const quadrotor_common::ControlCommand command =
        controller_.run(state, params_, target_position, target_velocity,
                        latest_q_image_, target_in_main_view_);

    if (command.armed &&
        command.control_mode != quadrotor_common::ControlMode::NONE) {
      last_active_command_ = command;
      last_active_command_time_ = command_time;
      has_last_active_command_ = true;
    }

    control_cmd_pub_.publish(command.toRosMessage());
    if (!dry_run_) {
      publishAttitudeTarget(command);
    }
  }

  // ==========================================================================
  // 5. Member Variables （成员变量）
  // ==========================================================================
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  So3MpcController<float> controller_;
  So3MpcParams<float> params_;

  ros::Subscriber uav_state_estimate_sub_;
  ros::Subscriber target_state_estimate_sub_;
  ros::Subscriber q_image_sub_;
  ros::Subscriber target_in_main_view_sub_;
  ros::Subscriber current_target_sub_;
  ros::Subscriber fsm_state_sub_;
  ros::Subscriber mavros_state_sub_;
  ros::Subscriber reload_params_sub_;
  ros::Publisher control_cmd_pub_;
  ros::Publisher attitude_target_pub_;
  ros::Subscriber local_frame_yaw_sub_;
  double control_frame_yaw_{0.0};
  ros::ServiceClient set_mode_client_;
  ros::Timer control_timer_;

  quadrotor_common::QuadStateEstimate latest_state_estimate_;
  Eigen::Vector3f latest_target_position_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f latest_target_velocity_{Eigen::Vector3f::Zero()};
  Eigen::Quaternionf latest_q_image_{Eigen::Quaternionf::Identity()};

  bool has_state_estimate_{false};
  bool has_target_state_estimate_{false};
  bool has_q_image_{false};
  bool has_target_pos_{true};
  bool require_target_in_view_{true};
  bool target_in_main_view_{false};
  bool interception_active_{false};
  bool watchdog_latched_{false};
  bool has_mavros_state_{false};
  mavros_msgs::State mavros_state_;
  ros::Time startup_time_;
  ros::Time last_fsm_state_time_;
  ros::Time last_uav_state_time_;
  ros::Time last_mavros_state_time_;
  ros::Time last_watchdog_mode_request_;
  bool has_last_active_command_{false};
  quadrotor_common::ControlCommand last_active_command_;
  ros::Time last_active_command_time_;
  int64_t current_target_id_{-1};
  ros::Time activation_time_;
  ros::Time latest_q_image_stamp_;
  ros::Time last_target_in_view_time_;
  double control_rate_hz_{50.0};
  double hover_thrust_{0.5};
  double hover_acceleration_{9.81};
  double activation_hover_sec_{0.25};
  double q_image_timeout_{0.3};
  double target_dropout_hold_sec_{0.15};
  double fsm_state_timeout_{0.3};
  double uav_state_timeout_{0.3};
  std::string attitude_target_topic_{"/nmpc/attitude_target"};
  std::string control_command_topic_{"/isb_nmpc/control_command"};
  std::string uav_state_estimate_topic_{"/nmpc/uavStateEstimate"};
  std::string target_state_estimate_topic_{"/nmpc/targetStateEstimate"};
  std::string q_image_topic_{"/nmpc/q_image"};
  std::string target_in_main_view_topic_{"/nmpc/target_in_main_view"};
  std::string current_target_topic_{"/target_manager/active_target"};
  std::string fsm_state_topic_{"/fsm/fsm_state"};
  std::string reload_params_topic_{"/isb_nmpc/reload_params"};
  bool dry_run_{false};
  bool restart_after_interception_{false};
};

}  // namespace isb_nmpc

// ============================================================================
// 6. Entry Point （入口函数）
// ============================================================================
int main(int argc, char** argv) {
  ros::init(argc, argv, "so3_mpc_bridge_node");
  isb_nmpc::So3MpcBridgeNode node;
  ros::spin();
  return 0;
}
