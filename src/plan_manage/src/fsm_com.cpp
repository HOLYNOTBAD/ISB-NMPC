#include "plan_manage/fsm_com.h"

#include <iostream>

namespace plan_manage {

FSMCom::FSMCom() {
  Px4Com::Context context;
  context.has_local_pose = &has_local_pose_;
  context.has_local_velocity = &has_local_velocity_;
  context.uav_state = &uav_state_;
  context.start_state = &start_state_;
  context.position_tolerance = &position_tolerance_;
  context.idle_frame_id = &idle_frame_id_;
  context.require_enter_confirmation = &require_enter_confirmation_;
  context.exec_permission_granted = &exec_permission_granted_;
  context.publish_empty_reference_trajectory =
      [this]() { publishEmptyHorizonTraj(); };
  px4_com_.bindContext(context);
}

FSMCom::~FSMCom() = default;

// ============================================================================
//  Utils （工具函数）
// ============================================================================

void FSMCom::loadCommonParams(ros::NodeHandle& nh,
                              const std::string& prefix) {
  px4_com_.loadParams(nh, prefix);
  trajectory_fsm_.loadParams(nh, prefix);

  nh.param(prefix + "/exec/require_enter_confirmation",
           require_enter_confirmation_, false);
  nh.param<std::string>(prefix + "/idle/frame_id", idle_frame_id_,
                        idle_frame_id_);
  nh.param(prefix + "/idle/start_x", start_x_, 0.0);
  nh.param(prefix + "/idle/start_y", start_y_, 0.0);
  nh.param(prefix + "/idle/start_z", start_z_, 1.5);
  nh.param<std::string>(prefix + "/state_estimate/topic",
                        uav_state_estimate_topic_, uav_state_estimate_topic_);
  nh.param<std::string>(prefix + "/target_state_estimate/topic",
                        target_state_estimate_topic_,
                        target_state_estimate_topic_);
  nh.param<std::string>(prefix + "/state_estimate/fsm_state_topic",
                        fsm_state_topic_, fsm_state_topic_);
  nh.param<std::string>(prefix + "/planning/reference_topic",
                        reference_traj_topic_, reference_traj_topic_);
  nh.param<std::string>(prefix + "/planning/active_global_traj_vis_topic",
                        active_global_traj_vis_topic_,
                        active_global_traj_vis_topic_);
  nh.param<std::string>(prefix + "/planning/active_horizon_traj_vis_topic",
                        active_horizon_traj_vis_topic_,
                        active_horizon_traj_vis_topic_);
}

// ============================================================================
//  Common FSM/ROS Interface Implementations （公共FSM/ROS接口实现）
// ============================================================================
// Publish FSM state, UAV state estimate, reference trajectory, etc.
// （发布FSM状态、无人机状态估计、参考轨迹等）
void FSMCom::initCommonRosInterfaces(ros::NodeHandle& nh) {
  px4_com_.initRosInterfaces(nh);
  uav_state_estimate_pub_ =
      nh.advertise<nav_msgs::Odometry>(uav_state_estimate_topic_, 10);
  fsm_state_pub_ =
      nh.advertise<quadrotor_msgs::FsmState>(fsm_state_topic_, 10);
  reference_traj_pub_ =
      nh.advertise<quadrotor_msgs::Trajectory>(reference_traj_topic_, 2);
}

// Initialize visualization publishers for global and horizon trajectories.
// （初始化全局与horizon轨迹可视化发布者）
void FSMCom::initCommonFsmInterfaces(ros::NodeHandle& nh) {
  trajectory_fsm_.initVisualization(
      nh, active_global_traj_vis_topic_, active_horizon_traj_vis_topic_);
}

void FSMCom::publishFsmState(const std::string& state_name) {
  quadrotor_msgs::FsmState msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = idle_frame_id_;
  msg.state_name = state_name;
  fsm_state_pub_.publish(msg);
}

void FSMCom::publishUavStateEstimate() {
  if (!has_local_pose_ || !has_local_velocity_) return;

  if (uav_state_.header.frame_id.empty()) {
    uav_state_.header.frame_id = idle_frame_id_;
  }
  if (uav_state_.child_frame_id.empty()) {
    uav_state_.child_frame_id = "base_link";
  }
  uav_state_estimate_pub_.publish(uav_state_);
}

void FSMCom::publishEmptyHorizonTraj() const {
  quadrotor_msgs::Trajectory empty_traj;
  empty_traj.header.stamp = ros::Time::now();
  empty_traj.header.frame_id = idle_frame_id_;
  reference_traj_pub_.publish(empty_traj);
}

// ============================================================================
//  Terminal Interaction （终端交互）
// ============================================================================

void FSMCom::inputLoop() {
  while (ros::ok() && !stop_input_thread_) {
    std::string command;
    if (!std::getline(std::cin, command)) return;
    onEnterConfirmation();
    exec_permission_granted_ = true;
    ROS_INFO("[fsm] received Enter confirmation.");
  }
}

void FSMCom::startInputThread(bool require_enter_confirmation) {
  require_enter_confirmation_ = require_enter_confirmation;
  if (input_thread_.joinable()) return;

  if (require_enter_confirmation) {
    stop_input_thread_ = false;
    input_thread_ = std::thread(&FSMCom::inputLoop, this);
  } else {
    exec_permission_granted_ = true;
  }
}

void FSMCom::stopInputThread() {
  stop_input_thread_ = true;
  if (input_thread_.joinable()) input_thread_.detach();
}

void FSMCom::resetExecPermission(bool granted) {
  exec_permission_granted_ = granted;
}

void FSMCom::onEnterConfirmation() {}

}  // namespace plan_manage
