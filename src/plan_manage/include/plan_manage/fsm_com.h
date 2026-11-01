#pragma once

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/FsmState.h>
#include <quadrotor_msgs/Trajectory.h>
#include <ros/ros.h>

#include "plan_manage/px4_com.h"
#include "plan_manage/trajectory_fsm.h"

namespace plan_manage {

class FSMCom {
 public:
  FSMCom();
  virtual ~FSMCom();

 protected:
  // Utils （工具函数）
  void loadCommonParams(ros::NodeHandle& nh, const std::string& prefix);

  void publishFsmState(const std::string& state_name);


  void publishUavStateEstimate();
  void publishEmptyHorizonTraj() const;

  // Trajectory Interface （轨迹接口）
  TrajectoryFSM trajectory_fsm_;
  
  // UAV state: position, velocity, orientation, angular velocity. （UAV状态：p v q w）
  nav_msgs::Odometry uav_state_;
  nav_msgs::Odometry start_state_;
  // Interception target state buffer. （拦截目标状态缓冲区）
  std::vector<nav_msgs::Odometry> intercept_state_;
  // Target state estimate. （目标状态估计）
  nav_msgs::Odometry target_state_;

  bool has_local_pose_{false};
  bool has_local_velocity_{false};
  double start_x_{0.0};
  double start_y_{0.0};
  double start_z_{1.5};
  double position_tolerance_{0.3};
  std::string idle_frame_id_{"map"};

  // Subscribers and Publishers （ROS话题订阅与发布）
  void initCommonRosInterfaces(ros::NodeHandle& nh);
  void initCommonFsmInterfaces(ros::NodeHandle& nh);
  std::string uav_state_estimate_topic_{"/nmpc/uavStateEstimate"};
  std::string target_state_estimate_topic_{"/nmpc/targetStateEstimate"};
  std::string reference_traj_topic_{"/nmpc/reference"};
  std::string fsm_state_topic_{"/fsm/fsm_state"};
  ros::Publisher uav_state_estimate_pub_;
  ros::Publisher reference_traj_pub_;
  ros::Publisher fsm_state_pub_;
  std::string active_global_traj_vis_topic_{"/planning/active_global_traj_vis"};
  std::string active_horizon_traj_vis_topic_{"/planning/active_horizon_traj_vis"};

  // Interaction — Enter key confirmation and permission gating. （终端交互：回车确认与权限门控）
  bool require_enter_confirmation_{false};
  std::atomic<bool> exec_permission_granted_{false};
  std::atomic<bool> stop_input_thread_{false};
  std::thread input_thread_;
  void inputLoop();
  void startInputThread(bool require_enter_confirmation);
  void stopInputThread();
  void resetExecPermission(bool granted);
  virtual void onEnterConfirmation();
  bool execPermissionGranted() const { return exec_permission_granted_.load(); }
  void setExecPermissionGranted(bool granted) { exec_permission_granted_.store(granted); }
  bool requireEnterConfirmation() const { return require_enter_confirmation_; }

  // PX4 Communication Interface （PX4通信接口）
  Px4Com px4_com_;
};

}  // namespace plan_manage
