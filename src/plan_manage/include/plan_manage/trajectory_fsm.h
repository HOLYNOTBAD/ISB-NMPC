#pragma once

#include <string>

#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/Trajectory.h>
#include <quadrotor_msgs/TrajectoryPoint.h>
#include <ros/ros.h>

namespace plan_manage {

class TrajectoryFSM {
 public:
  // Tracking state machine. （轨迹跟踪状态机）
  enum class TrackingState { NORMAL, HOLD, LOST };

  // Active trajectory execution context. （活动轨迹执行上下文）
  struct Context {
    bool has_traj{false};
    bool horizon_valid{false};
    bool is_finished{false};
    bool tracking{true};
    TrackingState tracking_state{TrackingState::NORMAL};
    ros::Time start_time;
    ros::Time last_publish_time;
    ros::Time hold_start_time;
    ros::Time lost_exceeded_since;
    double accumulated_hold_time{0.0};
    double total_duration{0.0};
    double current_time{0.0};
    quadrotor_msgs::Trajectory global_traj;
    quadrotor_msgs::Trajectory horizon_traj;
  };

  // Utils （工具函数）
  static quadrotor_msgs::TrajectoryPoint makeZeroPoint();
  // Fill a trajectory point with current UAV position, velocity, orientation.
  // （用当前UAV位置、速度、姿态填充轨迹点）
  static quadrotor_msgs::TrajectoryPoint makeCurrentStatePoint(
      const nav_msgs::Odometry& uav_state);
  static void clearDerivatives(quadrotor_msgs::TrajectoryPoint* point);

  // Trajectory Interface （轨迹接口）
  void loadParams(ros::NodeHandle& nh, const std::string& prefix);
  void initVisualization(ros::NodeHandle& nh,
                         const std::string& global_topic,
                         const std::string& horizon_topic);

  void reset();
  // Load trajectory into memory without starting execution.
  // （加载轨迹但不开始执行）
  void load(const quadrotor_msgs::Trajectory& trajectory);
  // Start executing the active trajectory from a wall-clock time.
  // （从给定墙钟时间开始执行）
  void execute(const ros::Time& start_time);
  // Advance trajectory progress by wall clock and correct tracking state from
  // current position. （按墙钟时间推进轨迹执行进度，并结合当前位置修正）
  void update(const ros::Time& now, const nav_msgs::Odometry& uav_state);
  // Extract the horizon window from global_traj.
  // （从global_traj截取horizon时间窗）
  void buildHorizon();
  // Publish the previously built horizon trajectory.
  // （发布已构建的horizon_traj）
  void publishHorizon(ros::Publisher& publisher);
  void visualize() const;
  // Replace the tail of global_traj with a replanned trajectory segment
  // without resetting the execution clock.
  // （不重置执行时钟，使用重规划轨迹段替换 global_traj 尾部）
  void updateGlobal(const quadrotor_msgs::Trajectory& replanned_trajectory);

  const Context& context() const { return context_; }
  Context& context() { return context_; }

 private:
  Context context_;
  double horizon_duration_{2.0};
  double sample_dt_{0.1};
  int horizon_points_{20};
  double hold_enter_distance_{0.3};
  double hold_exit_distance_{0.2};
  double lost_distance_{0.5};
  double lost_confirmation_sec_{0.15};
  double max_hold_sec_{2.0};
  bool hold_enabled_{true};
  ros::Publisher global_vis_pub_;
  ros::Publisher horizon_vis_pub_;
};

}  // namespace plan_manage
