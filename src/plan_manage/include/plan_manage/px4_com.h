#pragma once

#include <functional>
#include <atomic>
#include <optional>
#include <string>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/EstimatorStatus.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/Trajectory.h>
#include <quadrotor_msgs/TrajectoryPoint.h>
#include <ros/ros.h>

namespace plan_manage {

class Px4Com {
 public:
  struct Context {
    bool* has_local_pose{nullptr};
    bool* has_local_velocity{nullptr};
    nav_msgs::Odometry* uav_state{nullptr};
    nav_msgs::Odometry* start_state{nullptr};
    double* position_tolerance{nullptr};
    std::string* idle_frame_id{nullptr};
    bool* require_enter_confirmation{nullptr};
    std::atomic<bool>* exec_permission_granted{nullptr};
    std::function<void()> publish_empty_reference_trajectory;
  };

  Px4Com() = default;
  ~Px4Com();

  void bindContext(const Context& context);
  void initRosInterfaces(ros::NodeHandle& nh);
  void loadParams(ros::NodeHandle& nh, const std::string& prefix);

  void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
  void estimatorStatusCallback(
      const mavros_msgs::EstimatorStatus::ConstPtr& msg);
  void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void localVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void updateMavrosState(const mavros_msgs::State& msg);
  void updateLocalPose(const geometry_msgs::PoseStamped& msg,
                       const std::string& child_frame_id = "base_link");
  void updateLocalVelocity(const geometry_msgs::TwistStamped& msg);
  bool resetLocalOriginToLatestPose();
  // Interception locks its frame explicitly; other FSMs retain their policy.
  void deferLocalFrameCapture();
  bool localFrameLocked() const { return has_local_origin_ && has_initial_yaw_; }
  void publishMeasuredPositionSetpoint();

  geometry_msgs::Point toControlFrame(
      const geometry_msgs::Point& mavros_local_position) const;
  geometry_msgs::Point toMavrosLocalFrame(
      const geometry_msgs::Point& control_frame_position) const;
  geometry_msgs::Vector3 toControlFrame(
      const geometry_msgs::Vector3& mavros_local_vector) const;
  geometry_msgs::Vector3 toMavrosLocalFrame(
      const geometry_msgs::Vector3& control_frame_vector) const;
  geometry_msgs::Quaternion toControlFrame(
      const geometry_msgs::Quaternion& mavros_local_orientation) const;
  double toMavrosLocalYaw(double control_frame_yaw) const;
  void captureInitialYawIfNeeded(const geometry_msgs::PoseStamped& msg);
  void publishLocalOrigin() const;
  bool requestOffboardMode();
  bool requestPreparedOffboardMode();
  // Emergency attitude-only fallback used when the external position estimate
  // is no longer trustworthy.
  bool requestStabilizedMode();
  bool requestPositionMode();
  // Ask PX4 to leave OFFBOARD and perform its native automatic landing.
  bool requestAutoLandMode();
  bool armVehicle();
  void captureTakeoffAnchorIfUnset(const geometry_msgs::PoseStamped& msg);
  // Return whether at least one /mavros/state message has been received. （是否已收到至少一帧/mavros/state）
  bool hasMavrosState() const;
  bool mavrosStateFresh() const;
  const mavros_msgs::State& mavrosState() const;
  bool localPositionEstimateReliable() const;
  bool offboardEntryRequested() const;
  void setOffboardEntryRequested(bool requested);
  void resetIdleVelocitySetpoint();

  // Guide to a target pose by publishing velocity and yaw-rate setpoints. （通过速度和yaw角速度setpoint引导到目标位姿）
  void publishPoseSetpoint(const geometry_msgs::Point& position, double yaw,
                           bool count_warmup = false);
  // Publish an absolute PX4 position setpoint without the outer velocity loop.
  // This is used during takeoff so PX4 locks XY while climbing.
  void publishPositionSetpoint(const geometry_msgs::Point& position, double yaw);
  // Send position, velocity and acceleration feed-forward to PX4's native
  // multicopter position controller through MAVROS SET_POSITION_TARGET_LOCAL_NED.
  void publishPVASetpoint(
      const quadrotor_msgs::TrajectoryPoint& point);
  // Publish a velocity setpoint in the map/local frame. （发布map/local坐标系下的速度setpoint）
  void publishVelSetpoint(const geometry_msgs::Vector3& velocity);
  // Publish a persistent hover setpoint until resetHoverSetpoint() is called. （发布持续悬停setpoint，直到调用resetHoverSetpoint()）
  void publishHoverSetpoint(const geometry_msgs::Point& position, double yaw);
  void resetHoverSetpoint();
  
  void publishWarmUpPositionSetpoint();
  bool warmupSetpointsReady() const;
  bool ensureLocalPoseReceived(const char* wait_context) const;
  bool ensureLocalVelocityReceived(const char* wait_context) const;
  void publishWaitingEnterPrompt(const char* state_name) const;
  void resetConfirmationGate();

  Context context_;
  ros::Subscriber mavros_state_sub_;
  ros::Subscriber estimator_status_sub_;
  ros::Subscriber local_pose_sub_;
  ros::Subscriber local_velocity_sub_;
  ros::Publisher position_setpoint_pub_;
  ros::Publisher velocity_setpoint_pub_;
  ros::Publisher raw_local_setpoint_pub_;
  ros::Publisher local_origin_pub_;
  ros::Publisher local_yaw_pub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  bool has_mavros_state_{false};
  mavros_msgs::State mavros_state_;
  ros::Time last_mavros_state_time_;
  double mavros_state_timeout_{2.5};
  mavros_msgs::EstimatorStatus estimator_status_;
  bool has_estimator_status_{false};
  ros::Time last_estimator_status_time_;
  double estimator_status_timeout_{2.5};
  ros::Time last_local_pose_time_;
  ros::Time last_local_velocity_time_;
  double local_state_timeout_{0.5};
  int warmup_setpoint_count_{0};
  int warmup_setpoints_required_{50};
  double mode_request_interval_{2.0};
  double arm_request_interval_{2.0};
  double idle_vertical_speed_{0.6};
  double idle_horizontal_speed_{1.0};
  double idle_yaw_rate_{0.5};
  double idle_horizontal_kp_{1.0};
  double idle_horizontal_kd_{0.4};
  double idle_horizontal_ki_{0.0};
  double idle_horizontal_i_limit_{1.0};
  double idle_vertical_kp_{1.0};
  double idle_vertical_kd_{0.4};
  double idle_yaw_kp_{1.0};
  double idle_horizontal_accel_{2.0};
  double idle_vertical_accel_{1.5};
  double idle_velocity_bias_x_{0.0};
  double idle_velocity_bias_y_{0.0};
  bool defer_local_frame_capture_{false};
  bool use_first_pose_as_origin_{false};
  bool rotate_by_initial_yaw_{false};
  bool has_local_origin_{false};
  bool has_initial_yaw_{false};
  double initial_yaw_{0.0};
  geometry_msgs::Point local_origin_;
  std::optional<geometry_msgs::PoseStamped> latest_local_pose_;
  std::optional<geometry_msgs::TwistStamped> latest_local_velocity_;
  bool has_takeoff_xy_anchor_{false};
  std::optional<geometry_msgs::Point> takeoff_anchor_;
  std::optional<geometry_msgs::Point> hover_setpoint_position_;
  std::optional<double> hover_setpoint_yaw_;
  std::optional<geometry_msgs::Vector3> last_idle_velocity_setpoint_;
  ros::Time last_idle_velocity_setpoint_time_;
  geometry_msgs::Vector3 idle_position_i_error_;
  ros::Time last_mode_request_;
  ros::Time last_arm_request_;
  bool offboard_entry_requested_{false};
};

}  // namespace plan_manage
