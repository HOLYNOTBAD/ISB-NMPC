#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include "plan_manage/fsm_com.h"

namespace plan_manage {

class InterceptionFSM : public FSMCom {
 public:
  enum FSMExecState {
    IDLE = 0,
    WAIT_TARGET,
    EXEC_INTERCEPTION,
    HOLD,
    FINISH_HOLD,
    RETURN_TO_START
  };

  enum FinishHoldPhase {
    FINISH_BRAKE = 0,
    FINISH_POSITION_HOLD
  };

  enum HoldPhase {
    HOLD_BRAKE = 0,
    HOLD_POSITION_HOLD
  };

  InterceptionFSM() = default;
  void init(ros::NodeHandle& nh);

 private:
  const char* stateName(FSMExecState state) const;
  void targetStateEstimateCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void targetInMainViewCallback(const std_msgs::Bool::ConstPtr& msg);
  void detectionModeCallback(const std_msgs::String::ConstPtr& msg);
  void interceptedCallback(const std_msgs::Bool::ConstPtr& msg);
  void nmpcFailureCallback(const std_msgs::Bool::ConstPtr& msg);
  void execFSMCallback(const ros::TimerEvent& event);
  void onEnterConfirmation() override;
  bool resetTrialCallback(std_srvs::Trigger::Request& req,
                          std_srvs::Trigger::Response& resp);
  bool targetAvailable(bool allow_visual_grace = false,
                       double visual_timeout = -1.0) const;
  double distanceToFirstTarget() const;
  void publishBalloonMotionTrigger(bool trigger);
  void beginHold(const char* reason);
  void runHold();
  void beginFinishHold(const char* reason);
  void runFinishHold();
  void beginReturnToStart();
  void clearTrialState();
  bool runFailsafe();
  void beginStabilizedFailsafe(const char* reason);
  void runLatchedFailsafe();
  bool normalizedAttitude(geometry_msgs::Quaternion* attitude) const;
  bool localStateFinite() const;
  double rollAngleRad(const geometry_msgs::Quaternion& attitude) const;
  void resetFailsafe();

  ros::NodeHandle nh_;
  ros::Subscriber target_state_estimate_sub_;
  ros::Subscriber target_in_main_view_sub_;
  ros::Subscriber detection_mode_sub_;
  ros::Subscriber intercepted_sub_;
  ros::Subscriber nmpc_failure_sub_;
  ros::Publisher balloon_motion_trigger_pub_;
  ros::ServiceServer reset_trial_srv_;
  ros::Timer exec_timer_;
  std::string motion_trigger_topic_{"/fsm/motion_trigger"};
  std::string intercepted_topic_{"/fsm/intercepted"};
  std::string nmpc_failure_topic_{"/nmpc/failure"};
  std::string detection_mode_topic_{"/target_detection/mode"};
  ros::Time last_target_state_estimate_time_;
  ros::Time last_target_in_view_time_;
  double target_state_estimate_timeout_{0.5};
  double q_image_timeout_{0.3};
  double terminal_lost_timeout_{0.05};
  double terminal_mode_memory_{0.5};
  ros::Time last_track_lr_time_;
  std::string target_in_main_view_topic_{"/nmpc/target_in_main_view"};

  FSMExecState exec_state_{IDLE};
  bool has_object_target_{false};
  bool has_target_pos_{true};
  bool target_in_main_view_{false};
  bool intercepted_{false};
  bool nmpc_failure_received_{false};
  bool failsafe_latched_{false};
  bool failsafe_offboard_exit_observed_{false};
  std::string failsafe_reason_;
  double failsafe_max_abs_roll_rad_{1.3962634015954636};
  geometry_msgs::Point hold_position_;
  double hold_yaw_{0.0};

  HoldPhase hold_phase_{HOLD_BRAKE};
  ros::Time hold_brake_start_time_;
  ros::Time hold_brake_below_speed_since_;
  double hold_brake_speed_threshold_{0.3};
  double hold_brake_settle_time_{0.3};
  double hold_brake_timeout_{8.0};

  double position_tolerance_{0.3};
  enum class StartMode { GROUND_TAKEOFF, AIRBORNE_TAKEOVER };
  StartMode start_mode_{StartMode::GROUND_TAKEOFF};
  bool airborneTakeover() const { return start_mode_ == StartMode::AIRBORNE_TAKEOVER; }
  bool initializationReady();
  void resetInitializationWindow();
  ros::Time initialization_stable_since_;
  geometry_msgs::Point initialization_anchor_;
  double initialization_yaw_{0.0};
  double initialization_settle_sec_{0.5};
  double initialization_max_speed_{0.3};
  double initialization_max_position_change_{0.2};
  double initialization_max_yaw_change_rad_{0.0872664625997165};
  bool initialization_ready_{false};
  bool trial_frame_locked_{false};
  bool trial_was_armed_{false};
  std::atomic<double> last_enter_time_{0.0};
  double offboard_takeover_hold_sec_{1.0};
  ros::Time offboard_takeover_start_time_;
  bool offboard_takeover_pending_{false};
  geometry_msgs::Point return_cruise_position_;
  bool return_to_start_altitude_{false};
  bool last_balloon_motion_trigger_{false};

  FinishHoldPhase finish_hold_phase_{FINISH_BRAKE};
  ros::Time finish_brake_start_time_;
  ros::Time finish_brake_below_speed_since_;
  double finish_brake_speed_threshold_{0.3};
  double finish_brake_settle_time_{0.3};
  double finish_brake_timeout_{8.0};

};

}  // namespace plan_manage
