#pragma once

// MPC Debug Logging （MPC调试日志）
#include <fstream>
#include <limits>
#include <string>
// MPC Debug Logging End
#include <thread>

#include <Eigen/Eigen>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/quad_state_estimate.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include "isb_nmpc/so3_mpc_params.h"
#include "isb_nmpc/so3_mpc_wrapper.h"

namespace isb_nmpc {

enum STATE {
  kPosX = 0,
  kPosY = 1,
  kPosZ = 2,
  kOriW = 3,
  kOriX = 4,
  kOriY = 5,
  kOriZ = 6,
  kVelX = 7,
  kVelY = 8,
  kVelZ = 9,
  kFocusW = 10,
  kFocusX = 11,
  kFocusY = 12,
  kFocusZ = 13,
  kRange = 14,
  kInterceptSpeed = 15
};

template <typename T>
class So3MpcController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  So3MpcController(const ros::NodeHandle& nh,
                   const ros::NodeHandle& pnh,
                   const std::string& topic = "isb_nmpc/predicted_trajectory_vis");
  So3MpcController() : So3MpcController(ros::NodeHandle(), ros::NodeHandle("~")) {}

  quadrotor_common::ControlCommand run(
      const quadrotor_common::QuadStateEstimate& state_estimate,
      const So3MpcParams<T>& params,
      const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& target_position,
      const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& target_velocity,
      const Eigen::Quaternion<T>& q_image,
      bool target_in_main_view);

  void resetForActivation();

 private:
  void setStateEstimate(
      const quadrotor_common::QuadStateEstimate& state_estimate,
      const Eigen::Matrix<T, 3, 1>& target_position,
      const Eigen::Matrix<T, 3, 1>& target_velocity,
      const Eigen::Quaternion<T>& q_image,
      bool has_target_state);
  void setFocusStateFromQImage(
      Eigen::Ref<Eigen::Matrix<T, kStateSize, 1>> state,
      const Eigen::Quaternion<T>& q_image) const;
  quadrotor_common::ControlCommand makeHoverCommand(
      const ros::Time& time,
      const Eigen::Quaternion<T>& orientation = Eigen::Quaternion<T>::Identity()) const;
  quadrotor_common::ControlCommand makeZeroControlRecoveryCommand(
      const ros::Time& time) const;

  quadrotor_common::ControlCommand updateControlCommand(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
      const ros::Time& time);

  void publishPrediction(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
      const ros::Time& time);
  void publishFailure(bool failed) const;

  void setNewParams(So3MpcParams<T>& params);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_predicted_trajectory_;
  ros::Publisher failure_pub_;
  ros::Publisher solve_latency_pub_;

  So3MpcParams<T> params_;
  So3MpcWrapper<T> mpc_wrapper_;

  T timing_feedback_{T(1e-3)};
  bool solve_from_scratch_{true};
  bool initialize_intercept_speed_from_state_{false};
  bool has_last_valid_solution_{false};
  bool has_last_valid_command_{false};
  int consecutive_invalid_outputs_{0};
  int consecutive_zero_control_outputs_{0};
  int max_invalid_retries_{5};
  int max_zero_control_retries_{5};
  Eigen::Matrix<T, kStateSize, 1> est_state_;
  Eigen::Matrix<T, kStateSize, kSamples + 1> predicted_states_;
  Eigen::Matrix<T, kInputSize, kSamples> predicted_inputs_;
  Eigen::Matrix<T, kStateSize, kSamples + 1> last_valid_states_;
  Eigen::Matrix<T, kInputSize, kSamples> last_valid_inputs_;
  Eigen::Matrix<T, 3, 1> prediction_origin_{Eigen::Matrix<T, 3, 1>::Zero()};
  T current_intercept_speed_{T(0.0)};
  quadrotor_common::ControlCommand last_valid_command_;
  // MPC Debug Logging （MPC调试日志）
  std::size_t debug_log_sequence_{0};
  std::ofstream debug_log_file_;
  // MPC Debug Logging End
};

}  // namespace isb_nmpc
