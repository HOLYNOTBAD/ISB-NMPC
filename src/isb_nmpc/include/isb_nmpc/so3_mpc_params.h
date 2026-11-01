#pragma once

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>

#include <quadrotor_common/parameter_helper.h>

#include "isb_nmpc/so3_mpc_wrapper.h"

namespace isb_nmpc {

template <typename T>
class So3MpcParams {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool loadParameters(ros::NodeHandle& pnh) {
    T Q_velocity;
    T Q_perception_u, Q_perception_v;
    T Q_image_rate_u, Q_image_rate_v, Q_image_rate_z;
    T Q_speed;
    T Q_terminal_line;
    if (!quadrotor_common::getParam("Q_velocity", Q_velocity, pnh) ||
        !quadrotor_common::getParam("Q_perception_u", Q_perception_u, pnh) ||
        !quadrotor_common::getParam("Q_perception_v", Q_perception_v, pnh) ||
        !quadrotor_common::getParam("Q_terminal_line", Q_terminal_line, pnh)) {
      return false;
    }
    quadrotor_common::getParam("Q_image_rate_u", Q_image_rate_u, (T)0.0, pnh);
    quadrotor_common::getParam("Q_image_rate_v", Q_image_rate_v, (T)0.0, pnh);
    quadrotor_common::getParam("Q_image_rate_z", Q_image_rate_z,
                               T(0.5) * (Q_image_rate_u + Q_image_rate_v),
                               pnh);
    quadrotor_common::getParam("Q_speed", Q_speed, (T)50.0, pnh);

    if (Q_velocity < 0.0 || Q_perception_u < 0.0 ||
        Q_perception_v < 0.0 ||
        Q_image_rate_u < 0.0 || Q_image_rate_v < 0.0 ||
        Q_image_rate_z < 0.0 ||
        Q_speed < 0.0 || Q_terminal_line < 0.0) {
      ROS_ERROR("SO3 MPC: State cost Q must be non-negative.");
      return false;
    }

    T R_thrust, R_roll, R_pitch, R_yaw;
    if (!quadrotor_common::getParam("R_thrust", R_thrust, pnh)) {
      return false;
    }
    quadrotor_common::getParam("R_roll", R_roll, (T)-1.0, pnh);
    quadrotor_common::getParam("R_pitch", R_pitch, (T)-1.0, pnh);
    if (R_roll < 0.0 || R_pitch < 0.0) {
      T legacy_R_pitchroll;
      quadrotor_common::getParam("R_pitchroll", legacy_R_pitchroll, (T)0.0, pnh);
      if (R_roll < 0.0) R_roll = legacy_R_pitchroll;
      if (R_pitch < 0.0) R_pitch = legacy_R_pitchroll;
    }
    if (!quadrotor_common::getParam("R_yaw", R_yaw, pnh)) {
      return false;
    }
    quadrotor_common::getParam("R_speed_delta", R_speed_delta_, (T)1.0, pnh);
    quadrotor_common::getParam("R_slack_u", R_slack_u_, (T)500.0, pnh);
    quadrotor_common::getParam("R_slack_v", R_slack_v_, (T)500.0, pnh);

    if (R_thrust <= 0.0 || R_roll <= 0.0 || R_pitch <= 0.0 ||
        R_yaw <= 0.0 || R_speed_delta_ <= 0.0 ||
        R_slack_u_ <= 0.0 || R_slack_v_ <= 0.0) {
      ROS_ERROR("SO3 MPC: Input cost R must be strictly positive.");
      return false;
    }

    Q_ = (Eigen::Matrix<T, kCostSize, 1>() <<
          Q_velocity, Q_velocity, Q_velocity,
          Q_perception_u, Q_perception_v,
          Q_image_rate_v, Q_image_rate_u, Q_image_rate_z,
          Q_speed)
             .finished()
             .asDiagonal();
    R_ = (Eigen::Matrix<T, kInputSize, 1>() <<
          R_thrust, R_roll, R_pitch, R_yaw,
          R_speed_delta_,
          R_slack_u_, R_slack_v_)
             .finished()
             .asDiagonal();
    Q_terminal_line_ = Q_terminal_line;

    quadrotor_common::getParam("state_cost_exponential",
                               state_cost_exponential_, (T)0.0, pnh);
    quadrotor_common::getParam("input_cost_exponential",
                               input_cost_exponential_, (T)0.0, pnh);

    T legacy_max_bodyrate_xy;
    quadrotor_common::getParam("max_bodyrate_xy", legacy_max_bodyrate_xy,
                               (T)-1.0, pnh);
    quadrotor_common::getParam("max_bodyrate_x", max_bodyrate_x_,
                               legacy_max_bodyrate_xy, pnh);
    quadrotor_common::getParam("max_bodyrate_y", max_bodyrate_y_,
                               legacy_max_bodyrate_xy, pnh);
    if (!quadrotor_common::getParam("max_bodyrate_z", max_bodyrate_z_, pnh) ||
        !quadrotor_common::getParam("min_thrust", min_thrust_, pnh) ||
        !quadrotor_common::getParam("max_thrust", max_thrust_, pnh)) {
      return false;
    }

    if (max_bodyrate_x_ <= 0.0 || max_bodyrate_y_ <= 0.0 ||
        max_bodyrate_z_ <= 0.0 || min_thrust_ < 0.0 ||
        max_thrust_ <= min_thrust_) {
      ROS_ERROR("SO3 MPC: Invalid input limits.");
      return false;
    }

    T max_roll_angle_deg;
    quadrotor_common::getParam("max_roll_angle_deg", max_roll_angle_deg,
                               (T)55.0, pnh);
    if (!std::isfinite(max_roll_angle_deg) ||
        max_roll_angle_deg <= T(0.0) || max_roll_angle_deg >= T(90.0)) {
      ROS_ERROR("SO3 MPC: max_roll_angle_deg must be finite and in (0, 90). Got %.3f.",
                static_cast<double>(max_roll_angle_deg));
      return false;
    }
    max_roll_angle_rad_ =
        max_roll_angle_deg * T(3.14159265358979323846 / 180.0);

    std::vector<T> p_B_C, q_B_C;
    if (!pnh.getParam("/camera/p_B_C", p_B_C) || p_B_C.size() != 3 ||
        !pnh.getParam("/camera/q_B_C", q_B_C) || q_B_C.size() != 4) {
      ROS_ERROR("SO3 MPC: camera extrinsics are missing or invalid under /camera; "
                "load the selected camera YAML.");
      return false;
    }
    p_B_C_ = Eigen::Matrix<T, 3, 1>(p_B_C[0], p_B_C[1], p_B_C[2]);
    q_B_C_ = Eigen::Quaternion<T>(q_B_C[0], q_B_C[1], q_B_C[2], q_B_C[3]);
    if (!std::isfinite(q_B_C_.norm()) || q_B_C_.norm() <= T(1e-6)) {
      ROS_ERROR("SO3 MPC: /camera/q_B_C is not a valid quaternion.");
      return false;
    }
    q_B_C_.normalize();

    quadrotor_common::getParam("hover_acceleration", hover_acceleration_,
                               (T)9.81, pnh);
    if (hover_acceleration_ <= 0.0) {
      ROS_ERROR("SO3 MPC: hover_acceleration must be positive.");
      return false;
    }

    quadrotor_common::getParam("intercept_speed", intercept_speed_,
                               (T)2.0, pnh);
    if (intercept_speed_ < 0.0) {
      ROS_ERROR("SO3 MPC: intercept_speed must be non-negative.");
      return false;
    }
    quadrotor_common::getParam("min_intercept_speed", min_intercept_speed_,
                               (T)0.0, pnh);
    if (min_intercept_speed_ < 0.0 ||
        min_intercept_speed_ > intercept_speed_) {
      ROS_ERROR("SO3 MPC: min_intercept_speed must be in [0, intercept_speed].");
      return false;
    }
    quadrotor_common::getParam("los_speed_schedule_kh",
                               los_speed_schedule_kh_, (T)0.0, pnh);
    quadrotor_common::getParam("los_speed_schedule_kv",
                               los_speed_schedule_kv_,
                               T(2.0) * los_speed_schedule_kh_, pnh);
    if (los_speed_schedule_kh_ < 0.0 || los_speed_schedule_kv_ < 0.0) {
      ROS_ERROR("SO3 MPC: LOS speed scheduling gains must be non-negative.");
      return false;
    }
    quadrotor_common::getParam("max_initial_intercept_speed",
                               max_initial_intercept_speed_,
                               intercept_speed_, pnh);
    if (max_initial_intercept_speed_ < 0.0) {
      ROS_ERROR("SO3 MPC: max_initial_intercept_speed must be non-negative.");
      return false;
    }
    quadrotor_common::getParam("max_delta_intercept_speed",
                               max_delta_intercept_speed_, (T)1.0, pnh);
    if (max_delta_intercept_speed_ <= 0.0) {
      ROS_ERROR("SO3 MPC: max_delta_intercept_speed must be positive.");
      return false;
    }
    quadrotor_common::getParam("min_solver_range", min_solver_range_,
                               (T)1.0, pnh);
    if (min_solver_range_ < T(0.1)) {
      ROS_ERROR("SO3 MPC: min_solver_range must be at least 0.1 m.");
      return false;
    }
    quadrotor_common::getParam("nominal_target_range", nominal_target_range_,
                               (T)10.0, pnh);
    if (nominal_target_range_ < min_solver_range_) {
      ROS_ERROR("SO3 MPC: nominal_target_range must be >= min_solver_range.");
      return false;
    }

    double img_width = 640.0;
    double img_height = 480.0;
    double img_f = 115.0;
    pnh.getParam("/camera/img_width", img_width);
    pnh.getParam("/camera/img_height", img_height);
    pnh.getParam("/camera/img_f", img_f);
    double img_fx = img_f;
    double img_fy = img_f;
    pnh.getParam("/camera/fx", img_fx);
    pnh.getParam("/camera/fy", img_fy);
    if (!std::isfinite(img_width) || !std::isfinite(img_height) ||
        !std::isfinite(img_fx) || !std::isfinite(img_fy) ||
        img_width <= 0.0 || img_height <= 0.0 ||
        img_fx <= 0.0 || img_fy <= 0.0) {
      ROS_ERROR("SO3 MPC: invalid camera intrinsics for image-plane cost.");
      return false;
    }

    quadrotor_common::getParam("align_los_theta_to_image_bounds",
                               align_los_theta_to_image_bounds_, true, pnh);
    quadrotor_common::getParam("image_bound_margin", image_bound_margin_,
                               (T)0.3, pnh);
    if (image_bound_margin_ < T(0.0)) {
      ROS_ERROR("SO3 MPC: image_bound_margin must be non-negative.");
      return false;
    }

    quadrotor_common::getParam("los_theta_max", los_theta_max_,
                               (T)0.2617993878, pnh);
    image_center_u_ = T(0.0);
    image_center_v_ = T(0.0);
    image_radius_u_ = T(0.5) * T(img_width) / T(img_fx) - image_bound_margin_;
    image_radius_v_ = T(0.5) * T(img_height) / T(img_fy) - image_bound_margin_;
    if (image_radius_u_ <= T(0.0) || image_radius_v_ <= T(0.0)) {
      ROS_ERROR("SO3 MPC: image bounds must be positive. Check camera "
                "intrinsics and image_bound_margin.");
      return false;
    }
    if (align_los_theta_to_image_bounds_) {
      los_theta_max_ = std::atan(image_radius_v_);
      ROS_INFO_STREAM("SO3 MPC: los_theta_max aligned to image vertical "
                      "bound: theta=" << los_theta_max_
                      << " rad, img_v_max=" << image_radius_v_
                      << " from /camera/img_height=" << img_height
                      << ", /camera/fy=" << img_fy
                      << ", image_bound_margin=" << image_bound_margin_);
    }
    if (los_theta_max_ < T(0.0) || los_theta_max_ >= T(1.57079632679)) {
      ROS_ERROR("SO3 MPC: invalid LOS bearing reference parameters.");
      return false;
    }

    quadrotor_common::getParam("print_info", print_info_, false, pnh);
    changed_ = true;
    return true;
  }

  bool changed_{false};
  bool print_info_{false};
  T hover_acceleration_{9.81};
  T intercept_speed_{2.0};
  T min_intercept_speed_{0.0};
  T los_speed_schedule_kh_{0.0};
  T los_speed_schedule_kv_{0.0};
  T max_initial_intercept_speed_{2.0};
  T max_delta_intercept_speed_{1.0};
  T min_solver_range_{1.0};
  T nominal_target_range_{10.0};
  bool align_los_theta_to_image_bounds_{true};
  T image_bound_margin_{0.3};
  T image_center_u_{0.0};
  T image_center_v_{0.0};
  T image_radius_u_{2.4826086957};
  T image_radius_v_{1.7869565217};
  T los_theta_max_{0.2617993878};
  T R_speed_delta_{1.0};
  T R_slack_u_{500.0};
  T R_slack_v_{500.0};
  T state_cost_exponential_{0.0};
  T input_cost_exponential_{0.0};
  T Q_terminal_line_{0.0};
  T max_bodyrate_x_{0.0};
  T max_bodyrate_y_{0.0};
  T max_bodyrate_z_{0.0};
  T max_roll_angle_rad_{T(55.0 * 3.14159265358979323846 / 180.0)};
  T min_thrust_{0.0};
  T max_thrust_{0.0};
  Eigen::Matrix<T, 3, 1> p_B_C_{Eigen::Matrix<T, 3, 1>::Zero()};
  Eigen::Quaternion<T> q_B_C_{1.0, 0.0, 0.0, 0.0};
  Eigen::Matrix<T, kCostSize, kCostSize> Q_ =
      Eigen::Matrix<T, kCostSize, kCostSize>::Zero();
  Eigen::Matrix<T, kInputSize, kInputSize> R_ =
      Eigen::Matrix<T, kInputSize, kInputSize>::Zero();
};

}  // namespace isb_nmpc
