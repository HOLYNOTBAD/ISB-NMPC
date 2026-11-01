#pragma once

#include <string>

#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include <swarm_msgs/BoundingBoxes.h>
#include <swarm_msgs/BoundingBoxSequence.h>

#include "dc_s2ekf/attitude_history.h"
#include "dc_s2ekf/camera_model.h"
#include "dc_s2ekf/filter_common.h"
#include "dc_s2ekf/log_size_filter.h"
#include "dc_s2ekf/s2_filter.h"

class BBoxS2DelayEkfNode {
public:
  BBoxS2DelayEkfNode();

private:
  using Quat = dc_s2ekf::math::Quat;
  using Vec3 = dc_s2ekf::math::Vec3;
  using Meas = dc_s2ekf::Measurement;
  using SizeState = dc_s2ekf::SizeState;
  using MeasurementQuality = dc_s2ekf::MeasurementQuality;
  using S2MeasurementNoise = dc_s2ekf::S2MeasurementNoise;

  void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void imuCallback(const sensor_msgs::ImuConstPtr &msg);
  void bboxCallback(const swarm_msgs::BoundingBoxesConstPtr &msg);
  void outputTimerCallback(const ros::TimerEvent &);
  bool resetTrial(std_srvs::Trigger::Request &request,
                  std_srvs::Trigger::Response &response);

  Eigen::Matrix3d cameraRotationNewFromOld(const Quat &q_wb_old,
                                          const Quat &q_wb_new) const;
  bool validateTimestamp(const ros::Time &input,
                         ros::Time *validated_stamp) const;
  bool validateImuFreshness(const ros::Time &now) const;
  void publishRoiPredictions(const ros::Time &now);
  bool predictRoiAt(const ros::Time &now, const ros::Time &prediction_time,
                    swarm_msgs::BoundingBox *prediction) const;
  void processBBox(const swarm_msgs::BoundingBoxesConstPtr &msg);
  MeasurementQuality
  computeMeasurementQuality(const swarm_msgs::BoundingBox &box, const Meas &z,
                            const ros::Time &stamp);
  bool updateS2AndPredict(const Meas &z, const Vec3 &bearing_c_meas,
                          const ros::Time &measurement_time,
                          const ros::Time &now, double measurement_quality,
                          Meas *compensated);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string input_topic_;
  std::string output_topic_;
  std::string roi_prediction_topic_;
  std::string roi_prediction_sequence_topic_;
  std::string pose_topic_;
  std::string imu_topic_;

  int image_width_;
  int image_height_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double output_rate_hz_;
  double roi_prediction_horizon_s_;
  double roi_prediction_sequence_horizon_s_;
  double roi_prediction_sequence_step_s_;
  double roi_prediction_max_detection_age_s_;
  bool use_image_stamp_;
  double fallback_delay_s_;
  double max_delay_s_;
  double cam_imu_time_offset_s_;
  double max_predict_ahead_s_;
  double max_update_dt_s_;
  double reset_after_gap_s_;
  double max_coast_s_;
  double coast_probability_decay_s_;
  double coast_probability_floor_;
  double measurement_quality_min_weight_;
  double quality_reference_area_px2_;
  double quality_edge_margin_px_;
  double quality_edge_min_weight_;
  double quality_jump_rate_start_px_s_;
  double quality_jump_rate_full_px_s_;
  double measurement_std_px_;
  double size_measurement_std_px_;
  double min_box_size_px_;
  double min_confidence_for_bearing_;
  bool log_status_;
  double log_period_s_;

  double attitude_buffer_s_;
  double max_attitude_age_s_;
  double max_input_stamp_skew_s_;
  bool use_receive_time_on_stamp_skew_;
  double s2_max_imu_age_s_;
  Eigen::Matrix3d R_bc_;
  double sigma_bearing_meas_rad_;
  double s2_initial_bearing_std_rad_;
  double s2_initial_omega_std_rad_s_;
  double s2_gyro_noise_std_rad_s_;
  double s2_process_accel_std_rad_s2_;
  double min_project_z_;

  ros::Subscriber bbox_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber imu_sub_;
  ros::Timer output_timer_;
  ros::ServiceServer reset_trial_service_;
  ros::Publisher bbox_pub_;
  ros::Publisher roi_prediction_pub_;
  ros::Publisher roi_prediction_sequence_pub_;
  swarm_msgs::BoundingBoxesConstPtr latest_bbox_msg_;
  bool bbox_measurement_pending_ = false;
  swarm_msgs::BoundingBoxes latest_filtered_output_;
  bool has_latest_filtered_output_ = false;
  ros::Time last_detection_wall_time_;
  dc_s2ekf::AttitudeHistory attitude_history_;

  dc_s2ekf::CameraModel camera_model_;

  bool has_last_quality_z_ = false;
  Meas last_quality_z_{};
  ros::Time last_quality_stamp_;

  dc_s2ekf::S2Filter s2_filter_;
  dc_s2ekf::LogSizeFilter log_size_filter_;

  bool has_last_box_template_ = false;
  swarm_msgs::BoundingBox last_box_template_;
};
