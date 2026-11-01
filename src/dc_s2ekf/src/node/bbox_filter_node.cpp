#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "dc_s2ekf/bbox_filter_node.h"
#include "dc_s2ekf/filter_common.h"

using namespace dc_s2ekf::math;
// ==========================================================================
// Constructor （构造函数）
// ==========================================================================

BBoxS2DelayEkfNode::BBoxS2DelayEkfNode() : nh_(), pnh_("~") {
  pnh_.getParam("input_topic", input_topic_);
  pnh_.getParam("output_topic", output_topic_);
  pnh_.getParam("roi_prediction_topic", roi_prediction_topic_);
  pnh_.getParam("roi_prediction_sequence_topic",
                 roi_prediction_sequence_topic_);
  pnh_.getParam("pose_topic", pose_topic_);
  pnh_.getParam("imu_topic", imu_topic_);

  pnh_.getParam("output_rate_hz", output_rate_hz_);
  pnh_.getParam("roi_prediction_horizon_s", roi_prediction_horizon_s_);
  pnh_.getParam("roi_prediction_sequence_horizon_s",
                 roi_prediction_sequence_horizon_s_);
  pnh_.getParam("roi_prediction_sequence_step_s",
                 roi_prediction_sequence_step_s_);
  pnh_.getParam("roi_prediction_max_detection_age_s",
                roi_prediction_max_detection_age_s_);
  pnh_.getParam("use_image_stamp", use_image_stamp_);
  pnh_.getParam("fallback_delay_s", fallback_delay_s_);
  pnh_.getParam("max_delay_s", max_delay_s_);
  std::string cam_imu_time_offset_source = "~cam_imu_time_offset_s";
  if (!pnh_.getParam("cam_imu_time_offset_s", cam_imu_time_offset_s_)) {
    cam_imu_time_offset_source = "/camera/time_offset";
    if (!nh_.getParam("/camera/time_offset", cam_imu_time_offset_s_)) {
      cam_imu_time_offset_s_ = 0.0;
      cam_imu_time_offset_source = "default";
    }
  }
  if (!std::isfinite(cam_imu_time_offset_s_)) {
    ROS_FATAL_STREAM("invalid camera/IMU time offset from "
                     << cam_imu_time_offset_source << ": "
                     << cam_imu_time_offset_s_);
    throw std::runtime_error("invalid camera/IMU time offset");
  }
  pnh_.getParam("max_predict_ahead_s", max_predict_ahead_s_);
  pnh_.getParam("max_update_dt_s", max_update_dt_s_);
  pnh_.getParam("reset_after_gap_s", reset_after_gap_s_);
  pnh_.getParam("max_coast_s", max_coast_s_);
  pnh_.getParam("coast_probability_decay_s", coast_probability_decay_s_);
  pnh_.param("coast_probability_floor", coast_probability_floor_, 0.36);
  pnh_.getParam("measurement_quality_min_weight",
                measurement_quality_min_weight_);
  pnh_.getParam("quality_reference_area_px2", quality_reference_area_px2_);
  pnh_.getParam("quality_edge_margin_px", quality_edge_margin_px_);
  pnh_.getParam("quality_edge_min_weight", quality_edge_min_weight_);
  pnh_.getParam("quality_jump_rate_start_px_s", quality_jump_rate_start_px_s_);
  pnh_.getParam("quality_jump_rate_full_px_s", quality_jump_rate_full_px_s_);
  pnh_.getParam("measurement_std_px", measurement_std_px_);
  pnh_.getParam("size_measurement_std_px", size_measurement_std_px_);
  pnh_.getParam("min_box_size_px", min_box_size_px_);
  pnh_.getParam("min_confidence_for_bearing", min_confidence_for_bearing_);
  pnh_.getParam("log_status", log_status_);
  pnh_.getParam("log_period_s", log_period_s_);

  pnh_.getParam("attitude_buffer_s", attitude_buffer_s_);
  pnh_.getParam("max_attitude_age_s", max_attitude_age_s_);
  pnh_.getParam("max_input_stamp_skew_s", max_input_stamp_skew_s_);
  pnh_.param("use_receive_time_on_stamp_skew",
             use_receive_time_on_stamp_skew_, false);
  pnh_.getParam("s2_max_imu_age_s", s2_max_imu_age_s_);
  std::vector<double> r_bc_values;
  const bool has_camera_rotation =
      nh_.getParam("/camera/R_cam_body", r_bc_values);
  if (!has_camera_rotation || r_bc_values.size() != 9) {
    ROS_FATAL_STREAM(
        "missing or invalid /camera/R_cam_body (expected 9 values, got "
        << r_bc_values.size()
        << "); load the selected camera YAML before starting "
           "bbox_s2_delay_ekf");
    throw std::runtime_error("invalid /camera/R_cam_body");
  }
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      R_bc_(row, col) = r_bc_values[3 * row + col];
    }
  }

  pnh_.getParam("sigma_bearing_meas_rad", sigma_bearing_meas_rad_);
  pnh_.getParam("s2_initial_bearing_std_rad", s2_initial_bearing_std_rad_);
  pnh_.getParam("s2_initial_omega_std_rad_s", s2_initial_omega_std_rad_s_);
  pnh_.getParam("s2_gyro_noise_std_rad_s", s2_gyro_noise_std_rad_s_);
  pnh_.getParam("s2_process_accel_std_rad_s2", s2_process_accel_std_rad_s2_);
  pnh_.getParam("min_project_z", min_project_z_);

  // The camera model belongs to the selected camera profile.  Read it from
  // the global camera namespace so the filter cannot silently use a stale
  // copy from bbox_filter.yaml.
  if (!nh_.getParam("/camera/img_width", image_width_) ||
      !nh_.getParam("/camera/img_height", image_height_) ||
      !nh_.getParam("/camera/fx", fx_) ||
      !nh_.getParam("/camera/fy", fy_) ||
      !nh_.getParam("/camera/cx", cx_) ||
      !nh_.getParam("/camera/cy", cy_)) {
    ROS_FATAL("missing camera intrinsics under /camera; load the selected camera YAML before starting bbox_s2_delay_ekf");
    throw std::runtime_error("missing /camera intrinsics");
  }

  ROS_ASSERT_MSG(image_width_ >= 2 && image_height_ >= 2 &&
                     std::isfinite(fx_) && std::isfinite(fy_) &&
                     std::isfinite(cx_) && std::isfinite(cy_) && fx_ > 0.0 &&
                     fy_ > 0.0 && cx_ >= 0.0 && cx_ < image_width_ &&
                     cy_ >= 0.0 && cy_ < image_height_,
                 "invalid camera model: size=%dx%d fx=%f fy=%f cx=%f cy=%f",
                 image_width_, image_height_, fx_, fy_, cx_, cy_);

  dc_s2ekf::CameraModelConfig camera_config;
  camera_config.image_width = image_width_;
  camera_config.image_height = image_height_;
  camera_config.fx = fx_;
  camera_config.fy = fy_;
  camera_config.cx = cx_;
  camera_config.cy = cy_;
  std::string distortion_model = "none";
  nh_.param<std::string>("/camera/distortion_model", distortion_model,
                         distortion_model);
  if (distortion_model == "equidistant") {
    std::vector<double> distortion_coeffs;
    if (!nh_.getParam("/camera/distortion_coeffs", distortion_coeffs) ||
        distortion_coeffs.size() != 4 ||
        !std::all_of(distortion_coeffs.begin(), distortion_coeffs.end(),
                     [](double value) { return std::isfinite(value); })) {
      ROS_FATAL("/camera/distortion_coeffs must contain four values for the "
                "equidistant model");
      throw std::runtime_error("invalid equidistant distortion coefficients");
    }
    camera_config.distortion_model =
        dc_s2ekf::CameraDistortionModel::EQUIDISTANT;
    std::copy(distortion_coeffs.begin(), distortion_coeffs.end(),
              camera_config.distortion_coeffs.begin());
  } else if (distortion_model != "none" && !distortion_model.empty()) {
    ROS_WARN_STREAM("bbox_s2_delay_ekf: distortion model '"
                    << distortion_model
                    << "' is not supported; preserving pinhole projection");
  }
  camera_config.min_box_size_px = min_box_size_px_;
  camera_config.min_project_z = min_project_z_;
  camera_model_.configure(camera_config);

  dc_s2ekf::LogSizeFilterConfig size_config;
  size_config.min_box_size_px = min_box_size_px_;
  size_config.measurement_std_px = size_measurement_std_px_;
  log_size_filter_.configure(size_config);

  dc_s2ekf::S2FilterConfig s2_config;
  s2_config.measurement_quality_min_weight = measurement_quality_min_weight_;
  s2_config.sigma_bearing_meas_rad = sigma_bearing_meas_rad_;
  s2_config.initial_bearing_std_rad = s2_initial_bearing_std_rad_;
  s2_config.initial_omega_std_rad_s = s2_initial_omega_std_rad_s_;
  s2_config.gyro_noise_std_rad_s = s2_gyro_noise_std_rad_s_;
  s2_config.process_accel_std_rad_s2 = s2_process_accel_std_rad_s2_;
  s2_filter_.configure(s2_config);

  dc_s2ekf::AttitudeHistoryConfig attitude_config;
  attitude_config.buffer_duration_s = attitude_buffer_s_;
  attitude_config.max_sample_age_s = max_attitude_age_s_;
  attitude_history_.configure(attitude_config);

  // subscribe and advertise topics 
  bbox_pub_ = nh_.advertise<swarm_msgs::BoundingBoxes>(output_topic_, 10);
  roi_prediction_pub_ =
      nh_.advertise<swarm_msgs::BoundingBoxes>(roi_prediction_topic_, 10);
  roi_prediction_sequence_pub_ =
      nh_.advertise<swarm_msgs::BoundingBoxSequence>(
          roi_prediction_sequence_topic_, 10);
  bbox_sub_ =
      nh_.subscribe(input_topic_, 10, &BBoxS2DelayEkfNode::bboxCallback, this);

  pose_sub_ =
      nh_.subscribe(pose_topic_, 100, &BBoxS2DelayEkfNode::poseCallback, this);
  imu_sub_ =
      nh_.subscribe(imu_topic_, 100, &BBoxS2DelayEkfNode::imuCallback, this);

  // main loop timer
  const double safe_output_rate_hz = std::max(1.0, output_rate_hz_);
  output_timer_ =
      nh_.createTimer(ros::Duration(1.0 / safe_output_rate_hz),
                      &BBoxS2DelayEkfNode::outputTimerCallback, this);
  reset_trial_service_ = pnh_.advertiseService(
      "reset_trial", &BBoxS2DelayEkfNode::resetTrial, this);

  ROS_INFO_STREAM("bbox_s2_delay_ekf v3 active: estimator=s2_high_dynamic"
                  << " input=" << input_topic_ << " output=" << output_topic_
                  << " camera fx=" << fx_ << " fy=" << fy_ << " cx=" << cx_
                  << " cy=" << cy_ << " size=" << image_width_ << "x"
                  << image_height_ << " output_rate_hz=" << safe_output_rate_hz
                  << " distortion_model=" << distortion_model
                  << " process_accel_std_rad_s2="
                  << s2_process_accel_std_rad_s2_
                  << " cam_imu_time_offset_ms="
                  << cam_imu_time_offset_s_ * 1000.0
                  << " offset_source=" << cam_imu_time_offset_source
                  << " roi_sequence=" << roi_prediction_sequence_topic_
                  << " horizon_s=" << roi_prediction_sequence_horizon_s_
                  << " step_s=" << roi_prediction_sequence_step_s_
                  << " R_bc=[" << R_bc_(0, 0) << "," << R_bc_(0, 1) << ","
                  << R_bc_(0, 2) << ";" << R_bc_(1, 0) << "," << R_bc_(1, 1)
                  << "," << R_bc_(1, 2) << ";" << R_bc_(2, 0) << ","
                  << R_bc_(2, 1) << "," << R_bc_(2, 2) << "]");
}

bool BBoxS2DelayEkfNode::resetTrial(
    std_srvs::Trigger::Request &,
    std_srvs::Trigger::Response &response) {
  latest_bbox_msg_.reset();
  bbox_measurement_pending_ = false;
  latest_filtered_output_ = swarm_msgs::BoundingBoxes();
  has_latest_filtered_output_ = false;
  last_detection_wall_time_ = ros::Time();
  has_last_quality_z_ = false;
  last_quality_z_ = Meas{};
  last_quality_stamp_ = ros::Time();
  s2_filter_.reset();
  has_last_box_template_ = false;
  last_box_template_ = swarm_msgs::BoundingBox();

  swarm_msgs::BoundingBoxes empty;
  empty.header.stamp = ros::Time::now();
  bbox_pub_.publish(empty);
  roi_prediction_pub_.publish(empty);
  swarm_msgs::BoundingBoxSequence empty_sequence;
  empty_sequence.header.stamp = empty.header.stamp;
  roi_prediction_sequence_pub_.publish(empty_sequence);

  response.success = true;
  response.message = "bbox filter state cleared";
  ROS_INFO("bbox_s2_delay_ekf: trial state reset.");
  return true;
}

// ==========================================================================
//  ROS Callbacks（ROS回调）
// ==========================================================================
void BBoxS2DelayEkfNode::poseCallback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  ros::Time stamp;
  if (!validateTimestamp(msg->header.stamp, &stamp)) {
    return;
  }
  attitude_history_.pushPose(stamp, fromMsg(msg->pose.orientation));
}

void BBoxS2DelayEkfNode::imuCallback(const sensor_msgs::ImuConstPtr &msg) {
  if (!std::isfinite(msg->orientation.w) ||
      !std::isfinite(msg->orientation.x) ||
      !std::isfinite(msg->orientation.y) ||
      !std::isfinite(msg->orientation.z) ||
      !std::isfinite(msg->angular_velocity.x) ||
      !std::isfinite(msg->angular_velocity.y) ||
      !std::isfinite(msg->angular_velocity.z)) {
    return;
  }
  ros::Time stamp;
  if (!validateTimestamp(msg->header.stamp, &stamp)) {
    return;
  }
  const Vec3 omega_b = makeVec(msg->angular_velocity.x, msg->angular_velocity.y,
                               msg->angular_velocity.z);
  if (attitude_history_.hasImuRate()) {
    if (stamp < attitude_history_.lastImuStamp()) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "bbox_s2_delay_ekf: dropping out-of-sequence IMU sample by "
                   << (attitude_history_.lastImuStamp() - stamp).toSec()
                   << " s");
      return;
    }
    // MAVROS can publish more than one IMU message during the same Gazebo
    // clock tick. A duplicate stamp contains no positive integration interval;
    // discard it quietly instead of reporting it as out of sequence.
    if (stamp == attitude_history_.lastImuStamp()) {
      return;
    }
  }
  attitude_history_.pushImu(stamp, fromMsg(msg), omega_b);
}

void BBoxS2DelayEkfNode::bboxCallback(
    const swarm_msgs::BoundingBoxesConstPtr &msg) {
  if (!msg->bounding_boxes.empty()) {
    const auto &box = msg->bounding_boxes.front();
    const bool valid = std::isfinite(box.probability) && box.xmin >= 0 &&
                       box.ymin >= 0 && box.xmax > box.xmin &&
                       box.ymax > box.ymin && box.xmax <= image_width_ &&
                       box.ymax <= image_height_;
    // Reject invalid or out-of-frame boxes
    if (!valid) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "bbox_s2_delay_ekf: rejecting invalid/out-of-frame bbox ["
                   << box.xmin << "," << box.ymin << " -> " << box.xmax << ","
                   << box.ymax << "] for " << image_width_ << "x"
                   << image_height_
                   << " image, probability=" << box.probability);
      swarm_msgs::BoundingBoxesPtr sanitized(
          new swarm_msgs::BoundingBoxes(*msg));
      sanitized->bounding_boxes.clear();
      latest_bbox_msg_ = sanitized;
      bbox_measurement_pending_ = true;
      return;
    }
  }
  // Cache the latest bbox
  latest_bbox_msg_ = msg;
  bbox_measurement_pending_ = true;
}

void BBoxS2DelayEkfNode::outputTimerCallback(const ros::TimerEvent &) {
  if (bbox_measurement_pending_ && latest_bbox_msg_) {
    const swarm_msgs::BoundingBoxesConstPtr msg = latest_bbox_msg_;
    bbox_measurement_pending_ = false;
    // 1. bbox pending
    if (!msg->bounding_boxes.empty()) {
      // Process the latest bbox measurement and publish the filtered output
      processBBox(msg);
      // Publish the ROI prediction based on the current time
      publishRoiPredictions(ros::Time::now());
      return;
    }
  }
  // 2. No new bbox measurement; publish the last filtered output or coast
  swarm_msgs::BoundingBoxes out;
  const ros::Time now = ros::Time::now();
  const double detection_age_s =
      last_detection_wall_time_.isZero()
          ? std::numeric_limits<double>::infinity()
          : timeDeltaSec(now, last_detection_wall_time_);
  if (s2_filter_.initialized() && !s2_filter_.lastUpdateTime().isZero() &&
      std::isfinite(detection_age_s) && detection_age_s >= 0.0 &&
      detection_age_s <= max_coast_s_) {
    Vec3 bearing_c = s2_filter_.bearing();
    const double dt =
        clamp(timeDeltaSec(now, s2_filter_.lastUpdateTime()), 0.0,
              max_predict_ahead_s_);
    SizeState size = log_size_filter_.predictSize(dt);
    Quat q_wb_last;
    Quat q_wb_now;
    if (attitude_history_.attitudeAt(
            s2_filter_.lastUpdateTime(), &q_wb_last) &&
        attitude_history_.attitudeAt(now, &q_wb_now)) {
      const Eigen::Matrix3d rotation =
          cameraRotationNewFromOld(q_wb_last, q_wb_now);
      bearing_c = s2_filter_.predictBearing(dt, rotation);
      double u = 0.0;
      double v = 0.0;
      if (camera_model_.pixelFromBearing(bearing_c, &u, &v)) {
        out = latest_filtered_output_;
        if (out.bounding_boxes.empty() && has_last_box_template_) {
          out.bounding_boxes.push_back(last_box_template_);
        }
        if (!out.bounding_boxes.empty()) {
          const Meas predicted = {u, v,
                                  clamp(std::exp(size[0]), min_box_size_px_,
                                        static_cast<double>(image_width_)),
                                  clamp(std::exp(size[1]), min_box_size_px_,
                                        static_cast<double>(image_height_))};
          camera_model_.measurementToBox(predicted,
                                         &out.bounding_boxes.front());
          if (coast_probability_decay_s_ > 1e-6) {
            const double keep = clamp(
                1.0 - detection_age_s / coast_probability_decay_s_,
                coast_probability_floor_, 1.0);
            out.bounding_boxes.front().probability = std::max(
                coast_probability_floor_, last_box_template_.probability * keep);
          }
        }
        latest_filtered_output_ = out;
      }
    }
    if (out.bounding_boxes.empty() && has_latest_filtered_output_) {
      out = latest_filtered_output_;
    }
  } else if (has_latest_filtered_output_ && std::isfinite(detection_age_s) &&
             detection_age_s >= 0.0 && detection_age_s <= max_coast_s_) {
    out = latest_filtered_output_;
  }
  out.header.stamp = now;
  bbox_pub_.publish(out);
  publishRoiPredictions(now);
}

void BBoxS2DelayEkfNode::processBBox(
    const swarm_msgs::BoundingBoxesConstPtr &msg) {
  const ros::Time now = ros::Time::now();

  swarm_msgs::BoundingBoxes out = *msg;
  out.header.stamp = now;

  // Prefer the original image acquisition stamp. A detector message normally
  // arrives later because of exposure, transport and inference latency. If
  // that stamp is absent, in the future, or older than max_delay_s_, use the
  // configured fallback delay instead.
  //（优先采用原始图像采样时间；时间戳无效时，用固定延迟反推测量时刻。）
  double delay_s = fallback_delay_s_;
  bool used_image_stamp = false;
  ros::Time measurement_time = shiftedTime(now, -fallback_delay_s_);
  if (use_image_stamp_ && !msg->image_header.stamp.isZero()) {
    const double measured_delay =
        timeDeltaSec(now, msg->image_header.stamp);
    if (std::isfinite(measured_delay) && measured_delay >= 0.0 &&
        measured_delay <= max_delay_s_) {
      delay_s = measured_delay;
      used_image_stamp = true;
      measurement_time = msg->image_header.stamp;
    }
  }

  bool compensated_valid = false;

  // A valid detector measurement updates the filters. The current public
  // interface is single-target, so only bounding_boxes.front() is consumed.
  //（存在首框且置信度达标时才执行测量更新；当前接口按单目标处理首框。）
  if (!msg->bounding_boxes.empty() &&
      msg->bounding_boxes.front().probability >= min_confidence_for_bearing_) {
    const auto &raw_box = msg->bounding_boxes.front();

    // z = [center_u, center_v, width, height]. Apply the camera/IMU clock
    // offset before querying attitude.
    //（先把bbox转成中心与尺寸测量，再校正相机/IMU时偏。）
    const Meas z = camera_model_.boxToMeasurement(raw_box);
    const ros::Time effective_measurement_time =
        shiftedTime(measurement_time, cam_imu_time_offset_s_);
    delay_s =
        std::max(0.0, timeDeltaSec(now, effective_measurement_time));
    // Quality is a continuous measurement-noise weight, not a hard rejection
    // gate. Small, low-confidence, edge-adjacent or fast-jumping boxes
    // receive less trust in the estimator update.
    //（质量分数连续调节测量噪声；小框、低置信度、贴边或突跳框会被降低权重。）
    const MeasurementQuality meas_quality =
        computeMeasurementQuality(raw_box, z, effective_measurement_time);

    // Convert the pixel center to a unit line-of-sight vector in the camera
    // optical frame. S2 filtering operates on this bearing rather than
    // directly on image pixels.
    //（将像素中心反投影为相机系单位视线，S2滤波在球面视线上工作。）
    const Vec3 raw_bearing_c = camera_model_.bearingFromPixel(z[0], z[1]);
    Meas compensated = z;
    if (!validateImuFreshness(now)) {
      if (s2_filter_.initialized()) {
        ROS_WARN_STREAM_THROTTLE(
            1.0, "bbox_s2_delay_ekf: IMU is stale; resetting S2 and holding "
                 "the last valid output");
        s2_filter_.reset();
      }
    } else {
      compensated_valid =
          updateS2AndPredict(z, raw_bearing_c, effective_measurement_time, now,
                             meas_quality.weight, &compensated);
    }
    if (compensated_valid) {
      last_detection_wall_time_ = now;
      camera_model_.measurementToBox(compensated, &out.bounding_boxes.front());
      last_box_template_ = out.bounding_boxes.front();
      has_last_box_template_ = true;
    }
  }

  // Cache and publish the controller-facing bbox first. Every publication
  // below is optional telemetry and cannot alter the selected bbox.
  //（先缓存并发布控制器bbox；后续均为可选诊断，不再改变滤波决策。）
  if (compensated_valid) {
    latest_filtered_output_ = out;
    has_latest_filtered_output_ = true;
  } else {
    out.bounding_boxes.clear();
      const double valid_output_age_s =
          last_detection_wall_time_.isZero()
              ? std::numeric_limits<double>::infinity()
              : timeDeltaSec(now, last_detection_wall_time_);
    if (has_latest_filtered_output_ && std::isfinite(valid_output_age_s) &&
        valid_output_age_s >= 0.0 && valid_output_age_s <= max_coast_s_) {
      out = latest_filtered_output_;
      out.header.stamp = now;
    }
  }
  bbox_pub_.publish(out);

  // Throttled status summarizes the timing source and S2 result without
  // logging at the detector/output rate.
  //（限频打印本帧采用的时间源与S2输出状态。）
  if (log_status_) {
    ROS_INFO_STREAM_THROTTLE(
        log_period_s_,
        "bbox_s2_delay_ekf v3: estimator=s2_high_dynamic boxes="
            << msg->bounding_boxes.size() << " delay_s=" << delay_s
            << " delay_source="
            << (used_image_stamp ? "image_stamp" : "configured_delay")
            << " image_stamp=" << stampToSec(msg->image_header.stamp)
            << " s2_initialized=" << s2_filter_.initialized()
            << " attitude_samples=" << attitude_history_.size()
            << " compensated=" << compensated_valid
            << " output_boxes=" << out.bounding_boxes.size());
  }
}
// ==========================================================================
// helper functions（辅助函数）
// =========================================================================
Eigen::Matrix3d BBoxS2DelayEkfNode::cameraRotationNewFromOld(
    const Quat &q_wb_old, const Quat &q_wb_new) const {
  const Eigen::Matrix3d R_wc_old = quatToMatrix(q_wb_old) * R_bc_;
  const Eigen::Matrix3d R_wc_new = quatToMatrix(q_wb_new) * R_bc_;
  return R_wc_new.transpose() * R_wc_old;
}

bool BBoxS2DelayEkfNode::validateTimestamp(
    const ros::Time &input, ros::Time *validated_stamp) const {
  if (validated_stamp == nullptr) {
    return false;
  }
  const ros::Time now = ros::Time::now();
  if (now.isZero()) {
    *validated_stamp = input;
    return !validated_stamp->isZero();
  }
  if (input.isZero()) {
    *validated_stamp = now;
    return true;
  }
  const double skew_s = std::fabs(timeDeltaSec(input, now));
  if (std::isfinite(skew_s) && skew_s <= max_input_stamp_skew_s_) {
    *validated_stamp = input;
    return true;
  }
  if (use_receive_time_on_stamp_skew_) {
    *validated_stamp = now;
    ROS_WARN_STREAM_THROTTLE(
        1.0,
        "bbox_s2_delay_ekf: attitude timestamp differs from the active ROS "
        "clock by "
            << skew_s
            << " s; using receive time (expected when MAVROS wall-clock "
               "headers are consumed under Gazebo /use_sim_time)");
    return true;
  }
  ROS_WARN_STREAM_THROTTLE(
      1.0, "bbox_s2_delay_ekf: dropping stale/future attitude sample with "
               << skew_s << " s timestamp skew");
  return false;
}

bool BBoxS2DelayEkfNode::validateImuFreshness(const ros::Time &now) const {
  if (!attitude_history_.hasImuRate() ||
      attitude_history_.lastImuStamp().isZero() || now.isZero()) {
    return false;
  }
  const double age_s = (now - attitude_history_.lastImuStamp()).toSec();
  return std::isfinite(age_s) && age_s >= 0.0 && age_s <= s2_max_imu_age_s_;
}

// ==========================================================================
// ROI Prediction（ROI预测）
// =========================================================================
bool BBoxS2DelayEkfNode::predictRoiAt(
    const ros::Time &now, const ros::Time &prediction_time,
    swarm_msgs::BoundingBox *prediction) const {
  if (prediction == nullptr || prediction_time < now) {
    return false;
  }
  const double detection_age_s =
      last_detection_wall_time_.isZero()
          ? std::numeric_limits<double>::infinity()
          : timeDeltaSec(now, last_detection_wall_time_);
  const bool healthy =
      s2_filter_.initialized() && !s2_filter_.lastUpdateTime().isZero() &&
      s2_filter_.trackPhase() == dc_s2ekf::S2TrackPhase::TRACKING &&
      std::isfinite(detection_age_s) && detection_age_s >= 0.0 &&
      detection_age_s <= roi_prediction_max_detection_age_s_ &&
      has_last_box_template_;
  if (!healthy) {
    return false;
  }

  const double prediction_dt =
      timeDeltaSec(prediction_time, s2_filter_.lastUpdateTime());
  if (!std::isfinite(prediction_dt) || prediction_dt < 0.0 ||
      prediction_dt > max_predict_ahead_s_) {
    return false;
  }
  Vec3 bearing_c = s2_filter_.bearing();
  SizeState size = log_size_filter_.predictSize(prediction_dt);

  Quat q_wb_last;
  Quat q_wb_prediction;
  if (!attitude_history_.attitudeAt(
          s2_filter_.lastUpdateTime(), &q_wb_last) ||
      !attitude_history_.attitudeAt(prediction_time, &q_wb_prediction)) {
    return false;
  }
  const Eigen::Matrix3d rotation =
      cameraRotationNewFromOld(q_wb_last, q_wb_prediction);
  bearing_c = s2_filter_.predictBearing(prediction_dt, rotation);
  double u = 0.0;
  double v = 0.0;
  if (!camera_model_.pixelFromBearing(bearing_c, &u, &v)) {
    return false;
  }

  *prediction = last_box_template_;
  const Meas predicted = {u, v,
                          clamp(std::exp(size[0]), min_box_size_px_,
                                static_cast<double>(image_width_)),
                          clamp(std::exp(size[1]), min_box_size_px_,
                                static_cast<double>(image_height_))};
  camera_model_.measurementToBox(predicted, prediction);
  return true;
}

void BBoxS2DelayEkfNode::publishRoiPredictions(const ros::Time &now) {
  swarm_msgs::BoundingBoxSequence sequence;
  sequence.header.stamp = now;

  const double sequence_horizon =
      clamp(roi_prediction_sequence_horizon_s_, 0.0, max_predict_ahead_s_);
  const double sequence_step =
      clamp(roi_prediction_sequence_step_s_, 0.005,
            std::max(0.005, sequence_horizon));
  if (sequence_horizon > 0.0) {
    const int sample_count =
        std::max(1, static_cast<int>(
                        std::ceil(sequence_horizon / sequence_step)));
    sequence.predictions.reserve(static_cast<size_t>(sample_count + 1));
    for (int index = 0; index <= sample_count; ++index) {
      const double offset_s =
          std::min(sequence_horizon, index * sequence_step);
      swarm_msgs::TimedBoundingBox timed_prediction;
      timed_prediction.predicted_time = now + ros::Duration(offset_s);
      if (predictRoiAt(now, timed_prediction.predicted_time,
                       &timed_prediction.bbox)) {
        sequence.predictions.push_back(timed_prediction);
      }
    }
  }
  roi_prediction_sequence_pub_.publish(sequence);

  // Keep the original one-point topic for visualization and compatibility.
  swarm_msgs::BoundingBoxes prediction;
  const double horizon_s =
      clamp(roi_prediction_horizon_s_, 0.0, max_predict_ahead_s_);
  const ros::Time prediction_time = now + ros::Duration(horizon_s);
  prediction.header.stamp = prediction_time;
  prediction.image_header = prediction.header;
  swarm_msgs::BoundingBox legacy_box;
  if (predictRoiAt(now, prediction_time, &legacy_box)) {
    prediction.bounding_boxes.push_back(legacy_box);
  }
  roi_prediction_pub_.publish(prediction);
}



// ==========================================================================
//  Measurement Quality（测量质量）
// ==========================================================================
BBoxS2DelayEkfNode::MeasurementQuality
BBoxS2DelayEkfNode::computeMeasurementQuality(
    const swarm_msgs::BoundingBox &box, const Meas &z, const ros::Time &stamp) {
  MeasurementQuality q;
  const double min_w = clamp(measurement_quality_min_weight_, 0.02, 1.0);
  const double area = std::max(1.0, z[2] * z[3]);
  q.area_weight = clamp(dc_s2ekf::math::smoothstep(
                            min_box_size_px_ * min_box_size_px_,
                            std::max(min_box_size_px_ * min_box_size_px_ + 1.0,
                                     quality_reference_area_px2_),
                            area),
                        min_w, 1.0);
  q.confidence_weight = clamp(
      dc_s2ekf::math::smoothstep(
          min_confidence_for_bearing_,
          std::max(min_confidence_for_bearing_ + 1e-3, 1.0), box.probability),
      min_w, 1.0);

  const double edge_margin = std::max(1.0, quality_edge_margin_px_);
  const double dist_edge =
      std::min(std::min(z[0], static_cast<double>(image_width_ - 1) - z[0]),
               std::min(z[1], static_cast<double>(image_height_ - 1) - z[1]));
  q.edge_weight =
      quality_edge_min_weight_ +
      (1.0 - quality_edge_min_weight_) *
          dc_s2ekf::math::smoothstep(0.0, edge_margin, dist_edge);
  q.edge_weight = clamp(q.edge_weight, min_w, 1.0);

  q.jump_weight = 1.0;
  if (has_last_quality_z_ && !last_quality_stamp_.isZero()) {
    const double dt = timeDeltaSec(stamp, last_quality_stamp_);
    if (std::isfinite(dt) && dt > 1e-4 && dt < reset_after_gap_s_) {
      const double du = z[0] - last_quality_z_[0];
      const double dv = z[1] - last_quality_z_[1];
      q.jump_rate_px_s = std::sqrt(du * du + dv * dv) / dt;
      const double jump_badness = dc_s2ekf::math::smoothstep(
          quality_jump_rate_start_px_s_, quality_jump_rate_full_px_s_,
          q.jump_rate_px_s);
      q.jump_weight = clamp(1.0 - (1.0 - min_w) * jump_badness, min_w, 1.0);
    }
  }

  q.weight =
      clamp(q.area_weight * q.confidence_weight * q.edge_weight * q.jump_weight,
            min_w, 1.0);
  last_quality_z_ = z;
  last_quality_stamp_ = stamp;
  has_last_quality_z_ = true;
  return q;
}

// ==========================================================================
//  S2 Estimator and Current-Time Predictor（S2估计与当前时刻预测）
// ==========================================================================
bool BBoxS2DelayEkfNode::updateS2AndPredict(const Meas &z,
                                            const Vec3 &bearing_c_meas,
                                            const ros::Time &measurement_time,
                                            const ros::Time &now,
                                            double measurement_quality,
                                            Meas *compensated) {
  Quat q_wb_meas;
  Quat q_wb_now;
  // 插值得到t_meas时刻的姿态
  if (!attitude_history_.interpolateAt(measurement_time, &q_wb_meas)) {
    return false;
  }
  // 预测t_now时刻的姿态；如果预测失败，则直接获取t_now时刻的姿态。
  if (!attitude_history_.propagateFrom(
          measurement_time, now, q_wb_meas, &q_wb_now) &&
      !attitude_history_.attitudeAt(now, &q_wb_now)) {
    return false;
  }

  // Compute the noise of the bearing with respect to pixel coordinates.
  Vec3 db_du_c;
  Vec3 db_dv_c;
  camera_model_.bearingJacobiansFromPixel(z[0], z[1], &db_du_c, &db_dv_c);
  const double box_w = std::max(min_box_size_px_, z[2]);
  const double box_h = std::max(min_box_size_px_, z[3]);
  const double aspect = std::sqrt(clamp(box_w / box_h, 0.25, 4.0));
  const double quality_sqrt = std::sqrt(
      clamp(measurement_quality, measurement_quality_min_weight_, 1.0));
  const double sigma_u_px = measurement_std_px_ * clamp(aspect, 0.70, 1.80) /
                            std::max(0.25, quality_sqrt);
  const double sigma_v_px = measurement_std_px_ *
                            clamp(1.0 / aspect, 0.70, 1.80) /
                            std::max(0.25, quality_sqrt);

  S2MeasurementNoise measurement_noise;
  measurement_noise.pixel_u_jacobian_c = db_du_c;
  measurement_noise.pixel_v_jacobian_c = db_dv_c;
  measurement_noise.variance_u_px2 = sigma_u_px * sigma_u_px;
  measurement_noise.variance_v_px2 = sigma_v_px * sigma_v_px;

  // initialize if uninitialized/timeout/reset
  if (!s2_filter_.initialized() || s2_filter_.lastUpdateTime().isZero() ||
      std::fabs(
          timeDeltaSec(measurement_time, s2_filter_.lastUpdateTime())) >
          reset_after_gap_s_) {
    s2_filter_.initialize(bearing_c_meas, measurement_time);
    log_size_filter_.initialize(z);
  } else {
    const double raw_dt =
        timeDeltaSec(measurement_time, s2_filter_.lastUpdateTime());
    // Reject out-of-sequence measurements that are too old. A small negative
    if (raw_dt < -1e-4) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "bbox_s2_delay_ekf: dropping out-of-sequence bbox by " << -raw_dt
                                                                      << " s");
      return false;
    }
    const double dt_update = clamp(raw_dt, 0.0, max_update_dt_s_);
    Quat q_wb_previous;
    if (!attitude_history_.interpolateAt(s2_filter_.lastUpdateTime(),
                                         &q_wb_previous)) {
      return false;
    }
    const Eigen::Matrix3d rotation =
        cameraRotationNewFromOld(q_wb_previous, q_wb_meas);
    // Propagate the S2 filter to time_meas, then update with CV measurement.
    s2_filter_.propagate(dt_update, rotation);
    s2_filter_.update(bearing_c_meas, measurement_noise,
                      measurement_quality, measurement_time);
    // update the log-size filter with the raw pixel measurement and quality.
    log_size_filter_.update(z, dt_update, measurement_quality);
  }

  // time compensation: predict the S2 and log-size filters to the current time
  const double dt_ahead = clamp(timeDeltaSec(now, measurement_time), 0.0,
                                std::min(max_predict_ahead_s_, max_delay_s_));
  Vec3 b_now = s2_filter_.bearing();
  const SizeState size_now = log_size_filter_.predictSize(dt_ahead);
  const Eigen::Matrix3d rotation_now =
      cameraRotationNewFromOld(q_wb_meas, q_wb_now);
  b_now = s2_filter_.predictBearing(dt_ahead, rotation_now);

  double u = z[0];
  double v = z[1];
  if (!camera_model_.pixelFromBearing(b_now, &u, &v)) {
    return false;
  }

  if (compensated != nullptr) {
    (*compensated)[0] = u;
    (*compensated)[1] = v;
    (*compensated)[2] = clamp(std::exp(size_now[0]), min_box_size_px_,
                              static_cast<double>(image_width_));
    (*compensated)[3] = clamp(std::exp(size_now[1]), min_box_size_px_,
                              static_cast<double>(image_height_));
  }
  return true;
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "bbox_s2_delay_ekf");
  BBoxS2DelayEkfNode node;
  ros::spin();
  return 0;
}
