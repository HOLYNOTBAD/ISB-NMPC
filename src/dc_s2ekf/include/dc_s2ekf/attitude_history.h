#pragma once

#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>

#include <ros/duration.h>
#include <ros/time.h>

#include "dc_s2ekf/filter_common.h"

namespace dc_s2ekf {

struct AttitudeHistoryConfig {
  double buffer_duration_s = 1.0;
  double max_sample_age_s = 0.25;
};

// AttitudeHistory stores time-indexed attitudes and gyro increments.
//（AttitudeHistory存储按时间索引的姿态与陀螺仪增量。）
//
// It deliberately owns no ROS subscriptions: the node validates samples,
// while interpolation and replay remain independently testable.
//（该类不持有ROS订阅：节点负责校验样本，插值与回放逻辑可独立测试。）
class AttitudeHistory {
 public:
  using Quat = math::Quat;
  using Vec3 = math::Vec3;

  void configure(const AttitudeHistoryConfig& config) { config_ = config; }

  std::size_t size() const { return samples_.size(); }
  bool hasImuRate() const { return has_last_imu_rate_; }
  const ros::Time& lastImuStamp() const { return last_imu_rate_stamp_; }
  const Vec3& lastImuRate() const { return last_imu_rate_b_; }

  void pushPose(const ros::Time& stamp, const Quat& attitude) {
    pushAttitude(stamp, attitude, Vec3{}, false);
  }

  void pushImu(const ros::Time& stamp, const Quat& attitude,
               const Vec3& angular_rate_b) {
    if (has_last_imu_rate_ && stamp > last_imu_rate_stamp_) {
      const double dt = math::timeDeltaSec(stamp, last_imu_rate_stamp_);
      if (std::isfinite(dt) && dt > 1e-5 && dt < 0.1) {
        imu_deltas_.push_back(ImuDeltaSample{
            last_imu_rate_stamp_, stamp,
            0.5 * (last_imu_rate_b_ + angular_rate_b)});
      }
    }
    last_imu_rate_stamp_ = stamp;
    last_imu_rate_b_ = angular_rate_b;
    has_last_imu_rate_ = true;
    while (!imu_deltas_.empty() &&
           math::timeDeltaSec(stamp, imu_deltas_.front().end) >
               config_.buffer_duration_s) {
      imu_deltas_.pop_front();
    }
    pushAttitude(stamp, attitude, angular_rate_b, true);
  }

  // Replay buffered body rates from a known attitude to the requested time.
  //（从已知姿态开始回放缓存的机体系角速度，得到指定时刻姿态。）
  bool propagateFrom(const ros::Time& from, const ros::Time& to,
                     const Quat& attitude_from, Quat* attitude_to) const {
    if (attitude_to == nullptr || from.isZero() || to < from) {
      return false;
    }
    Quat attitude = attitude_from;
    ros::Time cursor = from;
    for (const ImuDeltaSample& sample : imu_deltas_) {
      if (sample.end <= cursor || sample.start >= to) {
        continue;
      }
      const ros::Time overlap_start =
          sample.start > cursor ? sample.start : cursor;
      const ros::Time overlap_end = sample.end < to ? sample.end : to;
      const double dt = math::timeDeltaSec(overlap_end, overlap_start);
      if (dt > 0.0) {
        attitude = integrateBodyRate(attitude, sample.omega_b, dt);
        cursor = overlap_end;
      }
      if (cursor >= to) {
        break;
      }
    }
    if (cursor < to && has_last_imu_rate_) {
      const double tail_dt = math::timeDeltaSec(to, cursor);
      if (tail_dt >= 0.0 && tail_dt <= config_.max_sample_age_s) {
        attitude = integrateBodyRate(attitude, last_imu_rate_b_, tail_dt);
      }
    }
    *attitude_to = math::normalize(attitude);
    return true;
  }

  // Return an interpolated attitude, or extrapolate it with fresh IMU rates.
  //（返回插值姿态；当IMU角速度足够新鲜时允许外推。）
  bool attitudeAt(const ros::Time& stamp, Quat* attitude) const {
    if (attitude == nullptr || samples_.empty()) {
      return false;
    }
    const AttitudeSample& latest = samples_.back();
    if (stamp <= latest.stamp) {
      return interpolateAt(stamp, attitude);
    }
    const double ahead_s = math::timeDeltaSec(stamp, latest.stamp);
    if (!std::isfinite(ahead_s) || ahead_s > config_.max_sample_age_s) {
      return false;
    }
    if (propagateFrom(latest.stamp, stamp, latest.q_wb, attitude)) {
      return true;
    }
    return interpolateAt(stamp, attitude);
  }

  // Interpolate attitude history without extrapolating beyond the age limit.
  //（在最大样本时延内插值姿态历史，不进行超限外推。）
  bool interpolateAt(const ros::Time& stamp, Quat* attitude) const {
    if (attitude == nullptr || samples_.empty()) {
      return false;
    }
    if (stamp <= samples_.front().stamp) {
      const double age = std::fabs(
          math::timeDeltaSec(samples_.front().stamp, stamp));
      if (age > config_.max_sample_age_s) {
        return false;
      }
      *attitude = samples_.front().q_wb;
      return true;
    }
    if (stamp >= samples_.back().stamp) {
      const double age = std::fabs(
          math::timeDeltaSec(stamp, samples_.back().stamp));
      if (age > config_.max_sample_age_s) {
        return false;
      }
      *attitude = samples_.back().q_wb;
      return true;
    }
    for (std::size_t i = 1; i < samples_.size(); ++i) {
      if (stamp <= samples_[i].stamp) {
        const double span = math::timeDeltaSec(
            samples_[i].stamp, samples_[i - 1].stamp);
        const double ratio =
            span > math::kEps
                ? math::timeDeltaSec(stamp, samples_[i - 1].stamp) / span
                : 0.0;
        *attitude =
            math::slerp(samples_[i - 1].q_wb, samples_[i].q_wb, ratio);
        return true;
      }
    }
    return false;
  }

 private:
  struct AttitudeSample {
    ros::Time stamp;
    Quat q_wb;
    Vec3 gyro_b;
    bool has_gyro = false;
  };

  struct ImuDeltaSample {
    ros::Time start;
    ros::Time end;
    Vec3 omega_b;
  };

  static Quat integrateBodyRate(const Quat& q_wb, const Vec3& omega_b,
                                double dt) {
    const double omega_norm = math::norm(omega_b);
    if (!std::isfinite(dt) || dt <= 0.0 || omega_norm < 1e-10) {
      return q_wb;
    }
    const double half_angle = 0.5 * omega_norm * dt;
    const Vec3 axis = omega_b / omega_norm;
    const Quat delta{std::cos(half_angle), axis.x * std::sin(half_angle),
                     axis.y * std::sin(half_angle),
                     axis.z * std::sin(half_angle)};
    return math::normalize(
        math::fromEigen(math::toEigen(q_wb) * math::toEigen(delta)));
  }

  void pushAttitude(const ros::Time& stamp, const Quat& attitude,
                    const Vec3& gyro_b, bool has_gyro) {
    if (stamp.isZero()) {
      return;
    }
    if (!samples_.empty() && stamp < samples_.back().stamp) {
      return;
    }
    samples_.push_back(
        AttitudeSample{stamp, math::normalize(attitude), gyro_b, has_gyro});
    // Comparing scalar deltas avoids constructing a negative ros::Time when
    // simulation starts before buffer_duration_s.
    while (samples_.size() > 2 &&
           math::timeDeltaSec(stamp, samples_.front().stamp) >
               config_.buffer_duration_s) {
      samples_.pop_front();
    }
  }

  AttitudeHistoryConfig config_;
  std::deque<AttitudeSample> samples_;
  std::deque<ImuDeltaSample> imu_deltas_;
  ros::Time last_imu_rate_stamp_;
  Vec3 last_imu_rate_b_{};
  bool has_last_imu_rate_ = false;
};

}  // namespace dc_s2ekf
