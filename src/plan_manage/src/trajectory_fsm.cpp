#include "plan_manage/trajectory_fsm.h"

#include <algorithm>
#include <cmath>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <quadrotor_common/trajectory.h>

namespace plan_manage {
namespace {

double positionDistance(const geometry_msgs::Point& lhs,
                        const geometry_msgs::Point& rhs) {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

}  // namespace

// ============================================================================
//  Parameters and ROS visualization （参数与ROS可视化）
// ============================================================================

void TrajectoryFSM::loadParams(ros::NodeHandle& nh,
                                     const std::string& prefix) {
  nh.param(prefix + "/exec/tracking_hold_enter_distance",
           hold_enter_distance_, hold_enter_distance_);
  nh.param(prefix + "/exec/tracking_hold_exit_distance",
           hold_exit_distance_, hold_exit_distance_);
  nh.param(prefix + "/exec/tracking_lost_distance",
           lost_distance_, lost_distance_);
  nh.param(prefix + "/exec/tracking_lost_confirmation_sec",
           lost_confirmation_sec_, lost_confirmation_sec_);
  nh.param(prefix + "/exec/tracking_max_hold_sec",
           max_hold_sec_, max_hold_sec_);
  nh.param(prefix + "/exec/hold_enabled", hold_enabled_, hold_enabled_);
  nh.param(prefix + "/exec/horizon_duration", horizon_duration_,
           horizon_duration_);
  nh.param(prefix + "/exec/sample_dt", sample_dt_, sample_dt_);
  nh.param(prefix + "/exec/horizon_points", horizon_points_, horizon_points_);

  hold_enter_distance_ = std::max(0.0, hold_enter_distance_);
  hold_exit_distance_ =
      std::clamp(hold_exit_distance_, 0.0, hold_enter_distance_);
  lost_distance_ = std::max(lost_distance_, hold_enter_distance_);
  lost_confirmation_sec_ = std::max(0.0, lost_confirmation_sec_);
  max_hold_sec_ = std::max(0.0, max_hold_sec_);
  horizon_duration_ = std::max(0.0, horizon_duration_);
  sample_dt_ = std::max(1e-3, sample_dt_);
  horizon_points_ = std::max(1, horizon_points_);
}

void TrajectoryFSM::initVisualization(
    ros::NodeHandle& nh, const std::string& global_topic,
    const std::string& horizon_topic) {
  global_vis_pub_ = nh.advertise<nav_msgs::Path>(global_topic, 2);
  horizon_vis_pub_ = nh.advertise<nav_msgs::Path>(horizon_topic, 2);
}

void TrajectoryFSM::reset() { context_ = Context(); }

// ============================================================================
//  Utils （工具函数）
// ============================================================================

quadrotor_msgs::TrajectoryPoint TrajectoryFSM::makeZeroPoint() {
  quadrotor_msgs::TrajectoryPoint point;
  point.pose.orientation.w = 1.0;
  return point;
}

void TrajectoryFSM::clearDerivatives(
    quadrotor_msgs::TrajectoryPoint* point) {
  if (point == nullptr) return;
  point->acceleration = geometry_msgs::Twist();
  point->jerk = geometry_msgs::Twist();
  point->snap = geometry_msgs::Twist();
  point->heading_rate = 0.0;
  point->heading_acceleration = 0.0;
}

quadrotor_msgs::TrajectoryPoint TrajectoryFSM::makeCurrentStatePoint(
    const nav_msgs::Odometry& uav_state) {
  auto point = makeZeroPoint();
  point.pose = uav_state.pose.pose;
  point.velocity = uav_state.twist.twist;
  clearDerivatives(&point);
  return point;
}

void TrajectoryFSM::load(
    const quadrotor_msgs::Trajectory& trajectory) {
  // Load trajectory into memory without starting execution.
  // （加载轨迹到内存但不开始执行）
  context_ = Context();
  context_.has_traj = !trajectory.points.empty();
  context_.global_traj = trajectory;
  if (context_.has_traj) {
    context_.total_duration = trajectory.points.back().time_from_start.toSec();
  }
}

void TrajectoryFSM::execute(const ros::Time& start_time) {
  // Start executing the active trajectory from the given wall-clock time.
  // （从给定墙钟时间开始执行活动轨迹）
  context_.start_time = start_time;
  context_.hold_start_time = ros::Time(0);
  context_.lost_exceeded_since = ros::Time(0);
  context_.accumulated_hold_time = 0.0;
  context_.tracking = true;
  context_.tracking_state = TrackingState::NORMAL;
}

void TrajectoryFSM::update(const ros::Time& now,
                                 const nav_msgs::Odometry& uav_state) {
  // Advance trajectory execution progress: compute current_time and tracking
  // status from wall clock. （更新活动轨迹执行进度：从墙钟计算current_time和tracking状态）
  if (!context_.has_traj || context_.global_traj.points.empty()) {
    context_.tracking = false;
    context_.tracking_state = TrackingState::LOST;
    context_.is_finished = false;
    return;
  }

  const auto trajectory = quadrotor_common::Trajectory(context_.global_traj);
  const auto referenceErrorAt = [&](double trajectory_time) {
    const double clamped_time =
        std::clamp(trajectory_time, 0.0, context_.total_duration);
    const auto reference = trajectory
        .getStateAtTime(ros::Duration(clamped_time)).toRosMessage();
    return positionDistance(uav_state.pose.pose.position,
                            reference.pose.position);
  };

  if (context_.tracking_state == TrackingState::LOST) {
    context_.tracking = false;
    context_.is_finished = false;
    return;
  }

  if (context_.tracking_state == TrackingState::HOLD) {
    // Freeze execution time so the UAV can catch up before the trajectory clock
    // resumes, preventing sustained tracking loss.
    // （冻结执行时间，等飞机追上后再恢复，避免持续脱轨）
    const double error = referenceErrorAt(context_.current_time);
    const double hold_duration = (now - context_.hold_start_time).toSec();

    if (error > lost_distance_) {
      if (context_.lost_exceeded_since.isZero()) {
        context_.lost_exceeded_since = now;
      }
    } else {
      context_.lost_exceeded_since = ros::Time(0);
    }

    const bool lost_confirmed = !context_.lost_exceeded_since.isZero() &&
        (now - context_.lost_exceeded_since).toSec() >= lost_confirmation_sec_;
    const bool hold_timed_out =
        max_hold_sec_ > 0.0 && hold_duration >= max_hold_sec_;

    if (lost_confirmed || hold_timed_out) {
      context_.tracking_state = TrackingState::LOST;
      context_.tracking = false;
      ROS_ERROR("[trajectory] HOLD -> LOST: error=%.2f m, hold=%.2f s, "
                "reason=%s.", error, hold_duration,
                lost_confirmed ? "distance" : "hold timeout");
    } else if (error <= hold_exit_distance_) {
      context_.accumulated_hold_time += std::max(0.0, hold_duration);
      context_.hold_start_time = ros::Time(0);
      context_.lost_exceeded_since = ros::Time(0);
      context_.tracking_state = TrackingState::NORMAL;
      context_.tracking = true;
      ROS_INFO("[trajectory] HOLD -> NORMAL: error=%.2f m.", error);
    } else {
      context_.tracking = true;
      ROS_INFO_THROTTLE(0.5,
                        "[trajectory] HOLD: error=%.2f m, hold=%.2f s.",
                        error, hold_duration);
    }
  } else {
    // Nominal execution time, subtracting accumulated hold duration.
    // （理论执行时间，扣除累计暂停时长）
    const double nominal_time = std::max(
        0.0, (now - context_.start_time).toSec() -
                 context_.accumulated_hold_time);
    const double candidate_time = std::clamp(
        // current_time must never go backwards.
        // （current_time不允许回退）
        std::max(nominal_time, context_.current_time), 0.0,
        context_.total_duration);
    const double error = referenceErrorAt(candidate_time);

    if (hold_enabled_ && error > hold_enter_distance_) {
      context_.tracking_state = TrackingState::HOLD;
      context_.hold_start_time = now;
      context_.lost_exceeded_since =
          error > lost_distance_ ? now : ros::Time(0);
      context_.tracking = true;
      ROS_WARN("[trajectory] NORMAL -> HOLD: error=%.2f m, time=%.2f s.",
               error, candidate_time);
    } else if (!hold_enabled_ && error > lost_distance_) {
      context_.tracking_state = TrackingState::LOST;
      context_.tracking = false;
      ROS_ERROR("[trajectory] NORMAL -> LOST: error=%.2f m.", error);
    } else {
      context_.current_time = candidate_time;
      context_.tracking = true;
    }
  }

  context_.is_finished = context_.tracking_state != TrackingState::LOST &&
      context_.total_duration > 0.0 &&
      context_.current_time >= context_.total_duration;
}

void TrajectoryFSM::buildHorizon() {
  // Extract the horizon window from global_traj.
  // （从global_traj截取horizon时间窗）
  quadrotor_msgs::Trajectory horizon = context_.global_traj;
  horizon.header.stamp = ros::Time::now();
  horizon.points.clear();

  if (!context_.has_traj || context_.global_traj.points.empty()) {
    // If there is no active trajectory, return an empty trajectory.
    // （无活动轨迹时返回空轨迹）
    context_.horizon_traj = horizon;
    context_.horizon_valid = false;
    return;
  }

  const double current_time =
      std::clamp(context_.current_time, 0.0, context_.total_duration);
  // Time window and point-count limits. （时间窗与点数限制）
  const double horizon_end_time = current_time + horizon_duration_;
  const std::size_t max_points = static_cast<std::size_t>(horizon_points_);
  horizon.points.reserve(
      std::min(context_.global_traj.points.size(), max_points));

  const auto trajectory = quadrotor_common::Trajectory(context_.global_traj);
  auto anchor = trajectory.getStateAtTime(
      ros::Duration(current_time)).toRosMessage();
  anchor.time_from_start = ros::Duration(0.0);
  horizon.points.push_back(anchor);

  double last_added_time = current_time;
  // Enforce sample_dt spacing so the horizon covers horizon_duration instead
  // of only (max_points x raw_point_dt). Without this filter, a 100 Hz
  // trajectory with max_points=50 would send only 0.5 s to the NMPC instead of
  // 2.5 s. （强制按sample_dt间隔采样，确保horizon覆盖完整的horizon_duration）
  for (const auto& point : context_.global_traj.points) {
    const double point_time = point.time_from_start.toSec();
    if (point_time <= current_time) continue;
    if (point_time > horizon_end_time) break;
    if (point_time - last_added_time < sample_dt_ - 1e-9) continue;

    auto shifted_point = point;
    shifted_point.time_from_start =
        ros::Duration(std::max(0.0, point_time - current_time));
    horizon.points.push_back(shifted_point);
    last_added_time = point_time;
    if (horizon.points.size() >= max_points) break;
  }

  context_.horizon_traj = horizon;
  context_.horizon_valid = !horizon.points.empty();
}

void TrajectoryFSM::publishHorizon(ros::Publisher& publisher) {
  // Publish the previously built horizon trajectory.
  // （发布已构建的horizon_traj）
  context_.horizon_traj.header.stamp = ros::Time::now();
  context_.last_publish_time = context_.horizon_traj.header.stamp;
  publisher.publish(context_.horizon_traj);
}

void TrajectoryFSM::visualize() const {
  if (context_.has_traj && !context_.global_traj.points.empty()) {
    global_vis_pub_.publish(
        quadrotor_common::Trajectory(context_.global_traj).toRosPath());
  }
  if (context_.horizon_valid && !context_.horizon_traj.points.empty()) {
    horizon_vis_pub_.publish(
        quadrotor_common::Trajectory(context_.horizon_traj).toRosPath());
  }
}

void TrajectoryFSM::updateGlobal(
    const quadrotor_msgs::Trajectory& replanned_trajectory) {
  // Replace the tail of global_traj without resetting the execution clock.
  // （不重置执行时钟，使用重规划轨迹段替换 global_traj 尾部）
  if (!context_.has_traj || replanned_trajectory.points.empty()) return;

  double seam_global_time = context_.current_time;
  // The horizon endpoint expressed in global_traj time.
  // （horizon末端在global_traj坐标系中的时间）
  if (context_.horizon_valid && !context_.horizon_traj.points.empty()) {
    seam_global_time +=
        context_.horizon_traj.points.back().time_from_start.toSec();
  }

  auto& points = context_.global_traj.points;
  // Truncate global_traj: retain points whose time is before the seam.
  // （截断global_traj，保留时间不晚于接缝时间的点）
  const auto cut = std::upper_bound(
      points.begin(), points.end(), seam_global_time,
      [](double time, const quadrotor_msgs::TrajectoryPoint& point) {
        return time < point.time_from_start.toSec();
      });
  points.erase(cut, points.end());
  const double seam_time =
      points.empty() ? seam_global_time : points.back().time_from_start.toSec();

  std::size_t index = replanned_trajectory.points.front()
          .time_from_start.toSec() <= 1e-6 ? 1 : 0;
  // Append the replanned segment, skipping the duplicate t≈0 seam point.
  // （追加新规划段，跳过t≈0的重复接缝点）
  for (; index < replanned_trajectory.points.size(); ++index) {
    auto point = replanned_trajectory.points[index];
    point.time_from_start =
        ros::Duration(seam_time + point.time_from_start.toSec());
    points.push_back(point);
  }

  context_.total_duration =
      points.empty() ? 0.0 : points.back().time_from_start.toSec();
  // Horizon will be rebuilt on the next cycle. （horizon将在下一周期重建）
  context_.horizon_valid = false;
}

}  // namespace plan_manage
