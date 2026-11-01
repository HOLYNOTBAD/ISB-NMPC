#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/FsmState.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <swarm_msgs/InterceptionEvent.h>
#include <swarm_msgs/MassPoint.h>
#include <swarm_msgs/MassPoints.h>

namespace {

struct Target {
  int64_t id{0};
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
};

// Compute intercept time; returns infinity if unreachable.（计算拦截时间，无法拦截则返回无穷大）
double meetingTime(const Eigen::Vector3d& current_position,
                   double interceptor_speed,
                   const Eigen::Vector3d& target_position,
                   const Eigen::Vector3d& target_velocity) {
  const Eigen::Vector3d delta = target_position - current_position;
  const double distance = delta.norm();
  if (distance < 1e-6) {
    return 0.0;
  }

  const Eigen::Vector3d direction = delta / distance;
  const double target_parallel_speed = target_velocity.dot(direction);
  const double target_perp_speed_sq =
      target_velocity.squaredNorm() -
      target_parallel_speed * target_parallel_speed;

  // Too much lateral speed makes interception impossible.（横向速度过大则无法拦截）
  if (interceptor_speed * interceptor_speed <= target_perp_speed_sq) {
    return std::numeric_limits<double>::infinity();
  }

  const double closing_speed =
      std::sqrt(interceptor_speed * interceptor_speed - target_perp_speed_sq) -
      target_parallel_speed;
  if (closing_speed <= 1e-9) {
    return std::numeric_limits<double>::infinity();
  }

  return distance / closing_speed;
}

double distance(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
  return (lhs - rhs).norm();
}

// Evaluate total cost for a given attack order.（评估指定攻击顺序的总代价）
std::pair<std::vector<Target>, double> evaluateSequence(
    const std::vector<Target>& sequence,
    const Eigen::Vector3d& current_position,
    double interceptor_speed) {
  double elapsed_time = 0.0;
  double total_distance = 0.0;
  Eigen::Vector3d cursor = current_position;
  std::vector<Target> predicted_sequence;
  predicted_sequence.reserve(sequence.size());

  for (const Target& target : sequence) {
    Eigen::Vector3d predicted_position =
        target.position + target.velocity * elapsed_time;
    const double dt = meetingTime(cursor, interceptor_speed, predicted_position,
                                  target.velocity);
    if (!std::isfinite(dt)) {
      return {predicted_sequence, std::numeric_limits<double>::infinity()};
    }

    predicted_position += target.velocity * dt;
    total_distance += distance(cursor, predicted_position);
    cursor = predicted_position;
    elapsed_time += dt;

    predicted_sequence.push_back(target);
  }

  return {predicted_sequence, total_distance};
}

// Use full permutations to find the optimal order for small target sets.（小规模目标用全排列找最优顺序）
std::vector<Target> exactShortestSequence(std::vector<Target> targets,
                                          const Eigen::Vector3d& current_position,
                                          double interceptor_speed) {
  std::vector<Target> best_sequence;
  double best_cost = std::numeric_limits<double>::infinity();

  std::sort(targets.begin(), targets.end(),
            [](const Target& lhs, const Target& rhs) {
              return lhs.position.norm() < rhs.position.norm();
            });

  do {
    const auto result =
        evaluateSequence(targets, current_position, interceptor_speed);
    if (result.second < best_cost) {
      best_cost = result.second;
      best_sequence = result.first;
    }
  } while (std::next_permutation(
      targets.begin(), targets.end(), [](const Target& lhs, const Target& rhs) {
        return lhs.position.norm() < rhs.position.norm();
      }));

  return best_sequence;
}

// Use greedy approximation for large target sets.（大规模目标用贪心近似排序）
std::vector<Target> greedySequence(std::vector<Target> targets,
                                   const Eigen::Vector3d& current_position,
                                   double interceptor_speed) {
  std::vector<Target> sequence;
  sequence.reserve(targets.size());

  double elapsed_time = 0.0;
  Eigen::Vector3d cursor = current_position;
  while (!targets.empty()) {
    auto best_it = targets.begin();
    double best_time = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_predicted_position = best_it->position;

    for (auto it = targets.begin(); it != targets.end(); ++it) {
      // Predict target state first, then estimate intercept time.（先外推目标，再估计拦截时间）
      const Eigen::Vector3d predicted_position =
          it->position + it->velocity * elapsed_time;
      const double dt =
          meetingTime(cursor, interceptor_speed, predicted_position, it->velocity);
      if (dt < best_time) {
        best_time = dt;
        best_it = it;
        best_predicted_position = predicted_position + it->velocity * dt;
      }
    }

    Target selected = *best_it;
    sequence.push_back(selected);
    cursor = best_predicted_position;
    if (std::isfinite(best_time)) {
      elapsed_time += best_time;
    }
    targets.erase(best_it);
  }

  return sequence;
}

swarm_msgs::MassPoint toMessage(const Target& target) {
  swarm_msgs::MassPoint point;
  point.position.x = target.position.x();
  point.position.y = target.position.y();
  point.position.z = target.position.z();
  point.velocity.x = target.velocity.x();
  point.velocity.y = target.velocity.y();
  point.velocity.z = target.velocity.z();
  point.volume = target.id;
  return point;
}

}  // namespace

class TargetManagerNode {
 public:
  TargetManagerNode() : nh_(), pnh_("~") {
    // Load ROS params to override defaults.（读取ROS参数覆盖默认配置）
    pnh_.param<std::string>("masspoint_topic", masspoint_topic_,
                            masspoint_topic_);
    pnh_.param<std::string>("uav_state_topic", uav_state_topic_,
                            uav_state_topic_);
    pnh_.param<std::string>("attack_sequence_topic", attack_sequence_topic_,
                            attack_sequence_topic_);
    pnh_.param<std::string>("active_target_topic", active_target_topic_,
                            active_target_topic_);
    pnh_.param<std::string>("p_target_topic", p_target_topic_,
                            p_target_topic_);
    pnh_.param<std::string>("interception_event_topic",
                            interception_event_topic_,
                            interception_event_topic_);
    pnh_.param<std::string>("fsm_state_topic", fsm_state_topic_,
                            fsm_state_topic_);
    pnh_.param("local_frame/use_origin_topic", use_local_origin_topic_,
               use_local_origin_topic_);
    pnh_.param<std::string>("local_frame/origin_topic", local_origin_topic_,
                            local_origin_topic_);
    pnh_.param("publish_rate", publish_rate_hz_, publish_rate_hz_);
    pnh_.param("mav_vel", mav_vel_, mav_vel_);
    pnh_.param("max_exact_targets", max_exact_targets_, max_exact_targets_);
    pnh_.param("manager/has_target_pos", has_target_pos_, has_target_pos_);

    // Initialize subscribers and publishers.（初始化订阅与发布接口）
    masspoint_sub_ = nh_.subscribe(masspoint_topic_, 10,
                                   &TargetManagerNode::masspointCallback, this);
    uav_state_sub_ = nh_.subscribe(uav_state_topic_, 10,
                                   &TargetManagerNode::uavStateCallback, this);
    interception_event_sub_ =
        nh_.subscribe(interception_event_topic_, 10,
                      &TargetManagerNode::interceptionEventCallback, this);
    fsm_state_sub_ = nh_.subscribe(fsm_state_topic_, 10,
                                   &TargetManagerNode::fsmStateCallback, this);
    if (use_local_origin_topic_) {
      local_origin_sub_ =
          nh_.subscribe(local_origin_topic_, 1,
                        &TargetManagerNode::localOriginCallback, this);
    }
    attack_sequence_pub_ =
        nh_.advertise<swarm_msgs::MassPoints>(attack_sequence_topic_, 1, true);
    active_target_pub_ =
        nh_.advertise<swarm_msgs::MassPoints>(active_target_topic_, 1, true);
    p_target_pub_ =
        nh_.advertise<geometry_msgs::PointStamped>(p_target_topic_, 1, true);
    reset_trial_service_ = nh_.advertiseService(
        "/target_manager/reset_trial", &TargetManagerNode::resetTrialCallback,
        this);
    timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, publish_rate_hz_)),
        &TargetManagerNode::publishAttackSequence, this);

    ROS_INFO(
        "target_manager initialized. masspoint=%s, uav_state=%s, "
        "attack_sequence=%s, active_target=%s, p_target=%s, event=%s, fsm_state=%s, mav_vel=%.2f, "
        "max_exact_targets=%d, use_local_origin_topic=%s.",
        masspoint_topic_.c_str(), uav_state_topic_.c_str(),
        attack_sequence_topic_.c_str(), active_target_topic_.c_str(),
        p_target_topic_.c_str(), interception_event_topic_.c_str(),
        fsm_state_topic_.c_str(), mav_vel_,
        max_exact_targets_, use_local_origin_topic_ ? "true" : "false");
    if (!has_target_pos_) {
      ROS_WARN(
          "target_manager: manager/has_target_pos=false, skipping 3D target sorting/output.");
    }
  }

 private:
  bool resetTrialCallback(std_srvs::Trigger::Request&,
                          std_srvs::Trigger::Response& response) {
    targets_.clear();
    completed_target_ids_.clear();
    has_targets_ = false;
    active_target_id_ = -1;
    locked_target_id_ = -1;
    publishAttackSequence();
    response.success = true;
    response.message = "Target manager trial state cleared.";
    ROS_INFO("target_manager: reset_trial cleared pending and completed targets.");
    return true;
  }

  void masspointCallback(const swarm_msgs::MassPoints::ConstPtr& msg) {
    if (use_local_origin_topic_ && !has_local_origin_) {
      ROS_WARN_THROTTLE(
          1.0,
          "target_manager: waiting for local origin before accepting masspoints.");
      has_targets_ = false;
      targets_.clear();
      return;
    }

    // Rebuild pending targets and skip completed ones.（重建待处理目标并过滤已完成目标）
    targets_.clear();
    targets_.reserve(msg->points.size());
    for (const auto& point : msg->points) {
      if (point.volume >= 0 && completed_target_ids_.count(point.volume) > 0) {
        continue;
      }
      Target target;
      target.id = point.volume;
      target.position =
          Eigen::Vector3d(point.position.x, point.position.y, point.position.z);
      if (use_local_origin_topic_ && has_local_origin_) {
        target.position -= local_origin_;
      }
      target.velocity =
          Eigen::Vector3d(point.velocity.x, point.velocity.y, point.velocity.z);
      targets_.push_back(target);
    }
    has_targets_ = true;
  }

  void uavStateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_position_ =
        Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    has_uav_state_ = true;
  }

  void localOriginCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    local_origin_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
    if (!has_local_origin_) {
      ROS_INFO("target_manager: local origin received [%.3f %.3f %.3f].",
               local_origin_.x(), local_origin_.y(), local_origin_.z());
    }
    has_local_origin_ = true;
  }

  void interceptionEventCallback(
      const swarm_msgs::InterceptionEvent::ConstPtr& msg) {
    if (!msg->success || msg->target_id < 0) {
      return;
    }
    // Record completed targets to avoid replanning.（记录已完成目标，避免重复规划）
    completed_target_ids_.insert(msg->target_id);
    if (locked_target_id_ == msg->target_id) {
      locked_target_id_ = -1;
    }
    targets_.erase(std::remove_if(targets_.begin(), targets_.end(),
                                  [this](const Target& target) {
                                    return completed_target_ids_.count(
                                               target.id) > 0;
                                  }),
                   targets_.end());
    ROS_INFO("target_manager: target id=%ld completed, removed from sequence.",
             static_cast<long>(msg->target_id));
    publishAttackSequence();
  }

  void fsmStateCallback(const quadrotor_msgs::FsmState::ConstPtr& msg) {
    const bool was_in_exec_interception = in_exec_interception_;
    in_exec_interception_ = msg->state_name == "EXEC_INTERCEPTION";
    if (in_exec_interception_ && !was_in_exec_interception) {
      // Lock active target during interception.（执行拦截时锁定当前目标）
      locked_target_id_ = active_target_id_;
      ROS_INFO("target_manager: locking id=%ld during EXEC_INTERCEPTION.",
               static_cast<long>(locked_target_id_));
    } else if (!in_exec_interception_ && was_in_exec_interception) {
      locked_target_id_ = -1;
      ROS_INFO("target_manager: released id=%ld after EXEC_INTERCEPTION.",
               static_cast<long>(locked_target_id_));
    }
  }

  void publishAttackSequence(const ros::TimerEvent&) {
    publishAttackSequence();
  }

  void publishAttackSequence() {
    swarm_msgs::MassPoints msg;
    swarm_msgs::MassPoints active_msg;
    if (!has_target_pos_) {
      attack_sequence_pub_.publish(msg);
      active_target_pub_.publish(active_msg);
      publishPTarget(nullptr);
      ROS_INFO_THROTTLE(
          2.0,
          "target_manager: manager/has_target_pos=false, bypassing target sorting.");
      return;
    }

    if (!has_targets_ || targets_.empty()) {
      // Clear outputs when no targets remain.（无目标时清空输出）
      attack_sequence_pub_.publish(msg);
      active_target_pub_.publish(active_msg);
      publishPTarget(nullptr);
      ROS_WARN_THROTTLE(
          1.0, "target_manager: no pending targets on %s.",
          masspoint_topic_.c_str());
      return;
    }

    if (!has_uav_state_) {
      ROS_WARN_THROTTLE(
          1.0, "target_manager: waiting for UAV state on %s.",
          uav_state_topic_.c_str());
      return;
    }

    // Choose exact or greedy ordering by target count.（按目标数量选择精确或贪心排序）
    std::vector<Target> ordered_targets =
        static_cast<int>(targets_.size()) <= max_exact_targets_
            ? exactShortestSequence(targets_, current_position_, mav_vel_)
            : greedySequence(targets_, current_position_, mav_vel_);
    applyActiveTargetLock(&ordered_targets);

    // Publish sequence and active target.（发布序列和当前目标）
    msg.points.reserve(ordered_targets.size());
    for (const Target& target : ordered_targets) {
      msg.points.push_back(toMessage(target));
    }
    if (!ordered_targets.empty()) {
      active_msg.points.push_back(toMessage(ordered_targets.front()));
      active_target_id_ = ordered_targets.front().id;
    } else {
      active_target_id_ = -1;
    }
    attack_sequence_pub_.publish(msg);
    active_target_pub_.publish(active_msg);
    publishPTarget(ordered_targets.empty() ? nullptr : &ordered_targets.front());

    std::ostringstream sequence_stream;
    for (std::size_t i = 0; i < msg.points.size(); ++i) {
      if (i > 0) {
        sequence_stream << " -> ";
      }
      sequence_stream << msg.points[i].volume;
    }

    ROS_INFO_THROTTLE(
        1.0,
        "target_manager: published attack sequence [%s] with %zu target(s). First "
        "target id=%ld [%.2f %.2f %.2f].",
        sequence_stream.str().c_str(),
        msg.points.size(),
        msg.points.empty() ? 0L : static_cast<long>(msg.points.front().volume),
        msg.points.empty() ? 0.0 : msg.points.front().position.x,
        msg.points.empty() ? 0.0 : msg.points.front().position.y,
        msg.points.empty() ? 0.0 : msg.points.front().position.z);
  }

  void publishPTarget(const Target* target) {
    // Publish NMPC tracking target.（发布NMPC跟踪目标点）
    geometry_msgs::PointStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    if (target != nullptr) {
      msg.point.x = target->position.x();
      msg.point.y = target->position.y();
      msg.point.z = target->position.z();
    }
    p_target_pub_.publish(msg);
  }

  void applyActiveTargetLock(std::vector<Target>* ordered_targets) {
    if (ordered_targets == nullptr || locked_target_id_ < 0) {
      return;
    }
    // Keep locked target at the front.（将锁定目标保持在首位）
    auto it = std::find_if(ordered_targets->begin(), ordered_targets->end(),
                           [this](const Target& target) {
                             return target.id == locked_target_id_;
                           });
    if (it == ordered_targets->end()) {
      locked_target_id_ = -1;
      return;
    }
    std::rotate(ordered_targets->begin(), it, it + 1);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber masspoint_sub_;
  ros::Subscriber uav_state_sub_;
  ros::Subscriber interception_event_sub_;
  ros::Subscriber fsm_state_sub_;
  ros::Subscriber local_origin_sub_;
  ros::Publisher attack_sequence_pub_;
  ros::Publisher active_target_pub_;
  ros::Publisher p_target_pub_;
  ros::ServiceServer reset_trial_service_;
  ros::Timer timer_;

  std::string masspoint_topic_{"/balloons/masspoint"};
  std::string uav_state_topic_{"/nmpc/uavStateEstimate"};
  std::string attack_sequence_topic_{"/target_manager/attack_sequence"};
  std::string active_target_topic_{"/target_manager/active_target"};
  std::string p_target_topic_{"/nmpc/p_target"};
  std::string interception_event_topic_{"/balloons/event"};
  std::string fsm_state_topic_{"/fsm/fsm_state"};
  std::string local_origin_topic_{"/local_frame/origin"};
  double publish_rate_hz_{10.0};
  double mav_vel_{14.0};
  int max_exact_targets_{8};

  std::vector<Target> targets_;
  std::unordered_set<int64_t> completed_target_ids_;
  Eigen::Vector3d current_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d local_origin_{Eigen::Vector3d::Zero()};
  bool has_targets_{false};
  bool has_uav_state_{false};
  bool has_target_pos_{true};
  bool use_local_origin_topic_{false};
  bool has_local_origin_{false};
  bool in_exec_interception_{false};
  int64_t active_target_id_{-1};
  int64_t locked_target_id_{-1};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_manager");
  TargetManagerNode node;
  ros::spin();
  return 0;
}
