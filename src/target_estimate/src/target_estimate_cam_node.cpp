#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <sstream>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <swarm_msgs/BoundingBox.h>
#include <swarm_msgs/BoundingBoxes.h>
#include <swarm_msgs/MassPoints.h>

class TargetEstimateCamNode {
 public:
  explicit TargetEstimateCamNode(ros::NodeHandle& nh) : nh_(nh) {
    loadParams();

    p_image_sub_ =
        nh_.subscribe(p_image_input_topic_, 10,
                      &TargetEstimateCamNode::pImageCallback, this);
    masspoint_sub_ =
        nh_.subscribe(masspoint_topic_, 10,
                      &TargetEstimateCamNode::masspointCallback, this);
    uav_state_sub_ =
        nh_.subscribe(uav_state_estimate_topic_, 10,
                      &TargetEstimateCamNode::uavStateCallback, this);
    balloon_intercepted_sub_ =
        nh_.subscribe(balloon_intercepted_topic_, 10,
                      &TargetEstimateCamNode::balloonInterceptedCallback, this);

    p_image_pub_ =
        nh_.advertise<geometry_msgs::PointStamped>(p_image_topic_, 10);
    q_image_pub_ =
        nh_.advertise<geometry_msgs::QuaternionStamped>(q_image_topic_, 10);
    target_in_main_view_pub_ =
        nh_.advertise<std_msgs::Bool>(target_in_main_view_topic_, 1, true);
    target_state_pub_ =
        nh_.advertise<nav_msgs::Odometry>(target_state_estimate_topic_, 10);
    if (publish_p_target_) {
      p_target_pub_ =
          nh_.advertise<geometry_msgs::PointStamped>(p_target_topic_, 1, true);
    }
    fsm_intercepted_pub_ =
        nh_.advertise<std_msgs::Bool>(fsm_intercepted_topic_, 1, true);
    publishFsmIntercepted(false);
    publishTargetInMainView(false);

    ROS_INFO(
        "target_estimate_cam initialized. p_image_input=%s, masspoint=%s, "
        "target_state=%s, p_target=%s, p_image=%s, q_image=%s, "
        "target_in_main_view=%s, balloon_intercepted=%s, "
        "fsm_intercepted=%s.",
        p_image_input_topic_.c_str(), masspoint_topic_.c_str(),
        target_state_estimate_topic_.c_str(), p_target_topic_.c_str(),
        p_image_topic_.c_str(), q_image_topic_.c_str(),
        target_in_main_view_topic_.c_str(),
        balloon_intercepted_topic_.c_str(), fsm_intercepted_topic_.c_str());
    if (!has_target_pos_) {
      ROS_WARN(
          "target_estimate_cam: manager/has_target_pos=false, using bearing-only q_image from detections.");
    }
  }

 private:
  void loadParams() {
    int img_w = 640;
    int img_h = 480;
    ros::param::param("/camera/img_width", img_w, img_w);
    ros::param::param("/camera/img_height", img_h, img_h);
    ros::param::param("/camera/img_f", img_f_, img_f_);
    img_cx_ = img_w * 0.5;
    img_cy_ = img_h * 0.5;

    nh_.param<std::string>("p_image/input_topic", p_image_input_topic_,
                           p_image_input_topic_);
    nh_.param<std::string>("masspoint/topic", masspoint_topic_,
                           masspoint_topic_);
    nh_.param<std::string>("p_image/topic", p_image_topic_,
                           p_image_topic_);
    nh_.param<std::string>("q_image/topic", q_image_topic_, q_image_topic_);
    nh_.param<std::string>("target_in_main_view/topic",
                           target_in_main_view_topic_,
                           target_in_main_view_topic_);
    nh_.param<std::string>("target_state_estimate/topic",
                           target_state_estimate_topic_,
                           target_state_estimate_topic_);
    nh_.param<std::string>("p_target/topic", p_target_topic_,
                           p_target_topic_);
    nh_.param("p_target/publish", publish_p_target_, publish_p_target_);
    nh_.param<std::string>("uav_state_estimate/topic",
                           uav_state_estimate_topic_,
                           uav_state_estimate_topic_);
    nh_.param<std::string>("balloon_intercepted/topic",
                           balloon_intercepted_topic_,
                           balloon_intercepted_topic_);
    nh_.param<std::string>("fsm_intercepted/topic", fsm_intercepted_topic_,
                           fsm_intercepted_topic_);
    nh_.param<std::string>("image_frame_id", image_frame_id_, image_frame_id_);
    nh_.param<std::string>("target_frame_id", target_frame_id_,
                           target_frame_id_);
    nh_.param("max_target_pixel_error", max_target_pixel_error_,
              max_target_pixel_error_);
    nh_.param("manager/has_target_pos", has_target_pos_, has_target_pos_);
    nh_.param("estimator/confirmation_frames", confirmation_frames_,
              confirmation_frames_);
    nh_.param("estimator/max_tracking_jump_px", max_tracking_jump_px_,
              max_tracking_jump_px_);
    nh_.param("estimator/max_tracking_jump_rate_px_s",
              max_tracking_jump_rate_px_s_, max_tracking_jump_rate_px_s_);
    nh_.param("estimator/pending_match_px", pending_match_px_,
              pending_match_px_);
    nh_.param("estimator/lock_timeout", lock_timeout_, lock_timeout_);

    confirmation_frames_ = std::max(1, confirmation_frames_);
    max_tracking_jump_px_ = std::max(0.0, max_tracking_jump_px_);
    max_tracking_jump_rate_px_s_ =
        std::max(0.0, max_tracking_jump_rate_px_s_);
    pending_match_px_ = std::max(0.0, pending_match_px_);
    lock_timeout_ = std::max(0.0, lock_timeout_);

    std::vector<double> q_B_C(4);
    if (nh_.getParam("/camera/q_B_C", q_B_C) && q_B_C.size() == 4) {
      q_B_C_ = Eigen::Quaterniond(q_B_C[0], q_B_C[1], q_B_C[2], q_B_C[3]);
      if (q_B_C_.norm() > 1e-6) {
        q_B_C_.normalize();
      } else {
        q_B_C_ = Eigen::Quaterniond(0.5, -0.5, 0.5, -0.5);
      }
    }

    std::vector<double> p_B_C(3);
    if (nh_.getParam("/camera/p_B_C", p_B_C) && p_B_C.size() == 3) {
      p_B_C_ = Eigen::Vector3d(p_B_C[0], p_B_C[1], p_B_C[2]);
    }
  }

  void pImageCallback(const swarm_msgs::BoundingBoxes::ConstPtr& msg) {
    const ros::Time measurement_time =
        msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    if (msg->bounding_boxes.empty()) {
      has_p_image_ = false;
      resetPendingCandidate();
      expireBearingLock(measurement_time);
      publishTargetInMainView(false);
      publishEquivalentQImageIfReady(msg->header.stamp);
      return;
    }

    const swarm_msgs::BoundingBox* box = selectActiveTargetBox(*msg);
    if (box == nullptr) {
      has_p_image_ = false;
      publishTargetInMainView(false);
      publishEquivalentQImageIfReady(msg->header.stamp);
      return;
    }

    if (msg->bounding_boxes.size() > 1) {
      ROS_DEBUG_THROTTLE(
          0.5,
          "target_estimate_cam: selected active-target bbox center=[%.1f %.1f] from %zu candidates.",
          0.5 * (box->xmin + box->xmax), 0.5 * (box->ymin + box->ymax),
          msg->bounding_boxes.size());
    }

    double u = 0.5 * (box->xmin + box->xmax);
    double v = 0.5 * (box->ymin + box->ymax);
    if (!has_target_pos_ &&
        !acceptBearingOnlyMeasurement(u, v, measurement_time)) {
      has_p_image_ = false;
      publishTargetInMainView(false);
      return;
    }
    publishPImageMeasurement(u, v);
  }

  const swarm_msgs::BoundingBox* selectActiveTargetBox(
      const swarm_msgs::BoundingBoxes& msg) const {
    if (msg.bounding_boxes.empty()) {
      return nullptr;
    }

    if (!has_target_pos_) {
      if (has_bearing_lock_) {
        const swarm_msgs::BoundingBox* nearest = nullptr;
        double nearest_error = std::numeric_limits<double>::infinity();
        for (const auto& box : msg.bounding_boxes) {
          const double u = 0.5 * (box.xmin + box.xmax);
          const double v = 0.5 * (box.ymin + box.ymax);
          const double error = std::hypot(u - tracked_u_, v - tracked_v_);
          if (error < nearest_error) {
            nearest_error = error;
            nearest = &box;
          }
        }
        if (nearest != nullptr && nearest_error <= max_tracking_jump_px_) {
          return nearest;
        }
      }
      return selectLargestBox(msg);
    }

    double projected_u = 0.0;
    double projected_v = 0.0;
    if (!projectActiveTargetToImage(&projected_u, &projected_v)) {
      if (msg.bounding_boxes.size() == 1) {
        return &msg.bounding_boxes.front();
      }
      ROS_WARN_THROTTLE(
          1.0,
          "target_estimate_cam: multiple detections but active target cannot be projected; refusing ambiguous bbox.");
      return nullptr;
    }

    const swarm_msgs::BoundingBox* best_box = nullptr;
    double best_error = std::numeric_limits<double>::infinity();
    for (const auto& box : msg.bounding_boxes) {
      const double u = 0.5 * (box.xmin + box.xmax);
      const double v = 0.5 * (box.ymin + box.ymax);
      const double error = std::hypot(u - projected_u, v - projected_v);
      if (error < best_error) {
        best_error = error;
        best_box = &box;
      }
    }

    if (best_box == nullptr || best_error > max_target_pixel_error_) {
      ROS_WARN_THROTTLE(
          1.0,
          "target_estimate_cam: no bbox matches active target projection [%.1f %.1f], best_error=%.1f px, threshold=%.1f px.",
          projected_u, projected_v, best_error, max_target_pixel_error_);
      return nullptr;
    }
    return best_box;
  }

  const swarm_msgs::BoundingBox* selectLargestBox(
      const swarm_msgs::BoundingBoxes& msg) const {
    const swarm_msgs::BoundingBox* best_box = nullptr;
    double best_area = -1.0;
    for (const auto& box : msg.bounding_boxes) {
      const double width =
          std::max(0.0, static_cast<double>(box.xmax - box.xmin));
      const double height =
          std::max(0.0, static_cast<double>(box.ymax - box.ymin));
      const double area = width * height;
      if (area > best_area) {
        best_area = area;
        best_box = &box;
      }
    }
    return best_box;
  }

  bool acceptBearingOnlyMeasurement(double& u, double& v,
                                    const ros::Time& stamp) {
    expireBearingLock(stamp);

    if (has_bearing_lock_) {
      const double jump = std::hypot(u - tracked_u_, v - tracked_v_);
      double allowed_jump = max_tracking_jump_px_;
      if (!last_bearing_measurement_time_.isZero()) {
        const double dt = (stamp - last_bearing_measurement_time_).toSec();
        if (std::isfinite(dt) && dt > 0.0) {
          allowed_jump = std::max(
              allowed_jump,
              max_tracking_jump_rate_px_s_ * std::min(dt, lock_timeout_));
        }
      }
      if (jump <= allowed_jump) {
        tracked_u_ = u;
        tracked_v_ = v;
        last_bearing_measurement_time_ = stamp;
        resetPendingCandidate();
        return true;
      }
      ROS_WARN_THROTTLE(
          0.5,
          "target_estimate_cam: rejected bearing jump %.1f px (allowed %.1f px).",
          jump, allowed_jump);
    }

    if (!has_pending_candidate_ ||
        std::hypot(u - pending_u_, v - pending_v_) > pending_match_px_) {
      pending_u_ = u;
      pending_v_ = v;
      pending_count_ = 1;
      has_pending_candidate_ = true;
    } else {
      ++pending_count_;
      const double weight = 1.0 / static_cast<double>(pending_count_);
      pending_u_ += weight * (u - pending_u_);
      pending_v_ += weight * (v - pending_v_);
    }

    if (pending_count_ < confirmation_frames_) {
      ROS_DEBUG_THROTTLE(
          0.5,
          "target_estimate_cam: bearing candidate pending confirmation %d/%d at [%.1f %.1f].",
          pending_count_, confirmation_frames_, pending_u_, pending_v_);
      return false;
    }

    tracked_u_ = pending_u_;
    tracked_v_ = pending_v_;
    has_bearing_lock_ = true;
    last_bearing_measurement_time_ = stamp;
    resetPendingCandidate();
    u = tracked_u_;
    v = tracked_v_;
    ROS_INFO("target_estimate_cam: bearing target confirmed at [%.1f %.1f].",
             u, v);
    return true;
  }

  void expireBearingLock(const ros::Time& stamp) {
    if (has_bearing_lock_ && !last_bearing_measurement_time_.isZero() &&
        (stamp - last_bearing_measurement_time_).toSec() > lock_timeout_) {
      has_bearing_lock_ = false;
      ROS_INFO("target_estimate_cam: bearing target lock expired.");
    }
  }

  void resetPendingCandidate() {
    has_pending_candidate_ = false;
    pending_count_ = 0;
  }

  bool projectActiveTargetToImage(double* u, double* v) const {
    if (u == nullptr || v == nullptr || !has_uav_state_ || !has_target_ ||
        std::abs(img_f_) < 1e-6) {
      return false;
    }

    Eigen::Quaterniond q_W_B(
        latest_uav_state_.pose.pose.orientation.w,
        latest_uav_state_.pose.pose.orientation.x,
        latest_uav_state_.pose.pose.orientation.y,
        latest_uav_state_.pose.pose.orientation.z);
    if (q_W_B.norm() <= 1e-6) {
      return false;
    }
    q_W_B.normalize();

    const Eigen::Vector3d p_W(
        latest_target_state_.pose.pose.position.x -
            latest_uav_state_.pose.pose.position.x,
        latest_target_state_.pose.pose.position.y -
            latest_uav_state_.pose.pose.position.y,
        latest_target_state_.pose.pose.position.z -
            latest_uav_state_.pose.pose.position.z);
    if (p_W.norm() <= 1e-6) {
      return false;
    }

    const Eigen::Vector3d p_B = q_W_B.inverse() * p_W - p_B_C_;
    const Eigen::Vector3d ray_C = q_B_C_.inverse() * p_B;
    if (ray_C.z() <= 1e-6 || !ray_C.allFinite()) {
      return false;
    }

    *u = img_cx_ + img_f_ * ray_C.x() / ray_C.z();
    *v = img_cy_ + img_f_ * ray_C.y() / ray_C.z();
    return std::isfinite(*u) && std::isfinite(*v);
  }

  void publishPImageMeasurement(double u, double v) {
    if (std::abs(img_f_) < 1e-6) {
      has_p_image_ = false;
      ROS_WARN_THROTTLE(1.0,
                        "camera/img_f is too small, cannot compute p_image.");
      return;
    }

    const Eigen::Vector3d ray((u - img_cx_) / img_f_,
                              (v - img_cy_) / img_f_, 1.0);
    Eigen::Quaterniond q_image =
        Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(),
                                           ray.normalized());
    q_image.normalize();
    has_p_image_ = true;
    publishTargetInMainView(true);

    geometry_msgs::PointStamped p_image_msg;
    p_image_msg.header.stamp = ros::Time::now();
    p_image_msg.header.frame_id = image_frame_id_;
    p_image_msg.point.x = ray.x();
    p_image_msg.point.y = ray.y();
    p_image_msg.point.z = ray.z();
    p_image_pub_.publish(p_image_msg);

    geometry_msgs::QuaternionStamped q_image_msg;
    q_image_msg.header = p_image_msg.header;
    if (has_target_ && !latest_target_state_.child_frame_id.empty()) {
      q_image_msg.header.frame_id = latest_target_state_.child_frame_id;
    }
    q_image_msg.quaternion.w = q_image.w();
    q_image_msg.quaternion.x = q_image.x();
    q_image_msg.quaternion.y = q_image.y();
    q_image_msg.quaternion.z = q_image.z();
    q_image_pub_.publish(q_image_msg);
  }

  void masspointCallback(const swarm_msgs::MassPoints::ConstPtr& msg) {
    if (!has_target_pos_) {
      return;
    }

    if (msg->points.empty()) {
      has_target_ = false;
      has_p_image_ = false;
      publishTargetInMainView(false);
      return;
    }

    const auto& pt = msg->points.front();
    std::ostringstream child_frame;
    child_frame << "target_" << pt.volume;
    const std::string child_frame_id = child_frame.str();
    const bool target_changed =
        !has_target_ || latest_target_state_.child_frame_id != child_frame_id;

    if (target_changed) {
      has_p_image_ = false;
      publishTargetInMainView(false);
      ROS_INFO("target_estimate_cam: active target switched to %s.",
               child_frame_id.c_str());
    }

    nav_msgs::Odometry target_state;
    target_state.header.stamp = ros::Time::now();
    target_state.header.frame_id = target_frame_id_;
    target_state.child_frame_id = child_frame_id;
    target_state.pose.pose.position.x = pt.position.x;
    target_state.pose.pose.position.y = pt.position.y;
    target_state.pose.pose.position.z = pt.position.z;
    target_state.pose.pose.orientation.w = 1.0;
    target_state.twist.twist.linear.x = pt.velocity.x;
    target_state.twist.twist.linear.y = pt.velocity.y;
    target_state.twist.twist.linear.z = pt.velocity.z;

    has_target_ = true;
    latest_target_state_ = target_state;
    target_state_pub_.publish(target_state);
    publishPTarget(target_state);
    if (!has_p_image_) {
      publishEquivalentQImageIfReady(target_state.header.stamp);
    }
  }

  void uavStateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    latest_uav_state_ = *msg;
    has_uav_state_ = true;
    if (!has_p_image_) {
      publishEquivalentQImageIfReady(msg->header.stamp);
    }
  }

  void balloonInterceptedCallback(const std_msgs::Bool::ConstPtr& msg) {
    publishFsmIntercepted(msg->data);
  }

  void publishFsmIntercepted(bool intercepted) {
    std_msgs::Bool msg;
    msg.data = intercepted;
    fsm_intercepted_pub_.publish(msg);
  }

  void publishTargetInMainView(bool in_view) {
    std_msgs::Bool msg;
    msg.data = in_view;
    target_in_main_view_pub_.publish(msg);
  }

  void publishPTarget(const nav_msgs::Odometry& target_state) {
    if (!publish_p_target_) {
      return;
    }
    geometry_msgs::PointStamped msg;
    msg.header = target_state.header;
    msg.point = target_state.pose.pose.position;
    p_target_pub_.publish(msg);
  }

  void publishEquivalentQImageIfReady(const ros::Time& stamp) {
    if (!has_uav_state_ || !has_target_) {
      return;
    }

    Eigen::Quaterniond q_W_B(
        latest_uav_state_.pose.pose.orientation.w,
        latest_uav_state_.pose.pose.orientation.x,
        latest_uav_state_.pose.pose.orientation.y,
        latest_uav_state_.pose.pose.orientation.z);
    if (q_W_B.norm() <= 1e-6) {
      ROS_WARN_THROTTLE(
          1.0,
          "target_estimate_cam: invalid UAV orientation, cannot compute fallback q_image.");
      return;
    }
    q_W_B.normalize();

    const Eigen::Vector3d p_W(
        latest_target_state_.pose.pose.position.x -
            latest_uav_state_.pose.pose.position.x,
        latest_target_state_.pose.pose.position.y -
            latest_uav_state_.pose.pose.position.y,
        latest_target_state_.pose.pose.position.z -
            latest_uav_state_.pose.pose.position.z);
    if (p_W.norm() <= 1e-6) {
      return;
    }

    const Eigen::Vector3d p_B = q_W_B.inverse() * p_W - p_B_C_;
    const Eigen::Vector3d ray_C = q_B_C_.inverse() * p_B;
    if (ray_C.norm() <= 1e-6) {
      return;
    }

    Eigen::Quaterniond q_image =
        Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(),
                                           ray_C.normalized());
    q_image.normalize();

    geometry_msgs::QuaternionStamped q_image_msg;
    q_image_msg.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
    q_image_msg.header.frame_id = latest_target_state_.child_frame_id.empty()
                                      ? image_frame_id_
                                      : latest_target_state_.child_frame_id;
    q_image_msg.quaternion.w = q_image.w();
    q_image_msg.quaternion.x = q_image.x();
    q_image_msg.quaternion.y = q_image.y();
    q_image_msg.quaternion.z = q_image.z();
    q_image_pub_.publish(q_image_msg);
  }

  ros::NodeHandle nh_;
  ros::Subscriber p_image_sub_;
  ros::Subscriber masspoint_sub_;
  ros::Subscriber uav_state_sub_;
  ros::Subscriber balloon_intercepted_sub_;
  ros::Publisher p_image_pub_;
  ros::Publisher q_image_pub_;
  ros::Publisher target_in_main_view_pub_;
  ros::Publisher target_state_pub_;
  ros::Publisher p_target_pub_;
  ros::Publisher fsm_intercepted_pub_;

  std::string p_image_input_topic_{"/tracker/bbox"};
  std::string masspoint_topic_{"/balloons/masspoint"};
  std::string p_image_topic_{"/nmpc/p_image"};
  std::string q_image_topic_{"/nmpc/q_image"};
  std::string target_in_main_view_topic_{"/nmpc/target_in_main_view"};
  std::string target_state_estimate_topic_{"/nmpc/targetStateEstimate"};
  std::string p_target_topic_{"/nmpc/p_target"};
  std::string uav_state_estimate_topic_{"/nmpc/uavStateEstimate"};
  std::string balloon_intercepted_topic_{"/balloons/intercepted"};
  std::string fsm_intercepted_topic_{"/fsm/intercepted"};
  std::string image_frame_id_{"base_link"};
  std::string target_frame_id_{"map"};

  double img_cx_{320.0};
  double img_cy_{240.0};
  double img_f_{115.0};
  double max_target_pixel_error_{80.0};
  int confirmation_frames_{3};
  double max_tracking_jump_px_{80.0};
  double max_tracking_jump_rate_px_s_{4000.0};
  double pending_match_px_{50.0};
  double lock_timeout_{0.5};
  Eigen::Vector3d p_B_C_{0.095, 0.0, 0.0};
  Eigen::Quaterniond q_B_C_{0.5, -0.5, 0.5, -0.5};
  nav_msgs::Odometry latest_uav_state_;
  nav_msgs::Odometry latest_target_state_;
  bool has_p_image_{false};
  bool publish_p_target_{true};
  bool has_target_pos_{true};
  bool has_target_{false};
  bool has_uav_state_{false};
  bool has_bearing_lock_{false};
  bool has_pending_candidate_{false};
  int pending_count_{0};
  double tracked_u_{0.0};
  double tracked_v_{0.0};
  double pending_u_{0.0};
  double pending_v_{0.0};
  ros::Time last_bearing_measurement_time_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_estimate_cam");
  ros::NodeHandle nh("~");
  TargetEstimateCamNode node(nh);
  ros::spin();
  return 0;
}
