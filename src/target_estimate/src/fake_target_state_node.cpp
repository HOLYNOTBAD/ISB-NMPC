#include <cstdint>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <swarm_msgs/MassPoint.h>
#include <swarm_msgs/MassPoints.h>

namespace {

class FakeTargetStateNode {
 public:
  FakeTargetStateNode() : pnh_("~") {
    pnh_.param<std::string>("truth_topic", truth_topic_,
                            "/balloons/masspoint");
    pnh_.param<std::string>("output_topic", output_topic_,
                            "/nmpc/targetStateEstimate");
    pnh_.param<std::string>("frame_id", frame_id_, "world");
    pnh_.param("target_id", target_id_, -1);

    target_pub_ = nh_.advertise<nav_msgs::Odometry>(output_topic_, 10);
    truth_sub_ = nh_.subscribe(truth_topic_, 10,
                               &FakeTargetStateNode::truthCallback, this);

    ROS_INFO("fake_target_state: %s -> %s (target_id=%ld)",
             truth_topic_.c_str(), output_topic_.c_str(),
             static_cast<long>(target_id_));
  }

 private:
  void truthCallback(const swarm_msgs::MassPoints::ConstPtr& msg) {
    if (msg->points.empty()) {
      ROS_WARN_THROTTLE(1.0, "fake_target_state: truth message is empty.");
      return;
    }

    const swarm_msgs::MassPoint* selected = nullptr;
    if (target_id_ < 0) {
      selected = &msg->points.front();
    } else {
      for (const auto& point : msg->points) {
        if (point.volume == target_id_) {
          selected = &point;
          break;
        }
      }
    }

    if (selected == nullptr) {
      ROS_WARN_THROTTLE(1.0,
                        "fake_target_state: target_id=%ld not found in truth.",
                        static_cast<long>(target_id_));
      return;
    }

    nav_msgs::Odometry target;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = frame_id_;
    target.child_frame_id = "target_" + std::to_string(selected->volume);
    target.pose.pose.position.x = selected->position.x;
    target.pose.pose.position.y = selected->position.y;
    target.pose.pose.position.z = selected->position.z;
    target.pose.pose.orientation.w = 1.0;
    target.twist.twist.linear.x = selected->velocity.x;
    target.twist.twist.linear.y = selected->velocity.y;
    target.twist.twist.linear.z = selected->velocity.z;
    target_pub_.publish(target);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber truth_sub_;
  ros::Publisher target_pub_;
  std::string truth_topic_;
  std::string output_topic_;
  std::string frame_id_;
  int target_id_{-1};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "fake_target_state");
  FakeTargetStateNode node;
  ros::spin();
  return 0;
}
