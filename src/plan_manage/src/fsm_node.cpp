#include <ros/ros.h>

#include "plan_manage/interception_fsm.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "fsm_node");
  ros::NodeHandle nh("~");

  std::string fsm_type = "interception";
  nh.param<std::string>("fsm/type", fsm_type, fsm_type);

  if (fsm_type == "interception") {
    plan_manage::InterceptionFSM fsm;
    fsm.init(nh);
    ros::spin();
    return 0;
  }

  ROS_FATAL("unsupported fsm/type '%s'. Expected 'interception'.",
            fsm_type.c_str());
  return 1;
}
