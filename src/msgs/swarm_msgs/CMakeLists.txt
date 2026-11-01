cmake_minimum_required(VERSION 2.8...3.27)
project(swarm_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Action.msg
  Pipeunit.msg
  Pipeline.msg
  PiecewiseBezier.msg
  TimeOptimalPMMParam.msg
  TimeOptimalPMMPieces.msg
  Waypoint.msg
  BoundingBox.msg
  BoundingBoxes.msg
  TimedBoundingBox.msg
  BoundingBoxSequence.msg
  ObjectCount.msg
  RflyObject.msg
  Index.msg
  DoorBox.msg
  DoorBoxes.msg
  IdList.msg
  SwarmInfo.msg
  Value.msg
  EntrancePose.msg
  FixedPoint.msg
  CenterPoints.msg
  InterceptionEvent.msg
  MassPoint.msg
  MassPoints.msg
)


## Generate actions in the 'action' folder
add_action_files(
  FILES
  CheckForObjects.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
