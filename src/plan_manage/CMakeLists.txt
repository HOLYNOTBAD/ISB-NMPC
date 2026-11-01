cmake_minimum_required(VERSION 3.0.2)
project(plan_manage)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  quadrotor_common
  quadrotor_msgs
  roscpp
  roslib
  rospy
  std_msgs
  std_srvs
  swarm_msgs
  xmlrpcpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    mavros_msgs
    nav_msgs
    quadrotor_common
    quadrotor_msgs
    roscpp
    roslib
    rospy
    std_msgs
    std_srvs
    swarm_msgs
    xmlrpcpp
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_executable(fsm_node
  src/px4_com.cpp
  src/trajectory_fsm.cpp
  src/fsm_com.cpp
  src/interception_fsm.cpp
  src/fsm_node.cpp
)

add_dependencies(fsm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(fsm_node
  ${catkin_LIBRARIES}
  yaml-cpp
)

install(
  TARGETS fsm_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
