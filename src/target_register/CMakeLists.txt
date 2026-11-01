cmake_minimum_required(VERSION 3.0.2)
project(target_register)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_BUILD_TYPE "RELEASE")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(WORKSPACE_ROOT ${CMAKE_SOURCE_DIR}/..)

if(EXISTS ${WORKSPACE_ROOT}/third_party/Eigen/Eigen)
  set(EIGEN3_INCLUDE_DIRS ${WORKSPACE_ROOT}/third_party/Eigen)
else()
  find_package(Eigen3 REQUIRED)
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  mavros_msgs
  quadrotor_msgs
  roscpp
  roslib
  std_msgs
  std_srvs
  swarm_msgs
  tf
  visualization_msgs
)
find_package(yaml-cpp)

catkin_package(
  CATKIN_DEPENDS quadrotor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_executable(gazebo_balloon_node
  src/gazebo_balloon_motion.cpp)
add_dependencies(gazebo_balloon_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_balloon_node
  ${catkin_LIBRARIES}
  yaml-cpp
)

install(TARGETS gazebo_balloon_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
