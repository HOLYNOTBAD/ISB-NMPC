cmake_minimum_required(VERSION 3.0.2)
project(target_estimate)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  swarm_msgs
)

catkin_package()

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(target_estimate_cam
  src/target_estimate_cam_node.cpp
)

add_executable(fake_target_state
  src/fake_target_state_node.cpp
)

add_dependencies(target_estimate_cam
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(fake_target_state
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(target_estimate_cam
  ${catkin_LIBRARIES}
)

target_link_libraries(fake_target_state
  ${catkin_LIBRARIES}
)

install(
  TARGETS target_estimate_cam fake_target_state
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
