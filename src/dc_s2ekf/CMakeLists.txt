cmake_minimum_required(VERSION 3.0.2)
project(dc_s2ekf)

add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_srvs
  swarm_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES perception_filter_core
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_srvs swarm_msgs
)

catkin_install_python(PROGRAMS
  scripts/evaluate_delay_comp_truth_live.py
  scripts/filter_eval_rqt_plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(perception_filter_core
  src/filter/camera_model.cpp
  src/filter/log_size_filter.cpp
  src/filter/s2_filter.cpp
)
add_dependencies(perception_filter_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_filter_core ${catkin_LIBRARIES})

add_executable(bbox_s2_delay_ekf_node
  src/node/bbox_filter_node.cpp
)
add_dependencies(bbox_s2_delay_ekf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bbox_s2_delay_ekf_node perception_filter_core ${catkin_LIBRARIES})

install(TARGETS perception_filter_core bbox_s2_delay_ekf_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
