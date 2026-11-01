cmake_minimum_required(VERSION 3.0.2)
project(target_detection_hsv)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  swarm_msgs
)
find_package(OpenCV 4.2 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs swarm_msgs
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(target_detection_hsv_node src/color_det.cpp)
add_dependencies(target_detection_hsv_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(target_detection_hsv_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS target_detection_hsv_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
