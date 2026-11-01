cmake_minimum_required(VERSION 3.0.2)
project(camera_driver)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  image_transport
  roscpp
  sensor_msgs
)

find_library(TURBOJPEG_LIBRARY NAMES turbojpeg REQUIRED)

catkin_package(
  CATKIN_DEPENDS camera_info_manager image_transport roscpp sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(parallel_mjpeg_camera_node
  src/parallel_mjpeg_camera_node.cpp
)

target_link_libraries(parallel_mjpeg_camera_node
  ${catkin_LIBRARIES}
  ${TURBOJPEG_LIBRARY}
  pthread
)

target_compile_options(parallel_mjpeg_camera_node PRIVATE -O3)

install(TARGETS parallel_mjpeg_camera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
