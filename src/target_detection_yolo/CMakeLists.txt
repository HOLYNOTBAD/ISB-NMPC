cmake_minimum_required(VERSION 3.0.2)
project(target_detection_yolo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  swarm_msgs
  yolo
)
find_package(OpenCV 4.2 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES detection_fsm
  CATKIN_DEPENDS cv_bridge image_transport nav_msgs roscpp sensor_msgs std_msgs std_srvs swarm_msgs yolo
  DEPENDS OpenCV
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(detection_fsm
  src/detection_fsm.cpp
  src/reacquire_hr.cpp
  src/track_hr.cpp
  src/track_lr.cpp
)
target_link_libraries(detection_fsm ${OpenCV_LIBS})

add_executable(target_detection_yolo_node
  src/target_detection_yolo_node.cpp
)
add_dependencies(target_detection_yolo_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(target_detection_yolo_node
  detection_fsm
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS detection_fsm target_detection_yolo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
