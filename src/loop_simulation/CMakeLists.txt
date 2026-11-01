cmake_minimum_required(VERSION 3.0.2)
project(loop_simulation)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  quadrotor_msgs
  rospy
  std_msgs
  std_srvs
  swarm_msgs
)

catkin_package(
  CATKIN_DEPENDS gazebo_msgs quadrotor_msgs rospy std_msgs std_srvs swarm_msgs
)

catkin_install_python(PROGRAMS
  scripts/loop_simulation_node.py
  scripts/summarize_trials.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY src/loop_simulation
  DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
)
