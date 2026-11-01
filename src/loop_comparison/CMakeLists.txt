cmake_minimum_required(VERSION 3.0.2)
project(loop_comparison)
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs geometry_msgs nav_msgs quadrotor_msgs rospy std_msgs std_srvs
  swarm_msgs visualization_msgs
)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS
  scripts/loop_comparison_node.py
  scripts/prepare_comparison_motion.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
