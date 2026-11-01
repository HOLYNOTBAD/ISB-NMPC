cmake_minimum_required(VERSION 3.0.2)
project(interception_baseline)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  quadrotor_msgs
)

catkin_python_setup()

catkin_package()

#############
#  Scripts  #
#############

# Python modules are exported through setup.py/catkin_python_setup(); the
# executable bridge node is installed below.
catkin_install_python(PROGRAMS
  scripts/interception_baseline_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
#  Launch #
###########

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
