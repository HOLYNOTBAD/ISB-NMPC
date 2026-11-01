cmake_minimum_required(VERSION 3.0.2)
project(interception_visualization)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/baylands_mesh_marker.py
  scripts/camera_frustum_marker.py
  scripts/fpv_los_overlay.py
  scripts/uav_mesh_marker.py
  scripts/world_map_alignment_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz rqt_plot
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
