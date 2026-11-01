cmake_minimum_required(VERSION 3.0.2)
project(isb_nmpc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  mavros_msgs
  nav_msgs
  quadrotor_common
  quadrotor_msgs
  roscpp
  sensor_msgs
  std_msgs
  swarm_msgs
  trajectory_msgs
)

set(WORKSPACE_ROOT ${CMAKE_SOURCE_DIR}/..)
if(EXISTS ${WORKSPACE_ROOT}/third_party/Eigen/Eigen)
  set(EIGEN3_INCLUDE_DIRS ${WORKSPACE_ROOT}/third_party/Eigen)
else()
  find_package(Eigen3 REQUIRED)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

catkin_package(
  INCLUDE_DIRS include model/so3_quadrotor_mpc_codegen externals/qpoases/INCLUDE
  LIBRARIES so3_qpoases so3_mpc_solver so3_mpc_wrapper so3_mpc_controller
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    mavros_msgs
    nav_msgs
    quadrotor_common
    quadrotor_msgs
    roscpp
    sensor_msgs
    std_msgs
    swarm_msgs
    trajectory_msgs
)

include_directories(
  include
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC
  model/so3_quadrotor_mpc_codegen
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(so3_qpoases
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
)

target_include_directories(so3_qpoases PUBLIC
  include
  model/so3_quadrotor_mpc_codegen
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC
  ${catkin_INCLUDE_DIRS}
)

target_compile_definitions(so3_qpoases PRIVATE
  QPOASES_CUSTOM_INTERFACE=acado_qpoases_interface.hpp
)

target_link_libraries(so3_qpoases
  ${catkin_LIBRARIES}
)

add_library(so3_mpc_solver
  model/so3_quadrotor_mpc_codegen/acado_qpoases_interface.cpp
  model/so3_quadrotor_mpc_codegen/acado_integrator.c
  model/so3_quadrotor_mpc_codegen/acado_solver.c
  model/so3_quadrotor_mpc_codegen/acado_auxiliary_functions.c
)

target_include_directories(so3_mpc_solver PUBLIC
  model/so3_quadrotor_mpc_codegen
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC
)

target_link_libraries(so3_mpc_solver
  so3_qpoases
  ${catkin_LIBRARIES}
)

add_library(so3_mpc_wrapper
  src/so3_mpc_wrapper.cpp
)

target_link_libraries(so3_mpc_wrapper
  so3_mpc_solver
  so3_qpoases
  ${catkin_LIBRARIES}
)

add_library(so3_mpc_controller
  src/so3_mpc_controller.cpp
)

target_link_libraries(so3_mpc_controller
  so3_mpc_wrapper
  so3_qpoases
  ${catkin_LIBRARIES}
)

add_executable(so3_mpc_bridge_node
  src/so3_mpc_bridge_node.cpp
)
add_dependencies(so3_mpc_bridge_node ${catkin_EXPORTED_TARGETS})
set_target_properties(so3_mpc_bridge_node PROPERTIES LINK_FLAGS "-rdynamic")

target_link_libraries(so3_mpc_bridge_node
  so3_mpc_controller
  so3_mpc_wrapper
  so3_qpoases
  ${catkin_LIBRARIES}
)

install(TARGETS
  so3_qpoases
  so3_mpc_solver
  so3_mpc_wrapper
  so3_mpc_controller
  so3_mpc_bridge_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
