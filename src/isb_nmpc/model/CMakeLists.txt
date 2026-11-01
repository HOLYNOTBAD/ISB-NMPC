cmake_minimum_required(VERSION 2.8)
project(so3_quadrotor_model)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

if(CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

find_package(ACADO REQUIRED)

include_directories(. ${ACADO_INCLUDE_DIRS})

add_executable(so3_quadrotor_model_codegen so3_quadrotor_model.cpp)
target_link_libraries(so3_quadrotor_model_codegen ${ACADO_SHARED_LIBRARIES})
set_target_properties(
  so3_quadrotor_model_codegen
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
