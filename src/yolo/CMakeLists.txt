cmake_minimum_required(VERSION 3.0.2)
project(yolo)

add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp std_msgs std_srvs sensor_msgs swarm_msgs camera_info_manager)

set(_cuda_search_paths)
if(DEFINED ENV{CUDACXX})
  get_filename_component(_cuda_from_cudacxx "$ENV{CUDACXX}" DIRECTORY)
  list(APPEND _cuda_search_paths "${_cuda_from_cudacxx}")
endif()
if(DEFINED ENV{CUDA_HOME})
  list(APPEND _cuda_search_paths "$ENV{CUDA_HOME}/bin")
endif()
if(DEFINED ENV{CUDA_PATH})
  list(APPEND _cuda_search_paths "$ENV{CUDA_PATH}/bin")
endif()
list(APPEND _cuda_search_paths /usr/local/cuda/bin /usr/local/cuda-12/bin /usr/local/cuda-11/bin)

find_program(CMAKE_CUDA_COMPILER
  NAMES nvcc
  HINTS ${_cuda_search_paths}
)

if(NOT CMAKE_CUDA_COMPILER)
  message(FATAL_ERROR
    "nvcc was not found. Install the CUDA toolkit and/or set CUDACXX, CUDA_HOME, or CUDA_PATH "
    "so the yolo package can be compiled."
  )
endif()

get_filename_component(CUDA_BIN_DIR "${CMAKE_CUDA_COMPILER}" DIRECTORY)
get_filename_component(CUDA_ROOT_DIR "${CUDA_BIN_DIR}" DIRECTORY)
enable_language(CUDA)

include_directories("${CUDA_ROOT_DIR}/include")
if(EXISTS "${CUDA_ROOT_DIR}/targets/aarch64-linux/include")
  include_directories("${CUDA_ROOT_DIR}/targets/aarch64-linux/include")
endif()
if(EXISTS "${CUDA_ROOT_DIR}/lib64")
  link_directories("${CUDA_ROOT_DIR}/lib64")
endif()

set(_system_opencv_candidates
  "/usr/lib/x86_64-linux-gnu/cmake/opencv4"
  "/usr/lib/aarch64-linux-gnu/cmake/opencv4"
  "/usr/lib/cmake/opencv4"
)
foreach(_system_opencv_dir ${_system_opencv_candidates})
  if(EXISTS "${_system_opencv_dir}/OpenCVConfig.cmake")
    set(OpenCV_DIR "${_system_opencv_dir}" CACHE PATH "Path to system OpenCVConfig.cmake" FORCE)
    break()
  endif()
endforeach()

find_package(OpenCV 4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yolo_detector
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs swarm_msgs
  DEPENDS OpenCV
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
  src
  src/backend
  src/src
  src/plugin
  
)

# Only inference-time sources belong in the ROS node. model.cpp and
# calibrator.cpp are offline engine-building tools and use a different
# TensorRT API lifecycle.
add_library(myplugins SHARED
  src/plugin/yololayer.cu
  src/src/preprocess.cu
  src/src/postprocess.cpp
)
target_link_libraries(myplugins nvinfer cudart)

set(YOLO_PLATFORM "AUTO" CACHE STRING "YOLO backend: AUTO, RTX5080, or ORIN_NX")
set_property(CACHE YOLO_PLATFORM PROPERTY STRINGS AUTO RTX5080 ORIN_NX)

if(YOLO_PLATFORM STREQUAL "AUTO")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(YOLO_PLATFORM "ORIN_NX")
  else()
    set(YOLO_PLATFORM "RTX5080")
  endif()
endif()

if(YOLO_PLATFORM STREQUAL "ORIN_NX")
  set(YOLO_BACKEND_SOURCE src/backend/tensorrt_orin_nx.cpp)
  set(YOLO_CUDA_ARCH 87)
elseif(YOLO_PLATFORM STREQUAL "RTX5080")
  set(YOLO_BACKEND_SOURCE src/backend/tensorrt_rtx5080.cpp)
  set(YOLO_CUDA_ARCH 120)
else()
  message(FATAL_ERROR "Unsupported YOLO_PLATFORM=${YOLO_PLATFORM}")
endif()

set_target_properties(myplugins PROPERTIES CUDA_ARCHITECTURES ${YOLO_CUDA_ARCH})
message(STATUS "YOLO platform: ${YOLO_PLATFORM}, CUDA architecture: sm_${YOLO_CUDA_ARCH}")

add_library(yolo_detector
  src/yolo_detector.cpp
  ${YOLO_BACKEND_SOURCE}
)
target_link_libraries(yolo_detector PRIVATE
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  nvinfer
  cudart
  myplugins
)

install(TARGETS yolo_detector myplugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


message(STATUS "library status:")
message(STATUS "OpenCV_VERSION: ${OpenCV_VERSION}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_DIR: ${OpenCV_DIR}")
message(STATUS "catkin libraries: ${catkin_LIBRARIES}")
message(STATUS "cv_bridge_DIR: ${cv_bridge_DIR}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
