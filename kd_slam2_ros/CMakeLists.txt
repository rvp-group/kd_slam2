cmake_minimum_required(VERSION 3.16)
project(kd_slam2_ros LANGUAGES C CXX)

# compile options as kd_ouster (no cuda: kd_io headers are host code)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wall -Warray-bounds=0 -Wno-overloaded-virtual -Wno-unused-parameter")
add_definitions(-D__host__=/**/ -D__device__=/**/)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(srrg2_core REQUIRED)
find_package(srrg2_solver REQUIRED)
find_package(kd_slam2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(kd_slam2_msgs REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${srrg2_core_INCLUDE_DIRS}
  ${srrg2_solver_INCLUDE_DIRS}
)

add_subdirectory(src)

ament_export_targets(kd_slam2_rosTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(kd_slam2 kd_slam2_msgs rclcpp rosbag2_cpp sensor_msgs nav_msgs tf2_msgs)
ament_package()
