cmake_minimum_required(VERSION 3.24)
option(HAVE_CUDA "Build with cuda support"  ON)

if (HAVE_CUDA)
project(kd_slam2 LANGUAGES C CXX CUDA)
set(SRRG_ENABLE_CUDA ON)
else()
project(kd_slam2 LANGUAGES C CXX)
set(SRRG_ENABLE_CUDA OFF)
endif()

find_package(ament_cmake REQUIRED)
find_package(srrg_cmake_modules REQUIRED)
include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(srrg2_core REQUIRED)
find_package(srrg2_solver REQUIRED)
find_package(kd_slam2_msgs REQUIRED)

set(BUILD_TESTS off)

set(OPT_CUDA_LIBRRIES "")
if (HAVE_CUDA) 
   add_compile_definitions(HAVE_CUDA)
   link_libraries(cudart)
endif()

message("${PROJECT_NAME} | CXX flags: ${CMAKE_CXX_FLAGS}")

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${srrg2_core_INCLUDE_DIRS}
  ${srrg2_solver_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/lib
  ${PROJECT_SOURCE_DIR}/src/lib/kd_slam
)

add_subdirectory(src)

ament_export_include_directories(src/lib src/common src/io)
ament_export_libraries(
  kd_slam_2d_library
  kd_slam_3d_library
  kd_io
  kd_slam_icp_2d
  kd_slam_scanner_2d
  kd_slam_slam_2d
  kd_slam_icp_3d
  kd_slam_scanner_3d
  kd_slam_slam_3d
  kd_slam_frame
  kd_slam_event
  kd_slam_utils
  kd_viewer
)

ament_package()


install(
  DIRECTORY src/lib     
  DESTINATION .)

install(
  TARGETS
  kd_slam_2d_library
  kd_slam_3d_library
  kd_io
  kd_slam_icp_2d
  kd_slam_scanner_2d
  kd_slam_slam_2d
  kd_slam_icp_3d
  kd_slam_scanner_3d
  kd_slam_slam_3d
  kd_slam_frame
  kd_slam_event
  kd_slam_utils
  kd_slam_motion_model
  kd_viewer
  kd_slam
  kd_bundler
  kd_localizer
  kd_converter
  kd_livox_converter
  kd_map_replay
  imu_replay
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION kd_slam2/
  )
