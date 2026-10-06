cmake_minimum_required(VERSION 3.24)
option(HAVE_CUDA "Build with cuda support" ON)
option(BUILD_TESTS "Build test executables" OFF)

project(kd_slam2 VERSION 2.0 LANGUAGES C CXX)

if (HAVE_CUDA)
  find_package(CUDAToolkit QUIET)
  if (NOT CUDAToolkit_FOUND)
    message(STATUS "kd_slam2: CUDA toolkit not found, disabling CUDA support")
    set(HAVE_CUDA OFF CACHE BOOL "" FORCE)
  else()
    enable_language(CUDA)
  endif()
endif()

# compile options (formerly from srrg_cmake_modules)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wall -Warray-bounds=0 -Wno-overloaded-virtual -Wno-unused-parameter")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99 -fPIC -Wall -Warray-bounds=0 -Wno-unused-parameter")
if (HAVE_CUDA)
  add_definitions(-DSRRG_USE_CUDA)
  find_file(CUDA_HEADER cuda_runtime.h PATHS ${CUDAToolkit_INCLUDE_DIRS} REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -include ${CUDA_HEADER}")
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr -diag-suppress=20011 -diag-suppress=20014 -diag-suppress=997 -Wno-unused-parameter")
else()
  add_definitions(-D__host__=/**/ -D__device__=/**/)
endif()

find_package(Eigen3 REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(srrg2_core REQUIRED)
find_package(srrg2_solver REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${srrg2_core_INCLUDE_DIRS}
  ${srrg2_solver_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/lib
)

add_subdirectory(src)

set(KD_SLAM2_LIBRARIES
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
)
if (HAVE_CUDA)
  list(APPEND KD_SLAM2_LIBRARIES
    kd_slam_cuda_init
    kd_slam_icp_2d_cuda
    kd_slam_slam_2d_cuda
    kd_slam_icp_3d_cuda
    kd_slam_slam_3d_cuda
  )
endif()

set(KD_SLAM2_APPS
  kd_slam
  kd_bundler
  kd_converter
  kd_translator
  kd_map_replay
  imu_filter_test
)

# ---- install ----
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS ${KD_SLAM2_LIBRARIES}
  EXPORT  kd_slam2Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(TARGETS ${KD_SLAM2_APPS}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY src/lib/
  DESTINATION include/kd_slam2
  FILES_MATCHING PATTERN "*.h" PATTERN "*.cuh")

install(EXPORT kd_slam2Targets
  NAMESPACE   kd_slam2::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kd_slam2)

list(TRANSFORM KD_SLAM2_LIBRARIES PREPEND "kd_slam2::" OUTPUT_VARIABLE PACKAGE_EXPORTED_LIBRARIES)
string(JOIN " " PACKAGE_EXPORTED_LIBRARIES ${PACKAGE_EXPORTED_LIBRARIES})

configure_package_config_file(cmake/kd_slam2Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/kd_slam2Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kd_slam2)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/kd_slam2ConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/kd_slam2Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/kd_slam2ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kd_slam2)
