cmake_minimum_required(VERSION 3.24)
project(kd_ouster VERSION 1.0 LANGUAGES C CXX)

# compile options (formerly from srrg_cmake_modules, no cuda)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wall -Warray-bounds=0 -Wno-overloaded-virtual -Wno-unused-parameter")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99 -fPIC -Wall -Warray-bounds=0 -Wno-unused-parameter")
add_definitions(-D__host__=/**/ -D__device__=/**/)

find_package(Eigen3 REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(srrg2_core REQUIRED)
find_package(srrg2_solver REQUIRED)
find_package(kd_slam2 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${srrg2_core_INCLUDE_DIRS}
  ${srrg2_solver_INCLUDE_DIRS}
  ${kd_slam2_INCLUDE_DIRS}
  src
)

add_subdirectory(src)

target_include_directories(tiny_ouster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/kd_ouster>)

# ---- install ----
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS tiny_ouster
  EXPORT  kd_ousterTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(TARGETS kd_live ouread ouserver
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY src/
  DESTINATION include/kd_ouster
  FILES_MATCHING PATTERN "*.h")

install(EXPORT kd_ousterTargets
  NAMESPACE   kd_ouster::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kd_ouster)

configure_package_config_file(cmake/kd_ousterConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/kd_ousterConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kd_ouster)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/kd_ousterConfig.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kd_ouster)
