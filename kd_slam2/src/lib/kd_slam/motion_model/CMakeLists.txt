add_library(kd_slam_motion_model SHARED pose_integrator.cpp)
target_include_directories(kd_slam_motion_model PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include/kd_slam2>
)
