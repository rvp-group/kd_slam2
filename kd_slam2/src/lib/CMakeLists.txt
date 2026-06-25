add_subdirectory(kd_io)
add_subdirectory(kd_slam)
add_subdirectory(kd_viewer)

add_library(kd_slam_2d_library SHARED
  instances_2d.cpp
)

set_target_properties(kd_slam_2d_library PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

target_link_libraries(kd_slam_2d_library PUBLIC
  ${srrg2_core_LIBRARIES}
  ${srrg2_solver_LIBRARIES}
  kd_slam_icp_2d
  kd_slam_slam_2d
  kd_io
  kd_slam_utils
)

target_include_directories(kd_slam_2d_library PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include/kd_slam2>
)

add_library(kd_slam_3d_library SHARED
  instances_3d.cpp
)

set_target_properties(kd_slam_3d_library PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

target_link_libraries(kd_slam_3d_library PUBLIC
  ${srrg2_core_LIBRARIES}
  ${srrg2_solver_LIBRARIES}
  kd_slam_icp_3d
  kd_slam_slam_3d
  kd_io
  kd_slam_utils
)

target_include_directories(kd_slam_3d_library PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include/kd_slam2>
)
