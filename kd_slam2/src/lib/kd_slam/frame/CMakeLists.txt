add_library(kd_slam_frame SHARED
            frame_base.cpp
            factor_base.cpp
            frame_queue_base.cpp
            frame_queue_sorting.cpp
            frame_graph.cpp)
target_include_directories(kd_slam_frame PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include/kd_slam2>
)
