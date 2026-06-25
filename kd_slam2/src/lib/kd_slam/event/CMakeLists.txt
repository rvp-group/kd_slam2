add_library(kd_slam_event SHARED
            event_queue.cpp
            event_dispatcher.cpp
)
target_include_directories(kd_slam_event PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include/kd_slam2>
)
