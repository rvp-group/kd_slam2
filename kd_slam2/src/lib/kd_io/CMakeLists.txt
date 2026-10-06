add_library(kd_io SHARED
  message_queue.cpp scan_utils.cpp
  static_tf_buffer.cpp
  kd_tum_writer.cpp
  kd_state_writer.cpp
  kd_state_reader.cpp
  tum_io.cpp
  tree_loader_2d.cpp
  tree_loader_3d.cpp
  queue_message_reader.cpp
  slam_message_data.cpp
  slam_messages.cpp
  reader_factory.cpp
  boss_message_reader.cpp boss_message_writer.cpp
  )

target_link_libraries(kd_io lz4 zstd kd_slam_icp_2d kd_slam_icp_3d kd_slam_event kd_slam_frame kd_slam_utils)

target_include_directories(kd_io PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
    $<INSTALL_INTERFACE:include/kd_slam2>)

