add_library(tiny_ouster
  json_item.cpp
  json_tokenizer.cpp
  json_parser.cpp
  tcp_socket.cpp
  tiny_http.cpp
  ouster_metadata.cpp
  ouster_lidar.cpp
  legacy_profile.cpp
  rng19_profile.cpp
  udp_socket.cpp
  dbuffer.cpp
)

add_executable(ouserver ouserver.cpp )
target_link_libraries(ouserver tiny_ouster)

add_executable(ouread ouread.cpp)
target_link_libraries(ouread tiny_ouster)

set(SLAM_LIBS
  ${srrg2_core_LIBRARIES}
  ${srrg2_solver_LIBRARIES}
  -Wl,--no-as-needed
  ${kd_slam2_LIBRARIES}
)


add_executable(kd_live
  kd_live_logger.cpp
  kd_live_ouster.cpp
  kd_live_running_context.cpp
  kd_live_web_server.cpp
  kd_live.cpp)
  
target_link_libraries(kd_live ${SLAM_LIBS} tiny_ouster)
