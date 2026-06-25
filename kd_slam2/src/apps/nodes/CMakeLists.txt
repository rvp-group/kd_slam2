set(SLAM_LIBS
  kd_viewer
  kd_slam_3d_library
  kd_slam_2d_library
)

add_executable(kd_slam kd_slam.cpp)
target_link_libraries(kd_slam ${SLAM_LIBS} )

add_executable(kd_localizer kd_localizer.cpp)
target_link_libraries(kd_localizer ${SLAM_LIBS} )

add_executable(kd_bundler kd_bundler.cpp)
target_link_libraries(kd_bundler ${SLAM_LIBS} )

add_executable(kd_converter kd_converter.cpp)
target_link_libraries(kd_converter ${SLAM_LIBS} )

add_executable(kd_map_replay kd_map_replay.cpp)
target_link_libraries(kd_map_replay ${SLAM_LIBS} )

add_executable(imu_replay imu_replay.cpp)
target_link_libraries(imu_replay kd_slam_utils)


add_executable(kd_livox_converter kd_livox_converter.cpp)
target_link_libraries(kd_livox_converter kd_io)
