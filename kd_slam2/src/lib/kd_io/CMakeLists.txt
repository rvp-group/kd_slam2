add_library(kd_io SHARED scan_utils.cpp kd_tum_writer.cpp kd_state_writer.cpp kd_state_reader.cpp tum_io.cpp tree_loader_2d.cpp tree_loader_3d.cpp)
target_link_libraries(kd_io lz4 zstd kd_slam_icp_2d kd_slam_icp_3d kd_slam_event kd_slam_frame kd_slam_utils)

find_package(rosbag2_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(kd_slam2_msgs REQUIRED)
target_sources(kd_io PRIVATE rosbag2_message_reader.cpp rosbag2_message_writer.cpp reader_factory.cpp)
target_link_libraries(kd_io rosbag2_cpp::rosbag2_cpp rclcpp::rclcpp)
ament_target_dependencies(kd_io sensor_msgs tf2 tf2_msgs kd_slam2_msgs)
target_include_directories(kd_io PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
    $<INSTALL_INTERFACE:include/kd_slam2>)
