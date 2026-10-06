# rosbag2 reader/writer for kd_io; registers Rosbag2MessageReader/Writer when loaded (-dl) or linked
add_library(kd_io_rosbag SHARED rosbag2_message_reader.cpp rosbag2_message_writer.cpp)
target_include_directories(kd_io_rosbag PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/kd_slam2_ros>)
target_link_libraries(kd_io_rosbag kd_slam2::kd_io rosbag2_cpp::rosbag2_cpp rclcpp::rclcpp)
ament_target_dependencies(kd_io_rosbag sensor_msgs nav_msgs tf2_msgs kd_slam2_msgs)

# bag converters (formerly in kd_eval_tools)
add_executable(kitti2ros2bag kitti2ros2bag.cpp)
ament_target_dependencies(kitti2ros2bag rclcpp rosbag2_cpp rosbag2_storage sensor_msgs)

add_executable(odom2tum odom2tum.cpp)
ament_target_dependencies(odom2tum rclcpp nav_msgs)

install(TARGETS kd_io_rosbag
  EXPORT kd_slam2_rosTargets
  LIBRARY DESTINATION lib)
install(FILES rosbag2_message_reader.h rosbag2_message_writer.h
  DESTINATION include/kd_slam2_ros)
install(TARGETS kitti2ros2bag odom2tum
  RUNTIME DESTINATION lib/${PROJECT_NAME})
