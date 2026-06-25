cmake_minimum_required(VERSION 3.8)
project(kd_slam2_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KDNode2f.msg"
  "msg/KDNode3f.msg"
  "msg/KDTree2f.msg"
  "msg/KDTree3f.msg"
  "msg/KDDescriptor.msg"
  DEPENDENCIES std_msgs nav_msgs tf2
)

ament_export_dependencies(rosidl_default_runtime std_msgs nav_msgs tf2)
ament_package()
