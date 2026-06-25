cmake_minimum_required(VERSION 3.8)
project(kd_eval_tools LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)

# ---------- eval library (ported from vbr_bench) ----------
add_library(eval_lib
  src/trajectory.cpp
  src/eval.cpp
  src/file_utils.cpp
)
target_include_directories(eval_lib PUBLIC src ${EIGEN3_INCLUDE_DIR})
target_link_libraries(eval_lib Eigen3::Eigen)

# ---------- kitti2ros2bag ----------
add_executable(kitti2ros2bag src/kitti2ros2bag.cpp)
ament_target_dependencies(kitti2ros2bag rclcpp rosbag2_cpp rosbag2_storage sensor_msgs)

# ---------- kitti_gt2tum ----------
add_executable(kitti_gt2tum src/kitti_gt2tum.cpp)
target_include_directories(kitti_gt2tum PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(kitti_gt2tum Eigen3::Eigen)

# ---------- kitti_tum2poses ----------
add_executable(kitti_tum2poses src/kitti_tum2poses.cpp)
target_include_directories(kitti_tum2poses PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(kitti_tum2poses Eigen3::Eigen)

# ---------- traj_compare ----------
add_executable(traj_compare src/traj_compare.cpp)
target_link_libraries(traj_compare eval_lib)

install(TARGETS kitti2ros2bag kitti_gt2tum kitti_tum2poses traj_compare
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
