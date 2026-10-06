cmake_minimum_required(VERSION 3.16)
project(kd_eval_tools LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)

# ---------- eval library (ported from vbr_bench) ----------
add_library(eval_lib
  src/trajectory.cpp
  src/eval.cpp
  src/file_utils.cpp
)
target_include_directories(eval_lib PUBLIC src ${EIGEN3_INCLUDE_DIR})
target_link_libraries(eval_lib Eigen3::Eigen)

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

# ---------- traj_intcompare ----------
add_executable(traj_intcompare src/traj_intcompare.cpp)
target_link_libraries(traj_intcompare eval_lib)

# kitti2ros2bag and odom2tum (ROS) live in kd_slam2_ros
# same install location as before: the scripts call lib/kd_eval_tools/<tool>
install(TARGETS kitti_gt2tum kitti_tum2poses traj_compare traj_intcompare
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
