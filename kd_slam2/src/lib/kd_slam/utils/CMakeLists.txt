add_library(kd_slam_utils SHARED runnable.cpp imu_filter.cpp imu_filter_setup.cpp)

add_executable(imu_filter_test imu_filter_test.cpp imu_log.cpp)
target_link_libraries(imu_filter_test kd_slam_utils)
