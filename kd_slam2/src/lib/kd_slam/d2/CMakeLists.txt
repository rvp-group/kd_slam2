add_library(kd_slam_icp_2d
            SHARED
            tree.cpp
            descriptor.cpp
            cticp.cpp
            icp.cpp
            typedefs.cpp)
            
target_link_libraries(kd_slam_icp_2d ${srrg2_core_LIBRARIES})

add_library(kd_slam_scanner_2d SHARED scanner.cpp)

target_link_libraries(kd_slam_scanner_2d kd_slam_icp_2d)

add_library(kd_slam_slam_2d
            SHARED
            slam.cpp
            instances_cpu.cpp)

target_link_libraries(kd_slam_slam_2d
                      kd_slam_icp_2d
                      kd_slam_frame
                      kd_io
                      kd_slam_event
                      kd_slam_utils
                      ${srrg2_core_LIBRARIES}
                      ${srrg2_solver_LIBRARIES})


target_include_directories(kd_slam_icp_2d PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include/kd_slam2>
)

target_include_directories(kd_slam_scanner_2d PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include/kd_slam2>
)
target_include_directories(kd_slam_slam_2d PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include/kd_slam2>
)

if (HAVE_CUDA)
add_library(kd_slam_icp_2d_cuda
            SHARED
            tree.cu
            icp.cu
            cticp.cu
            descriptor.cu)
target_link_libraries(kd_slam_icp_2d_cuda kd_slam_icp_2d cudart)
set_target_properties(kd_slam_icp_2d_cuda PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_library(kd_slam_slam_2d_cuda
            SHARED
            instances_cuda.cpp)
target_link_libraries(kd_slam_slam_2d_cuda kd_slam_slam_2d kd_slam_icp_2d_cuda cudart)


target_include_directories(kd_slam_icp_2d_cuda PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include/kd_slam2>
)

target_include_directories(kd_slam_slam_2d_cuda PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include/kd_slam2>
)

endif()
