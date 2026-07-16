add_library(kd_slam_icp_3d
            SHARED
            tree.cpp
            descriptor.cpp
            cticp.cpp
            icp.cpp
            typedefs.cpp)
            
target_link_libraries(kd_slam_icp_3d ${srrg2_core_LIBRARIES})

add_library(kd_slam_scanner_3d SHARED scanner.cpp)

target_link_libraries(kd_slam_scanner_3d kd_slam_icp_3d)

add_library(kd_slam_slam_3d
            SHARED
            slam.cpp
            slam_gravity_prior_factor.cpp
            instances_cpu.cpp)

target_link_libraries(kd_slam_slam_3d
                      kd_slam_icp_3d
                      kd_slam_frame
                      kd_io
                      kd_slam_event
                      kd_slam_utils
                      kd_slam_motion_model
                      ${srrg2_core_LIBRARIES}
                      ${srrg2_solver_LIBRARIES})


target_include_directories(kd_slam_icp_3d PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include/kd_slam2>
)

target_include_directories(kd_slam_scanner_3d PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include/kd_slam2>
)
target_include_directories(kd_slam_slam_3d PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include/kd_slam2>
)

if (HAVE_CUDA)
add_library(kd_slam_icp_3d_cuda
            SHARED
            tree.cu
            icp.cu
            cticp.cu
            descriptor.cu)

target_link_libraries(kd_slam_icp_3d_cuda kd_slam_icp_3d cudart)
set_target_properties(kd_slam_icp_3d_cuda PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

add_library(kd_slam_slam_3d_cuda
            SHARED
            instances_cuda.cpp)
target_link_libraries(kd_slam_slam_3d_cuda kd_slam_slam_3d kd_slam_icp_3d_cuda cudart)


target_include_directories(kd_slam_icp_3d_cuda PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include/kd_slam2>
)

target_include_directories(kd_slam_slam_3d_cuda PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
  $<INSTALL_INTERFACE:include/kd_slam2>
)

endif()
