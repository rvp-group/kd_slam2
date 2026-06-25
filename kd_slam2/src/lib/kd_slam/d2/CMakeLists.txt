set(KD_ICP_SOURCES typedefs.cpp icp.cpp tree.cpp  descriptor.cpp cticp.cpp)
set(KD_SLAM_SOURCES slam.cpp instances_cpu.cpp)
if (HAVE_CUDA)
   list(APPEND KD_ICP_SOURCES  tree.cu icp.cu cticp.cu descriptor.cu)
   list(APPEND KD_SLAM_SOURCES  instances_cuda.cpp)
endif()

add_library(kd_slam_icp_2d SHARED ${KD_ICP_SOURCES})

if (HAVE_CUDA)
   set_target_properties(kd_slam_icp_2d PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
endif()

target_link_libraries(kd_slam_icp_2d ${srrg2_core_LIBRARIES})

add_library(kd_slam_scanner_2d SHARED scanner.cpp)
target_link_libraries(kd_slam_scanner_2d kd_slam_icp_2d)

add_library(kd_slam_slam_2d SHARED ${KD_SLAM_SOURCES})

target_link_libraries(kd_slam_slam_2d kd_slam_icp_2d kd_slam_frame  kd_io kd_slam_event kd_slam_utils ${srrg2_core_LIBRARIES} ${srrg2_solver_LIBRARIES})

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
