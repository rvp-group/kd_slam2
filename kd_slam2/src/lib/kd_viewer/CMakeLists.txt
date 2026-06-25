find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

set(SHADER_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/shaders")
add_compile_definitions("SHADERS_PATH=\"${SHADER_SOURCE_DIR}\"")


include_directories(imgui)


add_library(kd_viewer SHARED
  glad/glad.c
  vbo_base.cpp
  camera_pose_vbo.cpp
  cloud_vbo.cpp
  line_segments_vbo.cpp
  drawable_kd_cloud.cpp
  drawable_base.cpp
  drawable_kd_slam.cpp
  glfw_viewer.cpp
  kd_viewer.cpp
  imgui/imgui.cpp
  imgui/imgui_draw.cpp
  imgui/imgui_widgets.cpp
  imgui/examples/imgui_impl_glfw.cpp
  imgui/examples/imgui_impl_opengl3.cpp
)

target_compile_definitions(kd_viewer PRIVATE IMGUI_IMPL_OPENGL_LOADER_GLAD)

target_link_libraries(kd_viewer PUBLIC
  OpenGL::GL
  glfw
  dl
  kd_slam_event
)

target_include_directories(kd_viewer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/lib>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/common>
    $<INSTALL_INTERFACE:include/kd_slam2>)
