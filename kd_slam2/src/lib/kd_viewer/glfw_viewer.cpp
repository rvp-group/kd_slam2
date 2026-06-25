#include "glfw_viewer.h"
#include <iostream>
#include <cmath>
#include "imgui/imgui.h"
#include "imgui/examples/imgui_impl_glfw.h"
#include "imgui/examples/imgui_impl_opengl3.h"

namespace kd_slam {

  using namespace std;

  Eigen::Matrix4f GLFWViewer::computeGLPerspectiveMatrix(float fov,
                                                         float width,
                                                         float height,
                                                         float near,
                                                         float far) {
    Eigen::Matrix4f P = Eigen::Matrix4f::Zero();
    float itfov  = 1.f / tanf(fov / 2.f);
    float aspect = height / width;
    P(0,0) = aspect * itfov;
    P(1,1) = itfov;
    P(2,2) = -(far + near) / (far - near);
    P(2,3) = -(2.f * far * near) / (far - near);
    P(3,2) = -1.f;
    return P;
  }

  static inline Eigen::Isometry3f getGLConstCameraTransform() {
    Eigen::Isometry3f iso;
    iso.matrix().setIdentity();
    iso.linear() <<
       0, 0,-1,
      -1, 0, 0,
       0, 1, 0;
    return iso;
  }

  const Eigen::Isometry3f GLFWViewer::gl_camera_transform = getGLConstCameraTransform();

  // gl_camera_transform maps camera_pose local axes to GL view axes:
  //   camera_pose +X  ->  screen down    (GL -Y)
  //   camera_pose +Y  ->  into screen    (GL  back, i.e. forward)
  //   camera_pose +Z  ->  screen left    (GL -X)
  //
  // So in camera_pose local frame:
  //   forward (into screen)  = -Y
  //   screen right           = -Z
  //   screen up              = -X
  //   yaw   = rotate around  camera_pose X
  //   pitch = rotate around  camera_pose Z
  //   roll  = rotate around  camera_pose Y

  GLFWViewer::GLFWViewer() : light_direction(1.f, 0.f, 0.f) {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "kd-slam viewer", NULL, NULL);
    if (!window) {
      cerr << "Failed to create GLFW window\n";
      glfwTerminate();
      exit(-1);
    }
    glfwSetWindowUserPointer(window, this);
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, key_callback);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
      cerr << "Failed to initialize GLAD\n";
      exit(-1);
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui_ImplGlfw_InitForOpenGL(window, false); // don't override our callbacks
    ImGui_ImplOpenGL3_Init("#version 330");


    camera_pose.setIdentity();
    camera_pose.translation() << 0.f, 0.f, 30.f;
  }

  void GLFWViewer::loop() {
    while (!glfwWindowShouldClose(window)) {
      bool retval = false;
      if (loop_callback) {
        retval=loop_callback();
        if (!retval) {
          loop_callback=nullptr;
        }
      }
      glClearColor(_palette_world.bg_color[0],
                   _palette_world.bg_color[1],
                   _palette_world.bg_color[2],
                   _palette_world.bg_color[3]);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      glEnable(GL_DEPTH_TEST);

      _drawScene();
      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();
      if (_draw_hud)
        _drawHUD(_getHUD());
      if (_draw_log)
        _drawLog(_getLog());
      ImGui::Render();
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      glfwSwapBuffers(window);

      glfwPollEvents();
    }
    glfwDestroyWindow(window);
  }

  // ---- helpers ------------------------------------------------------------

  static inline void applyLocalMotion(GLFWViewer* v, const Eigen::Isometry3f& motion) {
    v->camera_pose = v->camera_pose * motion;
  }

  static inline float distScale(const GLFWViewer* v) {
    return 1.f + v->camera_pose.translation().norm() * 0.05f;
  }

  // ---- key handler --------------------------------------------------------

  bool GLFWViewer::handleKey(int key, int /*scancode*/, int action, int mods) {
    if (action == GLFW_RELEASE)
      return false;

    const float trans_step = 0.5f * distScale(this);
    const float rot_step   = 0.02f;

    // alt + arrows: rotate light
    if (mods & GLFW_MOD_ALT) {
      float angle = rot_step;
      Eigen::Vector3f axis = Eigen::Vector3f::Zero();
      switch (key) {
      case GLFW_KEY_LEFT:
        axis = Eigen::Vector3f::UnitY();
        break;
      case GLFW_KEY_RIGHT:
        axis = Eigen::Vector3f::UnitY();
        angle = -rot_step; break;
      case GLFW_KEY_UP:
        axis = Eigen::Vector3f::UnitX();
        break;
      case GLFW_KEY_DOWN:
        axis = Eigen::Vector3f::UnitX();
        angle = -rot_step; break;
      case GLFW_KEY_H:
        _draw_hud=!_draw_hud;
        return true;
      case GLFW_KEY_L:
        _draw_log=!_draw_log;
        return true;

      default: return false;
      }
      light_direction = Eigen::AngleAxisf(angle, axis) * light_direction;
      return true;
    }

    Eigen::Isometry3f motion = Eigen::Isometry3f::Identity();

    switch (key) {
    case GLFW_KEY_ESCAPE:
      glfwSetWindowShouldClose(window, true);
      return true;

    // arrow keys: pan in screen XY
    // screen right = camera_pose -Z,  screen up = camera_pose -X
    case GLFW_KEY_LEFT:  motion.translation().z() =  trans_step; break;
    case GLFW_KEY_RIGHT: motion.translation().z() = -trans_step; break;
    case GLFW_KEY_UP:    motion.translation().x() = -trans_step; break;
    case GLFW_KEY_DOWN:  motion.translation().x() =  trans_step; break;

    // page up/down: dolly (plain) or roll (shift)
    // forward = camera_pose -Y
    case GLFW_KEY_PAGE_UP:
      if (mods & GLFW_MOD_SHIFT)
        motion.linear() = Eigen::AngleAxisf( rot_step, Eigen::Vector3f::UnitY()).toRotationMatrix();
      else
        motion.translation().y() = -trans_step;
      break;
    case GLFW_KEY_PAGE_DOWN:
      if (mods & GLFW_MOD_SHIFT)
        motion.linear() = Eigen::AngleAxisf(-rot_step, Eigen::Vector3f::UnitY()).toRotationMatrix();
      else
        motion.translation().y() =  trans_step;
      break;

    case GLFW_KEY_Q:
      motion.linear() = Eigen::AngleAxisf( rot_step, Eigen::Vector3f::UnitY()).toRotationMatrix();
      break;
    case GLFW_KEY_E:
      motion.linear() = Eigen::AngleAxisf(-rot_step, Eigen::Vector3f::UnitY()).toRotationMatrix();
      break;

    case GLFW_KEY_R:
      camera_pose.setIdentity();
      camera_pose.translation() << 0.f, 0.f, 30.f;
      return true;

    default:
      return false;
    }

    applyLocalMotion(this, motion);
    return true;
  }

  // ---- mouse callbacks ----------------------------------------------------

  void GLFWViewer::mouse_callback(GLFWwindow* window, double xposIn, double yposIn) {
    GLFWViewer* v = static_cast<GLFWViewer*>(glfwGetWindowUserPointer(window));
    if (!v) throw std::runtime_error("no viewer");

    Eigen::Vector2f prev = v->current_mouse;
    v->current_mouse << float(xposIn), float(yposIn);
    if (ImGui::GetIO().WantCaptureMouse) return;
    if (v->manipulation_mode == None) return;

    Eigen::Vector2f delta = v->current_mouse - prev;  // positive = right / down in screen
    const float rot_scale   = 0.005f;
    const float trans_scale = 0.006f * distScale(v);

    Eigen::Isometry3f motion = Eigen::Isometry3f::Identity();
    switch (v->manipulation_mode) {
    case Rotate:
      // yaw   = around camera_pose X:  drag right (delta.x>0) -> yaw right
      // pitch = around camera_pose Z:  drag down  (delta.y>0) -> pitch down
      motion.linear() =
        (Eigen::AngleAxisf(-delta.x() * rot_scale, Eigen::Vector3f::UnitX())
       * Eigen::AngleAxisf(-delta.y() * rot_scale, Eigen::Vector3f::UnitZ())).toRotationMatrix();
      break;
    case Translate:
      // screen right = camera_pose -Z,  screen down = camera_pose +X
      motion.translation().z() =  delta.x() * trans_scale;
      motion.translation().x() = -delta.y() * trans_scale;
      break;
    default:
      break;
    }
    applyLocalMotion(v, motion);
  }

  void GLFWViewer::scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
    if (ImGui::GetIO().WantCaptureMouse) return;

    GLFWViewer* v = static_cast<GLFWViewer*>(glfwGetWindowUserPointer(window));
    if (!v) throw std::runtime_error("no viewer");
    // scroll up (yoffset > 0) -> zoom in -> forward = camera_pose -Y
    const float zoom_scale = 2.f * distScale(v);
    Eigen::Isometry3f motion = Eigen::Isometry3f::Identity();
    motion.translation().y() = -float(yoffset) * zoom_scale;
    applyLocalMotion(v, motion);
  }

  void GLFWViewer::mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
    if (ImGui::GetIO().WantCaptureMouse) return;

    GLFWViewer* v = static_cast<GLFWViewer*>(glfwGetWindowUserPointer(window));
    if (!v) throw std::runtime_error("no viewer");
    (void)mods;
    if (button == GLFW_MOUSE_BUTTON_LEFT)
      v->manipulation_mode = (action == GLFW_PRESS) ? Rotate    : None;
    else if (button == GLFW_MOUSE_BUTTON_RIGHT)
      v->manipulation_mode = (action == GLFW_PRESS) ? Translate : None;
  }

  void GLFWViewer::framebuffer_size_callback(GLFWwindow* /*window*/, int width, int height) {
    glViewport(0, 0, width, height);
  }

  void GLFWViewer::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);

    GLFWViewer* v = static_cast<GLFWViewer*>(glfwGetWindowUserPointer(window));
    if (!v) throw std::runtime_error("no viewer");

    // H and L toggle HUD/log regardless of ImGui focus
    if (action == GLFW_PRESS) {
      if (key == GLFW_KEY_H) { v->_draw_hud = !v->_draw_hud; return; }
      if (key == GLFW_KEY_L) { v->_draw_log = !v->_draw_log; return; }
    }

    if (ImGui::GetIO().WantCaptureKeyboard)
      return;
    v->handleKey(key, scancode, action, mods);
  }

  GLFWViewer::~GLFWViewer() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui::DestroyContext();

  }



  void GLFWViewer::_drawHUD(const char* hud_text) {
    if (! hud_text)
      return;

    ImVec2 screen = ImGui::GetIO().DisplaySize;
    float hud_border=10;
    float line_height = ImGui::GetTextLineHeightWithSpacing();
    float hud_height = _hud_lines * line_height + ImGui::GetStyle().WindowPadding.y *2;
    ImGui::SetNextWindowPos({hud_border, hud_border});
    ImGui::SetNextWindowBgAlpha(0.6f);
    ImGui::SetNextWindowSize({(screen.x-hud_border)*_hud_width, hud_height});
    ImGui::Begin("##hud", nullptr,
                 ImGuiWindowFlags_NoDecoration |
                 ImGuiWindowFlags_NoResize |
                 ImGuiWindowFlags_NoNav);

    if (ImGui::IsWindowHovered()) {
      if (ImGui::IsKeyPressed((ImGuiKey)GLFW_KEY_DOWN))
        _hud_lines = std::min(_hud_lines + 1, 50);
      if (ImGui::IsKeyPressed((ImGuiKey)GLFW_KEY_UP))
        _hud_lines = std::max(_hud_lines - 1, 1);
      if (ImGui::IsKeyPressed((ImGuiKey)GLFW_KEY_RIGHT)) {
        _hud_width+=0.05;
        _hud_width=std::min(1.f, _hud_width);
      }
      if (ImGui::IsKeyPressed((ImGuiKey)GLFW_KEY_LEFT)) {
        _hud_width-=0.05;
        _hud_width=std::max(0.1f, _hud_width);
      }
    }

    ImGui::TextUnformatted(hud_text);
    ImGui::End();

  }

  void GLFWViewer::_drawLog(const std::list<std::string>& log) {
    ImVec2 screen = ImGui::GetIO().DisplaySize;
    int max_lines=_log_lines;
    float log_border=10;
    float line_height = ImGui::GetTextLineHeightWithSpacing();
    float log_height = max_lines * line_height + ImGui::GetStyle().WindowPadding.y * 2;

    ImGui::SetNextWindowPos({log_border, screen.y-log_border-log_height});
    ImGui::SetNextWindowBgAlpha(0.6f);
    ImGui::SetNextWindowSize({(screen.x-log_border)*_log_width, log_height});
    ImGui::Begin("##log", nullptr,
                 ImGuiWindowFlags_NoDecoration |
                 ImGuiWindowFlags_NoResize |
                 ImGuiWindowFlags_NoNav);

    if (ImGui::IsWindowHovered()) {
      if (ImGui::IsKeyPressed((ImGuiKey)GLFW_KEY_UP))
        _log_lines = std::min(_log_lines + 1, 50);
      if (ImGui::IsKeyPressed((ImGuiKey)GLFW_KEY_DOWN))
        _log_lines = std::max(_log_lines - 1, 1);
      if (ImGui::IsKeyPressed((ImGuiKey)GLFW_KEY_RIGHT)) {
        _log_width+=0.05;
        _log_width=std::min(1.f, _log_width);
      }
      if (ImGui::IsKeyPressed((ImGuiKey)GLFW_KEY_LEFT)) {
        _log_width-=0.05;
        _log_width=std::max(0.1f, _log_width);
      }
    }
    for (auto it=log.rbegin();
         it!=log.rend() && max_lines;
         ++it) {
      ImGui::TextUnformatted(it->c_str());
      --max_lines;
    }
    ImGui::End();

  }

  std::list<std::string> GLFWViewer::_empty;
} // namespace kd_slam
