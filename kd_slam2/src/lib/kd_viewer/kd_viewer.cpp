#include "kd_viewer.h"
#include <iostream>
#include "imgui/imgui.h"
#include "imgui/examples/imgui_impl_glfw.h"
#include "imgui/examples/imgui_impl_opengl3.h"
#include <thread>

namespace kd_slam {
  using namespace std;
  void KDViewer::_drawScene() {
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    if (h == 0) h = 1;
    Eigen::Matrix4f projection = computeGLPerspectiveMatrix(
      fov, float(w), float(h), 1.f, 10000.f);
    Eigen::Isometry3f effective_pose = _follow_cam
      ? _drawable->cameraPose() * camera_pose
      : camera_pose;
    Eigen::Matrix4f model_pose =
      (gl_camera_transform * effective_pose.inverse()).matrix();
    Eigen::Matrix4f object_pose = Eigen::Matrix4f::Identity();
    _drawable->draw(projection, model_pose, object_pose, light_direction);
  }

  void KDViewer::initCamera() {
    camera_pose.translation() << 0, 0, 20;
    camera_pose.linear() = Eigen::AngleAxisf(M_PI/2.f, Eigen::Vector3f::UnitX()).toRotationMatrix();

  }
  
  bool KDViewer::handleKey(int key, int cloudcode, int action, int mods) {
    if (action == GLFW_RELEASE)
      return false;
    if (! _drawable)
      return false;
    switch(key) {
    case GLFW_KEY_SPACE:
      _runnable.run(! _runnable.running());
      return true;
    case GLFW_KEY_S:
      _runnable.setStepMode(! _runnable.stepMode());
      return true;
    case GLFW_KEY_B:
      if (on_bundle) {
        std::thread([this](){ on_bundle(); }).detach();
      }
      return true;
    case GLFW_KEY_C:
      if (on_ct_bundle){
        std::thread([this](){ on_ct_bundle(); }).detach();
      }
      return true;
    case GLFW_KEY_G:
      if (on_map_doctor){
        std::thread([this](){ on_map_doctor(); }).detach();
      }
      return true;
    case GLFW_KEY_W:
      if (on_ct_bundle_vel_only) {
        std::thread([this](){ on_ct_bundle_vel_only(); }).detach();
      }
      return true;
    case GLFW_KEY_P:
      if (on_add_pose_noise) on_add_pose_noise();
      return true;
    case GLFW_KEY_V:
      if (on_add_vel_noise) on_add_vel_noise();
      return true;
    case GLFW_KEY_M:
      if (on_map_save)
        on_map_save();
      return true;
    case GLFW_KEY_D: {
      int d = _drawable->cloud_decimation;
      _drawable->cloud_decimation = (d >= 8) ? 1 : d * 2;
      return true;
    }
    case GLFW_KEY_F:
      _follow_cam = !_follow_cam;
      return true;
    default:
      return GLFWViewer::handleKey(key, cloudcode, action, mods);
    }
    return false;
  }

  const char* KDViewer::_getHUD() const {
    if (_drawable) {
      return _drawable->getHUDtext();
    }
    return nullptr;
  }
  
  const std::list<std::string>& KDViewer::_getLog() const {
    if (! _drawable)
      return _empty;
    return _drawable->log;
  }

} // namespace kd_slam
