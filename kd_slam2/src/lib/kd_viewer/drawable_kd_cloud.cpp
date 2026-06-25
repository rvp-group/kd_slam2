#include "drawable_kd_cloud.h"
#include <iostream>

namespace kd_slam {
  using namespace std;

  std::shared_ptr<CameraPoseVBO> DrawableKDCloud::_camera_pose_vbo;

  DrawableKDCloud::DrawableKDCloud(const Eigen::Isometry3f& pose,
                                   const std::vector<Eigen::Vector3f>& leaves)
    : pose_in_world(pose) {
    if (leaves.empty()) return;
    _cloud_vbo = std::make_shared<PointNormal3fKDCloudVBO>(leaves);
    if (!_camera_pose_vbo)
      _camera_pose_vbo = std::make_shared<CameraPoseVBO>(0.3f, 0.3f);
  }

  void DrawableKDCloud::initShaders() {
    CameraPoseVBO::getShaderProgram();
    PointNormal3fKDCloudVBO::getShaderProgram();
  }

  void DrawableKDCloud::draw(const Eigen::Matrix4f& projection,
                             const Eigen::Matrix4f& model_pose,
                             const Eigen::Matrix4f& object_pose,
                             const Eigen::Vector3f& light_direction) {
    if (taint) return;
    Eigen::Matrix4f piw = pose_in_world.matrix();
    if (show_cloud && _cloud_vbo) {
      _cloud_vbo->draw(projection, model_pose, piw, light_direction);
    }
    if (show_camera && _camera_pose_vbo) {
      _camera_pose_vbo->draw(projection, model_pose, piw, light_direction);
    }
  }

  void setGLPointSize(int size) { glPointSize(size); }

} // namespace kd_slam
