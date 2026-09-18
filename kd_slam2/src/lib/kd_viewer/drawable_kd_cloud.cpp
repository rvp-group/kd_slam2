#include "drawable_kd_cloud.h"
#include <iostream>

namespace kd_slam {
  using namespace std;

  std::shared_ptr<CameraPoseVBO> DrawableKDCloud::_camera_pose_vbo;

  DrawableKDCloud::DrawableKDCloud(const Eigen::Isometry3f& pose,
                                   const std::vector<PointNormal3fKDCloudVBO::PointType>& leaves) {
    if (!_camera_pose_vbo)
      _camera_pose_vbo = std::make_shared<CameraPoseVBO>(0.3f, 0.3f);
    updateBuffer(pose, leaves);
  }

  void DrawableKDCloud::updateBuffer(const Eigen::Isometry3f& pose,
                               const std::vector<PointNormal3fKDCloudVBO::PointType>& leaves_) {
    pose_in_world=pose;
    leaves=leaves_;
    if (! _cloud_vbo) {
      if (! leaves.empty()) {
        _cloud_vbo = std::make_shared<PointNormal3fKDCloudVBO>(leaves);
      }
    } else {
      _cloud_vbo->updateBuffer(leaves);
    }
  }

  void DrawableKDCloud::initShaders() {
    CameraPoseVBO::getShaderProgram();
    PointNormal3fKDCloudVBO::getShaderProgram();
  }

  void DrawableKDCloud::draw(const Eigen::Matrix4f& projection,
                             const Eigen::Matrix4f& model_pose,
                             const Eigen::Matrix4f& object_pose,
                             const Eigen::Vector3f& light_direction) {
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
