#pragma once
#include "camera_pose_vbo.h"
#include "cloud_vbo.h"
#include "drawable_base.h"
#include <Eigen/Geometry>
#include <memory>
#include <vector>

namespace kd_slam {

  struct DrawableKDCloud : public DrawableBase {
    DrawableKDCloud(const Eigen::Isometry3f& pose,
                    const std::vector<PointNormal3fKDCloudVBO::PointType>& leaves);
    void updateBuffer(const Eigen::Isometry3f& pose,
                const std::vector<PointNormal3fKDCloudVBO::PointType>& leaves_);
    static void initShaders();
    void update() override {}
    void draw(const Eigen::Matrix4f& projection,
              const Eigen::Matrix4f& model_pose,
              const Eigen::Matrix4f& object_pose,
              const Eigen::Vector3f& light_direction) override;

    Eigen::Isometry3f pose_in_world;
    std::shared_ptr<PointNormal3fKDCloudVBO> _cloud_vbo;
    static std::shared_ptr<CameraPoseVBO> _camera_pose_vbo;
    bool show_camera = true;
    bool show_cloud  = true;
    // horrible but functional
    std::vector<PointNormal3fKDCloudVBO::PointType> leaves;
  };

  using DrawableKDCloudPtr = std::shared_ptr<DrawableKDCloud>;

  void setGLPointSize(int size);

} // namespace kd_slam
