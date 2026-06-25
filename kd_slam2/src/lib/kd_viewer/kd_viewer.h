#pragma once
#include "glfw_viewer.h"
#include <Eigen/Core>
#include "kd_slam/utils/runnable.h"
#include "drawable_base.h"
#include <functional>
namespace kd_slam {

  // Non-template base: handles GL setup so glad symbols stay in kd_viewer.so
  struct KDViewer : public GLFWViewer {

    template <typename DrawableSLAMType>
    KDViewer(kd_slam::utils::Runnable& runnable_,
             std::shared_ptr<DrawableSLAMType> drawable_):
      _runnable(runnable_),
      _drawable(drawable_){
      DrawableSLAMType::initShaders();
      initCamera();
    }
    
    void initCamera();

    bool handleKey(int key, int cloudcode, int action, int mods) override;
    std::function<void()> on_bundle;
    std::function<void()> on_ct_bundle;
    std::function<void()> on_map_save;
    std::function<void()> on_add_pose_noise;
    std::function<void()> on_add_vel_noise;
    std::function<void()> on_ct_bundle_vel_only;
  protected:
    void _drawScene() override;
    const char* _getHUD() const override;
    const std::list<std::string>& _getLog() const override;

    kd_slam::utils::Runnable& _runnable;
    std::shared_ptr<DrawableBase> _drawable;

    bool _follow_cam = false;
  };

} // namespace kd_slam
