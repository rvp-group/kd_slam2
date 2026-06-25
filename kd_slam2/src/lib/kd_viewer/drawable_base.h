#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <list>

namespace kd_slam {
  struct DrawableBase {
    static constexpr int MAX_HUD_CHARS=4096;
    

    int cloud_decimation = 1;

    virtual void update() {}
    virtual Eigen::Isometry3f cameraPose() const { return Eigen::Isometry3f::Identity(); }
    virtual void draw(const Eigen::Matrix4f& projection,
                      const Eigen::Matrix4f& model_pose,
                      const Eigen::Matrix4f& object_pose,
                      const Eigen::Vector3f& light_direction) = 0;
    virtual ~DrawableBase() {}
    const char* getHUDtext() const;
    void writeHUD(const char* fmt, ...);
    void postHUD();
    std::list<std::string> log;
  protected:
    mutable std::mutex draw_mutex;
    char hud_text[2][MAX_HUD_CHARS];
    char* write_hud_start=hud_text[0];
    char* write_hud_end=hud_text[0];
    char* read_hud_start=hud_text[1];
    char* read_hud_end=hud_text[1];

  };

  template <typename T_>
  struct Drawable_: public DrawableBase {
    using InstanceType = T_;
    Drawable_(InstanceType instance_): _instance(instance_){}
    InstanceType _instance;
  };

}
