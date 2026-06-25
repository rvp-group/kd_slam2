#pragma once
#include "drawable_base.h"
#include "drawable_kd_cloud.h"
#include "line_segments_vbo.h"
#include "kd_slam/map/map_event_.h"
#include "kd_slam/event/event_dispatcher.h"
#include "kd_slam/utils/geometry_.h"
#include <map>
#include <set>
#include <mutex>
#include <memory>
#include <list>
#include "palette.h"

namespace kd_slam {
  namespace slam {
    using namespace kd_slam::event;
    using namespace kd_slam::map;
    using kd_slam::geometry::toIsometry3f;
    template <typename NodeType_>
    struct DrawableKDSlam_ :
      public DrawableBase,
      public event::EventDispatcher {

      using NodeType     = NodeType_;
      using IsometryType = typename NodeType_::IsometryType;
      using VectorType   = typename NodeType_::VectorType;
      static constexpr int Dim = NodeType_::Dim;

      using EvAdded = EventFrameAdded_<NodeType_>;
      using EvKF    = EventKeyframeAdded_<NodeType_>;
      using EvOdom  = EventOdometry_<NodeType_>;
      using EvFact  = EventFactorAdded_<NodeType_>;
      using EvPGO   = EventPGOOptimized_<NodeType_>;
      using EvLoop  = EventLoopSearch_<NodeType_>;
      using EvReloc = EventRelocalize_<NodeType_>;
      using EvFactChange = EventFactorChanged;
      
      std::mutex draw_mutex;
      std::map<int, DrawableKDCloudPtr> _frames;
      std::map<int, int> _keyframes;
      DrawableKDCloudPtr _rendered_cloud;
      Eigen::Isometry3f  _rendered_camera_pose = Eigen::Isometry3f::Identity();
      int  _rendered_kf_ref = -1;

      DrawableKDCloudPtr _current_cloud;
      int  _current_kf_ref = -1;
      int  _current_count=-1;
      
      std::map<int, Eigen::Vector3f> _kf_positions;
      DrawableKDCloudPtr keyframeCloud(int ref){
        auto it=_keyframes.find(ref);
        if (it==_keyframes.end())
          return nullptr;
        return _frames[it->second];
      }
      
      struct Edge { int from, to; KDFactorType type; bool enabled=true;};
      std::unordered_map <size_t, Edge> _edges;

      std::set<int> _local_map_indices;
      std::set<int> _loop_spatial_indices;
      std::set<int> _loop_appearance_indices;

      std::set<int> _rendered_local_map_indices;
      std::set<int> _rendered_loop_spatial_indices;
      std::set<int> _rendered_loop_appearance_indices;

      std::unique_ptr<LineSegmentsVBO> _segments;

      bool show_matches = true;
      bool show_cameras = true;
      bool show_clouds  = true;

      static void initShaders();

      DrawableKDSlam_();

      void draw(const Eigen::Matrix4f& projection,
                const Eigen::Matrix4f& model_pose,
                const Eigen::Matrix4f& object_pose,
                const Eigen::Vector3f& light_direction) override;
      
      Eigen::Isometry3f cameraPose() const;

#ifdef _PAPER_MODE_
      PaletteFrame palette_ift   = { Eigen::Vector3f(0.3, 0.3, 1.0),
                                     Eigen::Vector3f(0.3, 0.3, 1.0),
                                     Eigen::Vector3f(0.5, 0.5, 0.7)};


      PaletteFrame palette_kf    = { Eigen::Vector3f(0.6, 0.6, 0.6),
                                     Eigen::Vector3f(1.0, 0.0, 0.0),
                                     Eigen::Vector3f(0.5, 0.5, 0.5)};

      PaletteFrame palette_odom  = { Eigen::Vector3f(0.6, 0.6, 0.6),
                                     Eigen::Vector3f(1.0, 0.0, 0.0),
                                     Eigen::Vector3f(0.1, 0.3, 0.1)};

      PaletteFrame palette_rel   = { Eigen::Vector3f(0.6, 0.6, 0.6),
                                     Eigen::Vector3f(1.0, 0.0, 0.0),
                                     Eigen::Vector3f(0.1, 0.5, 0.5)};

      PaletteFrame palette_loop  = { Eigen::Vector3f(0.3, 0.3, 0.3),
                                     Eigen::Vector3f(1.0, 0.0, 0.0),
                                     Eigen::Vector3f(0.8, 0.1, 0.1)};

      PaletteFrame palette_loopsp= { Eigen::Vector3f(0.8, 0.1, 0.1),
                                     Eigen::Vector3f(1.0, 0.0, 0.0),
                                     Eigen::Vector3f(0.5, 0.5, 0.5) };

      PaletteFrame palette_loopd = { Eigen::Vector3f(0.9, 0.1, 0.1),
                                     Eigen::Vector3f(1.0, 0.0, 0.0),
                                     Eigen::Vector3f(1, 0, 0) };
      
      PaletteFrame palette_lmap  = { Eigen::Vector3f(0.5, 0.5, 1.0),
                                     Eigen::Vector3f(1.0, 0.0, 0.0),
                                     Eigen::Vector3f(0.5, 0.5, 0.5) };

      PaletteFrame palette_bundle = { Eigen::Vector3f(0.5, 0.5, 1.0),
                                     Eigen::Vector3f(1.0, 0.0, 0.0),
                                     Eigen::Vector3f(0.5, 0.5, 0.2) };
#else
      PaletteFrame palette_ift   = { Eigen::Vector3f(0.3, 0.3, 1.0),
                                     Eigen::Vector3f(0.3, 0.3, 1.0),
                                     Eigen::Vector3f(0.5, 0.5, 0.7)};


      PaletteFrame palette_kf    = { Eigen::Vector3f(0.3, 0.3, 0.3),
                                     Eigen::Vector3f(1.0, 0.0, 0.0),
                                     Eigen::Vector3f(0.5, 0.5, 0.5)};

      PaletteFrame palette_odom  = { Eigen::Vector3f(0.3, 0.3, 0.3),
                                     Eigen::Vector3f(1.0, 0.0, 0.0),
                                     Eigen::Vector3f(0.1, 0.3, 0.1)};

      PaletteFrame palette_rel   = { Eigen::Vector3f(0.3, 0.3, 0.3),
                                     Eigen::Vector3f(1.0, 0.0, 0.0),
                                     Eigen::Vector3f(0.1, 0.5, 0.5)};

      PaletteFrame palette_loop  = { Eigen::Vector3f(0.3, 0.3, 0.3),
                                     Eigen::Vector3f(1.0, 0.0, 0.0),
                                     Eigen::Vector3f(0.8, 0.1, 0.1)};

      PaletteFrame palette_loopsp= { Eigen::Vector3f(0.6, 0.1, 0.1),
                                     Eigen::Vector3f(1.0, 0.0, 0.0),
                                     Eigen::Vector3f(0.5, 0.5, 0.5) };

      PaletteFrame palette_loopd = { Eigen::Vector3f(0.8, 0.1, 0.1),
                                     Eigen::Vector3f(1.0, 0.0, 0.0),
                                     Eigen::Vector3f(1, 0, 0) };
      
      PaletteFrame palette_lmap  = { Eigen::Vector3f(0.5, 0.5, 1.0),
                                     Eigen::Vector3f(1.0, 0.0, 0.0),
                                     Eigen::Vector3f(0.5, 0.5, 0.5) };

      PaletteFrame palette_bundle = { Eigen::Vector3f(0.5, 0.5, 1.0),
                                     Eigen::Vector3f(1.0, 0.0, 0.0),
                                     Eigen::Vector3f(0.5, 0.5, 0.2) };
#endif
      
    private:
      KDStatus _status=KDStatus::Init;
      std::list<std::shared_ptr<DrawableBase>> _pending_deletes;
      std::vector<Eigen::Vector3f> _toVec3f(const std::vector<VectorType>& in) const;
      const PaletteFrame& _getKFPalette(int ref) const;
    };

  } // namespace slam
} // namespace kd_slam
