#include "drawable_kd_slam_.h"
#include "kd_slam/d2/typedefs.h"
#include "kd_slam/d3/typedefs.h"
#include <sstream>
#include <iomanip>
#include <iostream>

namespace kd_slam {
  namespace slam {
    using namespace std;

    template <typename NodeType_>
    void DrawableKDSlam_<NodeType_>::initShaders() {
      DrawableKDCloud::initShaders();
      LineSegmentsVBO::getShaderProgram();
    }

    template <typename NodeType_>
    DrawableKDSlam_<NodeType_>::DrawableKDSlam_() {
      
      registerCallback<EvAdded>([this](std::shared_ptr<EvAdded> ev) {
        std::lock_guard<std::mutex> guard(draw_mutex);
        auto vpoints    = _toVec3f(ev->vpoints);
        auto cloud=std::make_shared<DrawableKDCloud>(Eigen::Isometry3f::Identity(), vpoints);
        if(_current_cloud && ev->kf_ref<0) {
          //cerr << "no keyframe" << ev->count << endl;
          _frames[ev->count]=cloud;
          return;
        }
        _current_cloud  = cloud;
        _current_count  = ev->count;
        _current_kf_ref = ev->kf_ref;
        if (_status==ICPBundling || _status==CTICPBundling)
          return;
        ostringstream os;
        ev->print(os);
        writeHUD("%s\n", os.str().c_str());
      });

      registerCallback<EvOdom>([this](std::shared_ptr<EvOdom> ev) {
        std::lock_guard<std::mutex> guard(draw_mutex);
        if (_current_cloud) {
          _current_cloud->pose_in_world = toIsometry3f(ev->pose_global);
          if (ev->moving_stats.size()) {
            if (_current_cloud->leaves.size()!=ev->moving_stats.size())
              throw std::runtime_error("viewer bookeeping");
            auto base_color=palette_ift.cloud_color;
            uint8_t base_r=uint8_t(base_color.x()*255);
            uint8_t base_g=uint8_t(base_color.y()*255);
            uint8_t base_b=uint8_t(base_color.z()*255);
            size_t size=_current_cloud->leaves.size();
            for (size_t i=0; i<size; ++i) {
              auto& d=_current_cloud->leaves[i];
              const auto& s=ev->moving_stats[i];
              auto& r=d.rgba[0];
              auto& g=d.rgba[1];
              auto& b=d.rgba[2];
              auto& a=d.rgba[3];
              // exactly one of these fires per leaf, see ICP_::quadraticTerm
              if (s.num_bad) {
                r=255; g=0;   b=0;
              } else if (s.num_err_fn) {
                r=255; g=140; b=0;
              } else if (s.num_cut) {
                r=255; g=100; b=0;
              } else if (s.num_err_normal) {
                r=0;   g=255; b=0;
              } else if (s.num_outliers) {
                r=255; g=220; b=0;
              } else {
                r=base_r; g=base_g; b=base_b;
              }
              a=255;
            }
            _current_cloud->_cloud_vbo->updateBuffer(_current_cloud->leaves);
          }
          ostringstream os;
          ev->print(os);
          writeHUD("%s\n", os.str().c_str());
        }
      });

      registerCallback<EvKF>([this](std::shared_ptr<EvKF> ev) {
        std::lock_guard<std::mutex> guard(draw_mutex);
        Eigen::Isometry3f pose = toIsometry3f(ev->pose_global);
        if (_current_cloud)
          _current_cloud->pose_in_world = pose;
        _frames[_current_count]        = _current_cloud;
        _keyframes[ev->kf_ref]    = _current_count;
        _current_cloud            = nullptr;
        _kf_positions[ev->kf_ref] = pose.translation();
        if (_status==ICPBundling || _status==CTICPBundling)
          return;
        ostringstream os;
        ev->print(os);
        log.push_back(os.str());
      });

      registerCallback<EvFact>([this](std::shared_ptr<EvFact> ev) {
        std::lock_guard<std::mutex> guard(draw_mutex);
        Edge e{ev->from_ref, ev->to_ref, ev->type};
        _edges[ev->key]=e;
      });

      registerCallback<EvFactChange>([this](std::shared_ptr<EvFactChange> ev) {
        std::lock_guard<std::mutex> guard(draw_mutex);
        auto it=_edges.find(ev->key);
        if  (it==_edges.end())
          return;
        it->second.enabled=ev->enabled;
      });

      registerCallback<EvPGO>([this](std::shared_ptr<EvPGO> ev) {
        std::lock_guard<std::mutex> guard(draw_mutex);
        for (auto& [ref, iso] : ev->poses_after) {
          Eigen::Isometry3f pose_f = toIsometry3f(iso);
          _kf_positions[ref] = pose_f.translation();
          auto kf=keyframeCloud(ref);
          if (! kf)
            continue;
          kf->pose_in_world = pose_f;
        }
        if (! ev->message.empty()){
          log.push_back(ev->message);
        }
      });

      registerCallback<EvLoop>([this](std::shared_ptr<EvLoop> ev) {
        std::lock_guard<std::mutex> guard(draw_mutex);
        _loop_spatial_indices.clear();
        _loop_appearance_indices.clear();
        for (const auto& m : ev->matches) {
          _loop_spatial_indices.insert(m.ref);
          if (m.descriptor_distance > 0)
            _loop_appearance_indices.insert(m.ref);
        }
        ostringstream os;
        if (ev->matches.size()) {
          os << "[Loop] ";
          for (const auto& m : ev->matches) {
            os << "\t (ref: " << m.ref
               << fixed << setprecision(1)
               << " chi: " << m.chi2_spatial
               << " dst: " << m.descriptor_distance
               << " ang: " << m.rotation_delta*(180.f/M_PI)
               << " trs: " << m.translation_delta
               << " scr: " << m.score
               << " cov: " << m.coverage
               << " inl: " << m.inlier_ratio * 100
               << " res: " << MatchLabelResultStr[m.match_result]
               << ") " << endl;
          }
          log.push_back(os.str());
        }
      });

      registerCallback<EvReloc>([this](std::shared_ptr<EvReloc> ev) {
        std::lock_guard<std::mutex> guard(draw_mutex);
        _local_map_indices.clear();
        for (int r : ev->local_map_refs)
          _local_map_indices.insert(r);
        ostringstream os;
        ev->print(os);
        log.push_back(os.str());
      });

      using EvProc = EventFrameProcessed_<NodeType_>;
      registerCallback<EvProc>([this](std::shared_ptr<EvProc> ev) {
        std::lock_guard<std::mutex> guard(draw_mutex);
        _pending_deletes.push_back(_rendered_cloud);
        _rendered_cloud                   = _current_cloud;
        _rendered_kf_ref                  = _current_kf_ref;
        _rendered_local_map_indices       = _local_map_indices;
        _rendered_loop_spatial_indices    = _loop_spatial_indices;
        _rendered_loop_appearance_indices = _loop_appearance_indices;
        ostringstream os;
        ev->print(os);
        _status=ev->status;
        writeHUD("%s", os.str().c_str());
        postHUD();
      });
    }

    template <typename NodeType_>
    const PaletteFrame& DrawableKDSlam_<NodeType_>::_getKFPalette(int ref) const {
      if (_rendered_local_map_indices.count(ref))
        return palette_lmap;
      if (_rendered_loop_spatial_indices.count(ref))
        return palette_loopsp;
      if (_rendered_loop_appearance_indices.count(ref))
        return palette_loopd;
      return palette_kf;
    }

    template <typename NodeType_>
    void DrawableKDSlam_<NodeType_>::draw(const Eigen::Matrix4f& projection,
                                          const Eigen::Matrix4f& model_pose,
                                          const Eigen::Matrix4f& object_pose,
                                          const Eigen::Vector3f& light_direction) {
      if (!_segments)
        _segments.reset(new LineSegmentsVBO);

      std::lock_guard<std::mutex> guard(draw_mutex);
      if (_rendered_cloud)
        _rendered_camera_pose = _rendered_cloud->pose_in_world;
      _segments->clear();
      for (auto& [k,e] : _edges) {
        if (! e.enabled) {
          continue;
        }
        auto ia = _kf_positions.find(e.from);
        auto ib = _kf_positions.find(e.to);
        if (ia == _kf_positions.end() || ib == _kf_positions.end()) continue;
        Eigen::Vector3f color;
        switch (e.type) {
        case IFTracking: color = palette_ift.factor_color; break;
        case Odometry:   color = palette_odom.factor_color; break;
        case Relocalize: color = palette_rel.factor_color; break;
        case Loop:       color = palette_loop.factor_color; break;
        case CTICPMultiView:
        case ICPMultiView: color = {0.8f, 0.8f, 0.1f}; break;
        }
        _segments->addSegment(ia->second, ib->second, color);
      }

      if (_rendered_cloud) {
        auto it = _kf_positions.find(_rendered_kf_ref);
        if (it != _kf_positions.end())
          _segments->addSegment(_rendered_cloud->pose_in_world.translation(),
                                it->second,
                                palette_ift.factor_color);
      }

      for (auto& [ref, count] : _keyframes) {
        auto it=_frames.find(count);
        if (it ==_frames.end())
          continue;
        auto kf=it->second;
        setGLPointSize(1);
        kf->show_cloud  = show_clouds;
        kf->show_camera = show_cameras;
        if (kf->_cloud_vbo) {
          kf->_cloud_vbo->point_color = _getKFPalette(ref).cloud_color;
          kf->_cloud_vbo->decimation  = cloud_decimation;
          kf->_cloud_vbo->show_normals=show_normals;
        }
        kf->draw(projection, model_pose, object_pose, light_direction);
      }

      if (_rendered_cloud) {
        setGLPointSize(5);
        _rendered_cloud->show_cloud  = show_clouds;
        _rendered_cloud->show_camera = show_cameras;
        if (_rendered_cloud->_cloud_vbo) {
          _rendered_cloud->_cloud_vbo->point_color=Eigen::Vector3f(-1,-1,-1);
          _rendered_cloud->_cloud_vbo->decimation  = cloud_decimation;
          _rendered_cloud->_cloud_vbo->show_normals=show_normals;
        }
        _rendered_cloud->draw(projection, model_pose, object_pose, light_direction);
      }

      if (show_matches)
        _segments->draw(projection, model_pose, object_pose, light_direction);
      _pending_deletes.clear();
    }

    template <typename NodeType_>
    Eigen::Isometry3f DrawableKDSlam_<NodeType_>::cameraPose() const {
      return _rendered_camera_pose;
    }

    template <typename NodeType_>
    std::vector<PointNormal3fKDCloudVBO::PointType>
    DrawableKDSlam_<NodeType_>::_toVec3f(const std::vector<VPoint>& in) const {
      using PointType = PointNormal3fKDCloudVBO::PointType;
      std::vector<PointType> out;
      out.reserve(in.size());
      for (const auto& v : in) {
        auto m=v.point.template cast<float>();
        auto n=v.normal.template cast<float>();
        PointType p;
        if constexpr (Dim == 3) {
          p.point  = m;
          p.normal = n;
        } else {
          p.point  = Eigen::Vector3f(float(m.x()), float(m.y()), 0.f);
          p.normal = Eigen::Vector3f(float(n.x()), float(n.y()), 0.f);
        }
        p.rgba[0]=v.rgba[0]; p.rgba[1]=v.rgba[1]; p.rgba[2]=v.rgba[2]; p.rgba[3]=v.rgba[3];
        out.push_back(p);
      }
      return out;
    }

    // explicit instantiations
    template struct DrawableKDSlam_<kd_slam::d2::NodeType>;
    template struct DrawableKDSlam_<kd_slam::d3::NodeType>;

  } // namespace slam
} // namespace kd_slam
