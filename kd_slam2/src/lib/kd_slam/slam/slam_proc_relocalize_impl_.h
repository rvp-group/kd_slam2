#pragma once

namespace kd_slam {
  namespace slam {
    using namespace kd_slam::frame;

    template <typename T_>
    bool SLAMProc_<T_>::relocalize() {
      using namespace std;
      Match best;
      EventRelocalize ev_rel(_floating_frame->ts);
      if (_keyframe)
        ev_rel.local_map_refs.push_back(_keyframe->ref());
      for (auto& [ref, frame_base] : _map->frames()) {
        auto frame = static_pointer_cast<Frame>(frame_base);
        if (frame == _keyframe)
          continue;

        Match m(_floating_frame, frame);
        m.initFromGraph();
        m.filter(_slam_params.relocalize_thresholds, _odom_aligner->getPoseHessian().inverse());
        if (m.result != MatchOk) 
          continue;

        ev_rel.local_map_refs.push_back(ref);
        m.rematch(*_local_aligner, _slam_params.relocalize_thresholds, KDFactorType::Relocalize);

        if (m.result!=MatchOk)
          continue;
        
        auto match_frame=m.moving_frame;
        if (m.score > best.score)
          best = m;
        
        // add  a factor between al successful matches
        if (_keyframe && !_map->areConnected(match_frame->ref(), _keyframe->ref())) {
          Match m_kf(_keyframe, match_frame, _pose_in_kf*m.pose);
          m_kf.rematch(*_local_aligner, _slam_params.factor_thresholds, KDFactorType::Relocalize);
          if (m_kf.result == MatchOk
              && m_kf.stats.score() > _slam_params.factor_thresholds.min_score
              && m_kf.stats.inlierRatio()  > _slam_params.relocalize_thresholds.min_inlier_ratio) {
            PGOFactorPtr f_ptr = makeFactor(m_kf, Relocalize);
            addFactor(f_ptr);
          }
        }
      }
      if (best.result != MatchOk || best.score < _slam_params.relocalize_thresholds.min_score) {
        pushEvent(std::make_shared<EventRelocalize>(ev_rel));
        return false;
      }

      if (_keyframe && !_map->areConnected(best.moving_frame->ref(), _keyframe->ref())) {
        using MatrixType = PoseHessianType;
        
        Match m_kf(_keyframe, best.moving_frame, _pose_in_kf*best.pose);
        MatrixType Ad=GeometryTraits::adjoint(_pose_in_kf);
        MatrixType sigma=best.omega.inverse();
        m_kf.omega=(Ad * sigma * Ad.transpose()).inverse();
        auto f=makeFactor(m_kf, Relocalize);
        addFactor(f);
      }
      
      // reparent: hook current frame to found keyframe
      _keyframe   = best.moving_frame;
      _pose_in_kf = best.pose.inverse();
      _floating_frame->pose_in_world = _keyframe->pose_in_world * _pose_in_kf;
      ev_rel.matched_ref=_keyframe->ref();
      _motion_model->onOriginReset(_pose_in_kf);

      pushEvent(std::make_shared<EventRelocalize>(ev_rel));
      pushEvent(std::make_shared<EventOdometry>(_floating_frame->ts,
                                                _pose_in_kf,
                                                _floating_frame->pose_in_world,
                                                best.stats));

      return true;
    }

  }
} // kd_slam::slam
