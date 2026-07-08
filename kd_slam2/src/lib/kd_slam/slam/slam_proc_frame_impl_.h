#include <iomanip>
#pragma once
namespace kd_slam {
  namespace slam {
    using namespace kd_slam::frame;

    template <typename T_>
    void SLAMProc_<T_>::addFrame(const FrameTreePtr& src_) {
      using namespace std;
      syncParams();

      BaseType::addFrame(src_);

      if (! _keyframe) {
        addKeyframe();
        propagateCovarianceAndDistance();
        return;
      }

      auto previous_kf = _keyframe;

      auto t0 = chrono::steady_clock::now();
      PGOFactorPtr f_loop = seekLoops();
      double t_loop = chrono::duration<double, milli>(chrono::steady_clock::now() - t0).count();

      double t_rel = 0;
      if (f_loop) {
        EventRelocalize ev_rel(_floating_frame->ts);
        _keyframe=std::static_pointer_cast<Frame>(_map->frame(f_loop->to_ref));
        _pose_in_kf=f_loop->Z_to_from*_pose_in_kf;
        _floating_frame->pose_in_world=_keyframe->pose_in_world*_pose_in_kf;
        ev_rel.matched_ref=_keyframe->ref();
        _motion_model->onOriginReset(_pose_in_kf);
        pushEvent(std::make_shared<EventRelocalize>(ev_rel));
        pushEvent(std::make_shared<EventOdometry>(_floating_frame->ts,
                                                 _pose_in_kf,
                                                 _floating_frame->pose_in_world,
                                                 f_loop->stats));
      } else {
        if (shouldSwitchKeyframe(_pose_in_kf, _odom_aligner->stats)) {
          t0 = chrono::steady_clock::now();
          bool relocalize_ok = relocalize();
          t_rel = chrono::duration<double, milli>(chrono::steady_clock::now() - t0).count();
          if (relocalize_ok)
            _status = Relocalized;
          if (!relocalize_ok)
            addKeyframe();
        }
      }
      double t_opt = 0;
      Scalar pchi = _pending_chi2;
      if (_solver && _need_optimize) {
        if (_pending_chi2 > this->_optimizer_params.pgo_chi2_threshold) {
          _need_optimize = false;
          t0 = chrono::steady_clock::now();
          optimize();
          t_opt = chrono::duration<double, milli>(chrono::steady_clock::now() - t0).count();
          _floating_frame->pose_in_world = _keyframe->pose_in_world * _pose_in_kf;
          pushOptimizeEvent(_floating_frame->ts);
          _pending_chi2 = 0;
        }
      }

      pushEvent(std::make_shared<EventFrameProcessed>(_status, _t_align, 0, t_loop, t_rel, t_opt, pchi));

      if (previous_kf != _keyframe) {
        t0 = chrono::steady_clock::now();
        propagateCovarianceAndDistance();
      }
    }

    template <typename T_>
    void SLAMProc_<T_>::addKeyframe() {
      using namespace std;
      static constexpr int MAX_FRAMES = MapType::MAX_FRAMES;
      bool fix_frame        = (_keyframe == nullptr);
      auto previous_keyframe = _keyframe;
      _keyframe = _floating_frame;
      _map->addFrame(_keyframe);
      if (_solver) {
        auto var = std::make_shared<SolverVariableType>();
        var->setGraphId(_keyframe->ref());
        var->setEstimate(_keyframe->pose_in_world);
        if (fix_frame)
          var->setStatus(srrg2_solver::VariableBase::Fixed);
        factorGraph()->addVariable(var);
        _keyframe->solver_variable = var.get();

        int vel_ref = MAX_FRAMES + _keyframe->ref();
        auto vel_var = std::make_shared<SolverVelocityVariableType>();
        vel_var->setGraphId(vel_ref);
        vel_var->setEstimate(_keyframe->velocity);
        factorGraph()->addVariable(vel_var);
        _keyframe->solver_velocity = vel_var.get();
        vel_var->setStatus(srrg2_solver::VariableBase::Fixed);

        auto prior = std::make_shared<SolverVelocityPriorFactorType>();
        prior->setVariableId(0, vel_ref);
        prior->setMeasurement(_keyframe->velocity);
        prior->setInformationMatrix(
          SolverVelocityPriorFactorType::InformationMatrixType::Identity() * this->_optimizer_params.velocity_prior_info);
        factorGraph()->addFactor(prior);
        prior->setEnabled(false);
        _keyframe->solver_velocity_prior = prior.get();

      }

      _motion_model->onKeyframe(IsometryType::Identity());
      _motion_model->onOriginReset(IsometryType::Identity());
      if (_descriptor_matcher)
        _descriptor_matcher->addDescriptor(_keyframe->descriptor, _keyframe->ref());

      if (previous_keyframe) {
        PGOFactorPtr f_ptr;
        if (_odom_aligner->alignerType() == AlignerCTICP) {
        
          Match m(previous_keyframe, _keyframe);
          m.initFromGraph();
          m.rematch(*_local_aligner, _slam_params.factor_thresholds, Odometry);
          f_ptr = makeFactor(m, Odometry);
        } else {
           f_ptr = makeFactor(previous_keyframe, _keyframe, Odometry, true);
        }
        pushEvent(std::make_shared<EventKeyframeAdded>(_keyframe->ts,
                                                       _keyframe->ref(),
                                                       previous_keyframe->ref(),
                                                       _keyframe->pose_in_world,
                                                       f_ptr->stats));
        addFactor(f_ptr);
      } else {
        pushEvent(std::make_shared<EventKeyframeAdded>(_keyframe->ts,
                                                       _keyframe->ref(),
                                                       -1,
                                                       _keyframe->pose_in_world,
                                                       ICPStats()));
      }
      _pose_in_kf.setIdentity();
    }

  } // namespace slam
} // namespace kd_slam
