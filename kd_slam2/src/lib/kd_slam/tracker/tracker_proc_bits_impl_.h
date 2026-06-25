#pragma once
#include <fstream>
namespace kd_slam {
  namespace slam {

    template <typename Base_>
    bool TrackerProc_<Base_>::canCompute() const {
      return _odom_aligner != nullptr || _motion_model != nullptr;
    }

    template <typename Base_>
    bool TrackerProc_<Base_>::checkCompute() const {
      bool is_gpu = false, is_cpu = false;
      if (_odom_aligner) {
        is_gpu |=  _odom_aligner->isGPU();
        is_cpu |= !_odom_aligner->isGPU();
      }
      return !(is_cpu && is_gpu);
    }

    template <typename Base_>
    bool TrackerProc_<Base_>::isGPU() const {
      if (_odom_aligner)
        return _odom_aligner->isGPU();
      return false;
    }

    template <typename Base_>
    void TrackerProc_<Base_>::reset() {
      _pose_in_kf.setIdentity();
      _map->clear();
      _floating_frame.reset();
      _keyframe.reset();
      _frame_idx = 0;
      _motion_model->reset();
    }

    template <typename Base_>
    void TrackerProc_<Base_>::push(std::shared_ptr<frame::FrameBase> f) {
      if (! f) {
        _status = Ready;
        pushEvent(std::make_shared<EventFrameProcessed>(Ready, 0.0, 0.0, 0.0, 0.0));
        return;
      }
      auto ft = std::dynamic_pointer_cast<FrameTree>(f);
      if (ft) {
        addFrame(ft);
      } 
    }

    
    template <typename Base_>
    void TrackerProc_<Base_>::syncParams() {
      if (!_param_changed)
        return;
      _tracker_params.min_inlier_frac       = param_min_inlier_frac.value();
      _tracker_params.max_kf_trans          = param_max_kf_trans.value();
      _tracker_params.max_kf_rot_rad        = param_max_kf_rot_deg.value() * float(M_PI / 180.);
      _odom_aligner        = param_odom_aligner.value();
      _descriptor_matcher  = param_descriptor_matcher.value();
      _motion_model = param_motion_model.value();
      if (_motion_model)
        _motion_model->setTracker(this);
      _param_changed = false;
    }

  } // namespace slam
} // namespace kd_slam
