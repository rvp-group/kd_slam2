#pragma once
namespace kd_slam {
  namespace slam {

    template <typename Node_>
    bool Localizer_<Node_>::canCompute() const {
      return TrackerType::canCompute() && _local_aligner != nullptr;
    }

    template <typename Node_>
    bool Localizer_<Node_>::checkCompute() const {
      bool is_gpu = false, is_cpu = false;
      auto check = [&](const auto& a) {
        if (a) { is_gpu |= a->isGPU(); is_cpu |= !a->isGPU(); }
      };
      check(_odom_aligner);
      check(_local_aligner);
      return !(is_cpu && is_gpu);
    }

    template <typename Node_>
    bool Localizer_<Node_>::isGPU() const {
      if (_local_aligner)
        return _local_aligner->isGPU();
      return TrackerType::isGPU();
    }

    template <typename Node_>
    void Localizer_<Node_>::reset() {
      TrackerType::reset();
      _last_real_keyframe.reset();
      _using_dummy_keyframe = false;
      _num_relocalizes      = 0;
    }

    template <typename Node_>
    void Localizer_<Node_>::syncParams() {
      if (!_param_changed)
        return;
      TrackerType::syncParams();
      params.ba_range              = param_ba_range.value();
      params.relocalize_thresholds = {param_relocalize_max_chi2.value(),
                                      param_relocalize_min_inliers_ratio.value(),
                                      param_relocalize_min_score.value(),
                                      -1,
                                      param_relocalize_max_hops.value(),
                                      param_relocalize_slack.value()};
      _local_aligner = param_local_aligner.value();
    }

    template <typename Node_>
    void Localizer_<Node_>::setMap(std::shared_ptr<MapType> m) {
      MapOwnerType::setMap(m);
      applyKFVelocities();
      pushFrameEvents();
      pushFactorEvents();
      pushFactorUpdateEvents();
      _status = Ready;
      pushEvent(std::make_shared<EventFrameProcessed>(Ready, 0.0, 0.0, 0.0, 0.0));
    }

  } // namespace slam
} // namespace kd_slam
