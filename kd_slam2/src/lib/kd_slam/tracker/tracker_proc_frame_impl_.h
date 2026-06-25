#include <iomanip>
#pragma once
namespace kd_slam {
  namespace slam {
    using namespace kd_slam::frame;

    template <typename Base_>
    typename TrackerProc_<Base_>::FramePtr
    TrackerProc_<Base_>::makeFrame(FrameTreePtr src_) {
      if (!canCompute())
        throw std::runtime_error("Tracker| makeFrame, no compute configured");
      if (!src_)
        return nullptr;
      Runnable::waitRun();
      src_->tree->applyIsometryInPlace(src_->T_base_lidar);
      src_->T_base_lidar.setIdentity();
      DescriptorType desc;
      if (_descriptor_matcher) {
        if (src_->isGPU()) {
          TreeCPU_<TreeBaseType> t_cpu(*src_->tree);
          desc = DescriptorExtractor::extract(t_cpu, 0);
        } else {
          desc = DescriptorExtractor::extract(*src_->tree, 0);
        }
      }
      auto src = FrameTree::toCompute(src_, isGPU());
      return std::make_shared<Frame>(*src, desc, poseGlobal(), _frame_idx);
    }

    // odometry-only addFrame. stops before keyframe management.
    // when !_keyframe (first frame): sets _status=Init, returns without adding keyframe.
    // caller (SLAM_) is responsible for addKeyframe() on Init.
    template <typename Base_>
    void TrackerProc_<Base_>::addFrame(const FrameTreePtr& src_) {
      using namespace std;
      _floating_frame  = makeFrame(src_);

      IsometryType pose_in_kf_prev = _pose_in_kf;
      _motion_model->doPredict(*_floating_frame);
      _floating_frame->pose_in_world = poseGlobal();

      vector<VectorType> compressed_leaves;
      int kf_ref = _keyframe ? _keyframe->ref() : -1;
      pushEvent(std::make_shared<EventFrameAdded>(_floating_frame->ts,
                                                  frameCount(), kf_ref, _pose_in_kf,
                                                  compressed_leaves,
                                                  FrameAddedPlaceholder,
                                                  src_->topic, src_->bag_offset, src_->stamp_ns));
      _floating_frame->velocity.setZero();
      if (_motion_model->frameDuration() > 0) {
        _floating_frame->velocity = _motion_model->velocityLog() / static_cast<Scalar>(_motion_model->frameDuration());
        pushEvent(std::make_shared<EventVelocity>(_floating_frame->ts,
                                                  _floating_frame->frame_count,
                                                  _floating_frame->velocity));
        _floating_frame->applyVelocity();
      }
      compressed_leaves = src_->tree->extractCompressedLeaves();
      pushEvent(std::make_shared<EventFrameAdded>(_floating_frame->ts,
                                                  frameCount(), kf_ref, _pose_in_kf,
                                                  compressed_leaves,
                                                  FrameAddedCloud,
                                                  src_->topic, src_->bag_offset, src_->stamp_ns));
      if (!_keyframe) {
        _pose_in_kf.setIdentity();
        _floating_frame->pose_in_world = IsometryType::Identity();
        _status = Init;
        ++_frame_idx;
        return;
      }

      _pose_in_kf = _pose_in_kf * _motion_model->prediction();

      auto t0 = chrono::steady_clock::now();

      _odom_aligner->setPosePrior(_motion_model->priorXRef(),
                                  _motion_model->priorZ(),
                                  _motion_model->priorOmega());
      alignFrames(*_odom_aligner, _keyframe, _floating_frame, _pose_in_kf, IFTracking);
      _t_align = chrono::duration<double, milli>(chrono::steady_clock::now() - t0).count();
      _odom_aligner->setPosePrior(IsometryType::Identity(),
                                 IsometryType::Identity(),
                                 PoseHessianType::Zero());

      _status = Tracking;
      if (_odom_aligner->getX().matrix().allFinite()) {
        _pose_in_kf = _odom_aligner->getX();
        _floating_frame->pose_in_world = _keyframe->pose_in_world * _pose_in_kf;
        IsometryType T_delta  = pose_in_kf_prev.inverse() * _pose_in_kf;
        _motion_model->doUpdate(T_delta, _odom_aligner->getPoseHessian());
      } else {
        cerr << "SOLVER DIVERGED at frame " << _frame_idx << "\n";
        throw std::runtime_error("divergence");
      }

      if (_odom_aligner->alignerType() == AlignerCTICP) {
        _floating_frame->velocity = _odom_aligner->velocities();
        pushEvent(std::make_shared<EventVelocity>(_floating_frame->ts,
                                                  _floating_frame->frame_count,
                                                  _floating_frame->velocity));
        _floating_frame->applyVelocity();
        compressed_leaves = _floating_frame->tree->extractCompressedLeaves();
        pushEvent(std::make_shared<EventFrameAdded>(_floating_frame->ts,
                                                    frameCount(),
                                                    (_keyframe ? _keyframe->ref() : -1),
                                                    _pose_in_kf, compressed_leaves,
                                                    FrameAddedDeskew,
                                                    src_->topic, src_->bag_offset, src_->stamp_ns));
      }

      pushEvent(std::make_shared<EventOdometry>(_floating_frame->ts,
                                                _pose_in_kf,
                                                _floating_frame->pose_in_world,
                                                _odom_aligner->stats));
      ++_frame_idx;
    }

    template <typename Base_>
    bool TrackerProc_<Base_>::shouldSwitchKeyframe(const IsometryType& X,
                                               const ICPStats& stats) const {
      return stats.inlierRatio() < _tracker_params.min_inlier_frac
        || X.translation().norm() > _tracker_params.max_kf_trans
        || fabs(GeometryTraits::getAngle(X.linear())) > _tracker_params.max_kf_rot_rad;
    }

  } // namespace slam
} // namespace kd_slam
