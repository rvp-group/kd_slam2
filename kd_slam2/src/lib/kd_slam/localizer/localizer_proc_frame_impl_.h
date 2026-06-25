#pragma once
#include <queue>
namespace kd_slam {
  namespace slam {
    using namespace kd_slam::frame;

    template <typename Node_>
    void Localizer_<Node_>::addFrame(const FrameTreePtr& src_) {
      using namespace std;
      syncParams();

      if (!canCompute())
        throw std::runtime_error("Localizer| addFrame, no compute configured");
      if (!src_)
        return;
      Runnable::waitRun();

      std::vector<VectorType> compressed_leaves;
      if (src_->tree)
        compressed_leaves = src_->tree->extractCompressedLeaves();
      auto src = FrameTree::toCompute(src_, isGPU());

      auto pose_global = poseGlobal();
      _floating_frame = makeFrame(src);
      _floating_frame->pose_in_world = pose_global;
      pushEvent(std::make_shared<EventFrameAdded>(_floating_frame->ts,
                                                  frameCount(),
                                                  (_keyframe ? _keyframe->ref() : -1),
                                                  _pose_in_kf,
                                                  compressed_leaves,
                                                  FrameAddedCloud,
                                                  src_->topic,
                                                  src_->bag_offset,
                                                  src_->stamp_ns));

      if (!_keyframe) {
        for (auto& [ref, fb] : _map->frames()) {
          auto f = std::dynamic_pointer_cast<Frame>(fb);
          if (!f) continue;
          if (!_keyframe || ref < _keyframe->ref())
            _keyframe = f;
        }
        if (!_keyframe) {
          _status = Lost;
          pushEvent(std::make_shared<EventFrameProcessed>(_status, 0, 0, 0, 0));
          return;
        }
      }

      auto t0 = std::chrono::steady_clock::now();
      int retval = alignFrames(*_odom_aligner, _keyframe, _floating_frame, _pose_in_kf, Odometry);
      if (retval >= 0 && _odom_aligner->getX().matrix().allFinite()) {
        _pose_in_kf = _odom_aligner->getX();
        _floating_frame->pose_in_world = _keyframe->pose_in_world * _pose_in_kf;
        _status = _using_dummy_keyframe ? Lost : Tracking;
      } else {
        cerr << "LOCALIZE| solver diverged at frame " << _frame_idx << "\n";
        retval = -1;
        _status = Lost;
      }
      const auto& stats = _odom_aligner->stats;
      double t_align = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

      if (retval >= 0) {
        if (_odom_aligner->alignerType() == AlignerCTICP) {
          pushEvent(std::make_shared<EventVelocity>(_floating_frame->ts,
                                                    _floating_frame->frame_count,
                                                    _odom_aligner->velocities()));
          _floating_frame->tree->applyVelocitiesInPlace(_odom_aligner->velocities());
          compressed_leaves = _floating_frame->tree->extractCompressedLeaves();
          pushEvent(std::make_shared<EventFrameAdded>(_floating_frame->ts,
                                                      frameCount(),
                                                      (_keyframe ? _keyframe->ref() : -1),
                                                      _pose_in_kf,
                                                      compressed_leaves,
                                                      FrameAddedDeskew,
                                                      src_->topic,
                                                      src_->bag_offset,
                                                      src_->stamp_ns));
        }
        pushEvent(std::make_shared<EventOdometry>(_floating_frame->ts,
                                                  _pose_in_kf,
                                                  _floating_frame->pose_in_world,
                                                  stats));
      }
      ++_frame_idx;

      bool force_switch = (retval < 0);
      if (force_switch || shouldSwitchKeyframe(_pose_in_kf, stats)) {
        const IsometryType global_pose = poseGlobal();
        FramePtr bfs_root = _using_dummy_keyframe ? _last_real_keyframe : _keyframe;

        if (bfs_root) {
          std::unordered_map<int, int> visited;
          std::queue<std::pair<int, int>> bfs_queue;
          bfs_queue.push({bfs_root->ref(), 0});
          visited[bfs_root->ref()] = 0;

          while (!bfs_queue.empty()) {
            auto [cur_ref, hop] = bfs_queue.front();
            bfs_queue.pop();
            if (hop >= params.relocalize_thresholds.max_hops && params.relocalize_thresholds.max_hops >= 0)
              continue;
            for (auto& fac_base : _map->factors(cur_ref)) {
              auto* fb = fac_base.get();
              if (!fb->is_enabled) continue;
              int neighbor_ref = -1;
              if (auto* pf = dynamic_cast<PGOFactor*>(fb)) {
                if (!pf->is_valid) continue;
                neighbor_ref = (pf->from_ref == cur_ref) ? pf->to_ref : pf->from_ref;
              } else if (auto* bf = dynamic_cast<MultiViewICPFactor*>(fb)) {
                neighbor_ref = (bf->from_ref == cur_ref) ? bf->to_ref : bf->from_ref;
              } else if (auto* cf = dynamic_cast<MultiViewCTICPFactor*>(fb)) {
                neighbor_ref = (cf->from_ref == cur_ref) ? cf->to_ref : cf->from_ref;
              }
              if (neighbor_ref < 0 || visited.count(neighbor_ref)) continue;
              visited[neighbor_ref] = hop + 1;
              bfs_queue.push({neighbor_ref, hop + 1});
            }
          }

          _local_aligner->_fixed_forest.clear();
          for (auto& [candidate_ref, hop] : visited) {
            auto nb = std::dynamic_pointer_cast<Frame>(_map->frame(candidate_ref));
            if (!nb || !nb->tree) continue;
            if ((nb->pose_in_world.translation() - global_pose.translation()).norm() > params.ba_range)
              continue;
            _local_aligner->_fixed_forest[candidate_ref] =
              _local_aligner->makeFixedEntry(*nb->tree, nb->pose_in_world);
          }

          if (!_local_aligner->_fixed_forest.empty()) {
            _local_aligner->setMoving(*_floating_frame->tree);
            _local_aligner->setX(global_pose);
            int retval_forest = _local_aligner->compute();
            const IsometryType new_global = _local_aligner->getX();

            if (retval_forest >= 0 && new_global.matrix().allFinite() &&
                _local_aligner->stats.inlierRatio() > (_using_dummy_keyframe ? 0.f : stats.inlierRatio())) {

              int best_ref = -1;
              int best_inliers = 0;
              for (auto& [ref, entry] : _local_aligner->_fixed_forest) {
                if (entry->stats.num_inliers > best_inliers) {
                  best_inliers = entry->stats.num_inliers;
                  best_ref = ref;
                }
              }

              if (best_ref >= 0) {
                auto best_kf = std::dynamic_pointer_cast<Frame>(_map->frame(best_ref));
                if (best_kf) {
                  _keyframe = best_kf;
                  _floating_frame->pose_in_world = new_global;
                  _pose_in_kf = _keyframe->pose_in_world.inverse() * new_global;
                  if (_using_dummy_keyframe) {
                    _using_dummy_keyframe = false;
                    _last_real_keyframe.reset();
                    cerr << "LOCALIZE| re-hooked to map keyframe " << _keyframe->ref()
                         << " at frame " << _frame_idx << "\n";
                  }
                  _status = Relocalized;
                  ++_num_relocalizes;
                }
              }
            }
          }
          _local_aligner->_fixed_forest.clear();
        }

        if (_status != Relocalized && (_using_dummy_keyframe || force_switch)) {
          if (!_using_dummy_keyframe) {
            _last_real_keyframe = _keyframe;
            _using_dummy_keyframe = true;
            cerr << "LOCALIZE| entering dummy mode at frame " << _frame_idx << "\n";
          }
          _keyframe = _floating_frame;
          _pose_in_kf = IsometryType::Identity();
          if (retval < 0 && _odom_aligner->alignerType() == AlignerCTICP)
            _keyframe->tree->applyVelocitiesInPlace(_odom_aligner->velocities());
          _status = Lost;
        }
      }

      if (!_using_dummy_keyframe && _keyframe) {
        auto ev_reloc = std::make_shared<EventRelocalize>(_floating_frame->ts);
        ev_reloc->matched_ref = _keyframe->ref();
        for (auto& fac_base : _map->factors(_keyframe->ref())) {
          auto* fb = fac_base.get();
          if (!fb->is_enabled) continue;
          int neighbor_ref = -1;
          if (auto* pf = dynamic_cast<PGOFactor*>(fb)) {
            if (!pf->is_valid) continue;
            neighbor_ref = (pf->from_ref == _keyframe->ref()) ? pf->to_ref : pf->from_ref;
          } else if (auto* bf = dynamic_cast<MultiViewICPFactor*>(fb)) {
            neighbor_ref = (bf->from_ref == _keyframe->ref()) ? bf->to_ref : bf->from_ref;
          } else if (auto* cf = dynamic_cast<MultiViewCTICPFactor*>(fb)) {
            neighbor_ref = (cf->from_ref == _keyframe->ref()) ? cf->to_ref : cf->from_ref;
          }
          if (neighbor_ref >= 0)
            ev_reloc->local_map_refs.push_back(neighbor_ref);
        }
        ev_reloc->local_map_refs.push_back(_keyframe->ref());
        pushEvent(ev_reloc);
      }

      pushEvent(std::make_shared<EventFrameProcessed>(_status, t_align, 0, 0, 0));
    }

  } // namespace slam
} // namespace kd_slam
