#pragma once
namespace kd_slam {
  namespace map {
    using namespace kd_slam::frame;
    using namespace std;
    
    
    template <typename NodeType_>
    void MapOwner_<NodeType_>::saveFrames() {
      for (auto [ref, frame_]: _map->frames()) {
        auto frame = std::dynamic_pointer_cast<typename MapType::Frame>(frame_);
        if (! frame)
          continue;
        frame->save();
      }
    }

    template <typename NodeType_>
    void MapOwner_<NodeType_>::restoreFrames() {
      for (auto [ref, frame_]: _map->frames()) {
        auto frame = std::dynamic_pointer_cast<Frame>(frame_);
        if (! frame)
          continue;
        frame->restore();
      }
    }

    template <typename Node_>
    void MapOwner_<Node_>::pushFactorUpdateEvents() {
      for(auto f:_map->factors()){
        pushEvent(std::make_shared<EventFactorChanged>((size_t)f.get(), f->is_enabled));
      }
    }

    template <typename Node_>
    void MapOwner_<Node_>::pushFactorEvents() {
      ICPStats empty{};
      for (auto fac_ : _map->factors()) {
        if (auto fac = std::dynamic_pointer_cast<PGOFactor>(fac_)) {
          pushEvent(std::make_shared<EventFactorAdded>(
            0, fac->from_ref, fac->to_ref,
            fac->Z_from_to, fac->omega_from_to,
            empty, fac->type, (size_t)fac.get()));
        } else if (auto fac = std::dynamic_pointer_cast<MultiViewICPFactor>(fac_)) {
          if (!fac->is_enabled) continue;
          pushEvent(std::make_shared<EventFactorAdded>(
            0, fac->from_ref, fac->to_ref,
            IsometryType::Identity(), PoseHessianType::Zero(),
            empty, fac->type, (size_t)fac.get()));
        } else if (auto fac = std::dynamic_pointer_cast<MultiViewCTICPFactor>(fac_)) {
          if (!fac->is_enabled) continue;
          pushEvent(std::make_shared<EventFactorAdded>(
            0, fac->from_ref, fac->to_ref,
            IsometryType::Identity(), PoseHessianType::Zero(),
            empty, fac->type, (size_t)fac.get()));
        }
      }
    }

    template <typename Node_>
    void MapOwner_<Node_>::pushFrameEvents() {
      for (auto& [ref, base_frame] : _map->frames()) {
        auto frame = std::dynamic_pointer_cast<Frame>(base_frame);
        if (!frame || !frame->tree) continue;
        auto leaves = frame->tree->extractCompressedLeaves(frame->velocity);
        pushEvent(std::make_shared<EventFrameAdded>(
                                                    frame->ts,
                                                    frame->frame_count,
                                                    -1,
                                                    frame->pose_in_world,
                                                    leaves,
                                                    FrameAddedCloud));
        ICPStats empty{};
        pushEvent(std::make_shared<EventKeyframeAdded>(
                                                       frame->ts,
                                                       ref,
                                                       -1,
                                                       frame->pose_in_world,
                                                       empty));
      }
    }

    
    template <typename Node_>
    void MapOwner_<Node_>::pushOptimizeEvent(double ts, const std::string& message) {
      // if (_optimizer) {
      //   _optimizer->sync();
      // }
      if (! event_sinks.empty()) {
        std::map<int, IsometryType> poses;
        for (auto& [ref, fb] : _map->frames())
          poses[ref] = std::static_pointer_cast<Frame>(fb)->pose_in_world;
        // we shoot the optimizer string only when in off-line mode (ts=0);
        pushEvent(std::make_shared<EventPGOOptimized>(ts, std::move(poses), message));
      }
    }
  
    template <typename Node_>
    void MapOwner_<Node_>::pushFrameProcessedEvent(double ts, KDStatus _status) {
      pushEvent(std::make_shared<EventFrameProcessed>(_status,-1,-1,-1,-1));
    }

    
    template <typename Node_>
    void MapOwner_<Node_>::applyKFVelocities() {
      for (auto [ref, frame_]: _map->frames() ){
        auto frame=std::dynamic_pointer_cast<Frame>(frame_);
        if (! frame || !frame->tree)
          continue;
        if (! frame->_temp_tree) {
          frame->velocity.setZero();
        }
        frame->tree->applyVelocitiesInPlace(frame->velocity, false);
      }
    }

  }
}
