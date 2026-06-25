#include "map_frame_collector_.h"

namespace kd_slam{
  namespace map {

    template <typename NodeType_>
    MapFrameCollector_<NodeType_>::MapFrameCollector_(){
      registerCallback<EvAdded>([this](std::shared_ptr<EvAdded> ev) {
        _frames[ev->count]=ev;
        _current=ev;
      });

      registerCallback<EvOdom>([this](std::shared_ptr<EvOdom> ev) {
        if (!_current)
          return;
        _current->pose_in_kf = ev->pose_in_kf;
      }
        );

      registerCallback<EvKF>([this](std::shared_ptr<EvKF> ev) {
        if (! _current)
          return;
        _keyframes[ev->kf_ref]=_current;
        _current->kf_ref=-1;
        _current->pose_in_kf=ev->pose_global;
      });

      registerCallback<EvReloc>([this](std::shared_ptr<EvReloc> ev) {
        if (! _current || ev->matched_ref<0)
          return;
        _current->kf_ref=ev->matched_ref;
      });
    
      registerCallback<EvFact>([this](std::shared_ptr<EvFact> ev) {
        _factors.push_back(ev);
        _factors_map[ev->key]=ev;
      });

      registerCallback<EvFacCh>([this](std::shared_ptr<EvFacCh> ev) {
        _factors_map[ev->key]->is_enabled=ev->enabled;
      });

      registerCallback<EvVel>([this](std::shared_ptr<EvVel> ev) {
        if (_current)
          _velocities[ev->count].push_back(ev);
      });

      registerCallback<EvPGO>([this](std::shared_ptr<EvPGO> ev) {
        for (auto& [ref, global_pose] : ev->poses_after) {
          auto it = _keyframes.find(ref);
          if (it != _keyframes.end())
            it->second->pose_in_kf = global_pose;
        }
      });

      registerCallback<EvProc>([this](std::shared_ptr<EvProc> ev) {
        if (!_current)
          return;
        _current->compressed_leaves.clear();
        _current->compressed_leaves.shrink_to_fit();
      });
    }

  }
}
