#include "kd_tum_writer_.h"
#include "kd_slam/map/map_frame_collector_impl_.h"
#include "kd_slam/d2/typedefs.h"
#include "kd_slam/d3/typedefs.h"
#include "tum_io.h"

namespace kd_slam {

  template <typename NodeType_>
  KDTumWriter_<NodeType_>::KDTumWriter_():
    BaseType(){}

  template <typename NodeType_>
  void KDTumWriter_<NodeType_>::setDone()  {
    
    if (! output_stream && ! output_stream->good()) {
      cerr << "TumWriter done, no output selected" << endl;
      return;
    }
    cerr << "TumWriter done, writing output" << endl;
    int n_frames=0;
    for (auto& [count, frame]: _frames) {
      int kf_ref=frame->kf_ref;
      const IsometryType& pose_in_kf=frame->pose_in_kf;
      IsometryType global_pose;
      if (kf_ref>=0) { // is keyframe
        global_pose=_keyframes[kf_ref]->pose_in_kf*pose_in_kf;
      } else {
        global_pose=pose_in_kf;
      }
      writeTUM(*output_stream, frame->ts, global_pose);
      cerr << "\rwriting " << n_frames << "/" << _frames.size();
      ++n_frames;
    }
    cerr << endl << "Done" << endl;

  }

  template struct KDTumWriter_<kd_slam::d2::NodeType>;
  template struct KDTumWriter_<kd_slam::d3::NodeType>;
}
