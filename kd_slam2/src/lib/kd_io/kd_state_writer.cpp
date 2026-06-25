#include "kd_state_writer_.h"
#include "kd_slam/map/map_frame_collector_impl_.h"
#include "kd_slam/d2/typedefs.h"
#include "kd_slam/d3/typedefs.h"
#include "tum_io.h"
#include <iomanip>

namespace kd_slam {
  using namespace std;
  

  template <typename NodeType_>
  KDStateWriter_<NodeType_>::KDStateWriter_():
    BaseType(){}

  template <typename NodeType_>
  void KDStateWriter_<NodeType_>::setDone()  {
    
    if (! output_stream && ! output_stream->good()) {
      cerr << "SLAMStateWriter done, no output selected" << endl;
      return;
    }
    cerr << "SLAMStateWriter done, writing output" << endl;
    ostream& os=*output_stream;
    cerr << "writing keyframes: ";
    for (auto& [idx, f]: _keyframes) {
      os << "KEYFRAME_" << Dim << " "
         << f->count << " "
         << idx << " "
         << std::fixed << std::setprecision(9) << f->ts << " ";
         writeIsometry(os, f->pose_in_kf);
      if (!f->topic.empty())
        os << " T: " << f->topic;
      if (f->bag_offset)
        os << " O: " << f->bag_offset;
      if (f->stamp_ns)
        os << " N: " << f->stamp_ns;
      os << endl;
      cerr <<"\r" << idx << "/" << _keyframes.size();
    }
    cerr << "Writing PGO factors: " << endl;
    int n_fact=0;
    for (auto& e: _factors) {
      if (e->type!=Odometry
          && e->type!=Relocalize
          && e->type!=Loop){
        continue;
      }
      os << "PGO_FACTOR_" << Dim << " "
         << e->key << " "
         << e->type << " "
         << e->from_ref << " "
         << e->to_ref << " ";
      writeIsometry(os, e->Z_from_to);
      os << " ";
      constexpr int HDim=NodeType_::Traits::GeometryTraits::PerturbationPoseDim;
      for (int r=0; r<HDim; ++r)
        for (int c=r; c<HDim; ++c)
          os << e->omega_from_to(r,c) << " ";
      os << endl;
      cerr <<"\r" << n_fact << "/" << _factors.size();
      ++n_fact;
    }
    cerr<< endl;
    cerr << "Writing frames: " << endl;
    int n_frames=0;
    for (auto& [count, f]: _frames) {
      os << "FRAME_" << Dim << " "
         << f->count << " "
         << f->kf_ref << " "
         << std::fixed << std::setprecision(9) << f->ts << " "
         << f->reason << " ";
      writeIsometry(os, f->pose_in_kf);
      if (!f->topic.empty())
        os << " T: " << f->topic;
      if (f->bag_offset)
        os << " O: " << f->bag_offset;
      if (f->stamp_ns)
        os << " N: " << f->stamp_ns;
      os << endl;
      cerr << "\rwriting " << n_frames << "/" << _frames.size();
      ++n_frames;
    }
    cerr << endl << "Done" << endl;
    int n_vels=0;
    cerr << "Writing Velocities" << endl;
    for (auto& [cnt, vel]: _velocities) {
      for (auto& v: vel) {
        os << "VEL_" << Dim << " " << v->count << " "
           << v->velocity.transpose() << endl;
        cerr << "\rwriting " << cnt << " " << n_vels;
        ++n_vels;
      }
    }
    cerr << "Done" << endl;
    cerr<< endl;
  }

  template struct KDStateWriter_<kd_slam::d2::NodeType>;
  template struct KDStateWriter_<kd_slam::d3::NodeType>;
}
