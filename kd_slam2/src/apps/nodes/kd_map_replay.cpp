// kd_map_replay: reload a .kf state file + a BA-optimized _map.boss,
// patch keyframe global poses from the boss variables, replay to TUM.
// No ICP, no bag, no SLAM computation needed.

#include <iostream>
#include <fstream>
#include <string>
#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/system_utils.h"
#include <srrg_solver/solver_core/factor_graph.h>
#include "kd_slam/d2/typedefs.h"
#include "kd_slam/d3/typedefs.h"
#include "kd_slam/map/map_traits_.h"
#include "kd_io/kd_state_reader_.h"
#include "kd_io/kd_tum_writer_.h"
#include "instances_2d.h"
#include "instances_3d.h"

using namespace kd_slam;
using namespace kd_slam::map;

const char* banner[] = {
  "kd_map_replay: patch SLAM state with BA-corrected poses and write TUM",
  "  Loads .kf + _map.boss, overrides keyframe global poses from boss variables,",
  "  then replays the full trajectory to a TUM file. No ICP or bag needed.",
  nullptr
};


// ---- KDMapReplay_ ----------------------------------------------------------
// Extends KDSLAMStateReader_ with the ability to patch keyframe poses from a
// BA-optimized factor graph (boss file) before replaying.

template <typename NodeType_>
struct KDMapReplay_ :
  public KDStateReplay_<NodeType_> {
  using Base          = KDStateReader_<NodeType_>;
  using IsometryType  = typename NodeType_::IsometryType;
  using SolverVarType = typename MapTraits_<NodeType_>::SolverVariableType;

  using Base::_keyframe_records;
  using Base::_frame_records;

  // Read boss_file, find pose variables, overwrite matching keyframe global poses.
  // Also patches the corresponding FRAME_N entry (kf_ref == -1) so both records
  // carry consistent global poses.
  void patchFromBoss(const std::string& boss_file) {
    using namespace srrg2_solver;
    auto graph = FactorGraph::read(boss_file);
    if (!graph) {
      std::cerr << "KDMapReplay: cannot read " << boss_file << "\n";
      return;
    }
    int n_patched = 0;
    for (auto [id, var] : graph->variables()) {
      auto v = dynamic_cast<SolverVarType*>(var);
      if (!v) continue;
      int ref = v->graphId();
      auto kf_it = _keyframe_records.find(ref);
      if (kf_it == _keyframe_records.end()) continue;
      auto& kf       = kf_it->second;
      kf.pose_in_kf  = v->estimate();
      // keep the FRAME_N entry in sync (skipped during replay but kept consistent)
      auto fr_it = _frame_records.find(kf.count);
      if (fr_it != _frame_records.end())
        fr_it->second.pose_in_kf = IsometryType::Identity();
      ++n_patched;
    }
    std::cerr << "KDMapReplay: patched " << n_patched << " / "
              << _keyframe_records.size() << " keyframe poses\n";
  }
  
};

// ---- run -------------------------------------------------------------------

template <typename NodeType_>
static int run(const std::string& kf_file,
               const std::string& boss_file,
               const std::string& tum_file) {
  using TUMWriterType = KDTumWriter_<NodeType_>;

  KDMapReplay_<NodeType_> reader;
  if (!reader.open(kf_file))
    return 1;

  reader.patchFromBoss(boss_file);

  std::ofstream os_tum(tum_file);
  if (!os_tum.good()) {
    std::cerr << "KDMapReplay: cannot open output " << tum_file << "\n";
    return 1;
  }

  auto tum_writer = std::make_shared<TUMWriterType>();
  tum_writer->output_stream = &os_tum;
  reader.event_sinks.push_back(tum_writer);

  std::cerr << "KDMapReplay: replaying to " << tum_file << "\n";
  reader.replay();
  return 0;
}

// ---- main ------------------------------------------------------------------

int main(int argc, char** argv) {
  srrg2_core::srrgInit(argc, argv);
  kd_slam_registerTypes2D();
  kd_slam_registerTypes3D();

  srrg2_core::ParseCommandLine cmd(argv, banner);
  srrg2_core::ArgumentString a_kf  (&cmd, "is", "input-state", "input state file (.kf)",                "");
  srrg2_core::ArgumentString a_map (&cmd, "im", "input-map",   "map prefix (loads <prefix>_map.boss)",  "");
  srrg2_core::ArgumentString a_tum (&cmd, "ot", "output-tum",  "output TUM trajectory file",            "");
  srrg2_core::ArgumentFlag   a_2d  (&cmd, "2",  "two-dim",     "use 2D pipeline");
  cmd.parse();

  if (!a_kf.isSet() || !a_map.isSet() || !a_tum.isSet()) {
    std::cerr << "ERROR: need -is <state.kf> -im <map_prefix> -ot <out.tum>\n";
    cmd.summary();
    return 1;
  }

  const std::string boss_file = a_map.value() + "_map.boss";

  if (a_2d.isSet())
    return run<d2::NodeType>(a_kf.value(), boss_file, a_tum.value());
  else
    return run<d3::NodeType>(a_kf.value(), boss_file, a_tum.value());
}
