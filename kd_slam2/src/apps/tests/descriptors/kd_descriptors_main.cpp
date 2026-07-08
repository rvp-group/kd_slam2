#include "kd_slam/utils/time_utils.h"
using namespace kd_slam::utils;
#include <iostream>
#include <sys/time.h>
#include <fstream>
#include <list>
#include "srrg_system_utils/parse_command_line.h"
using namespace srrg2_core;


#ifdef _TWO_DIMENSIONS_
#include "kd_slam/d2/typedefs.h"
using namespace kd_slam;
using namespace kd_slam::d2;
#endif

#ifdef _THREE_DIMENSIONS_
#include "kd_slam/d3/typedefs.h"
using namespace kd_slam;
using namespace kd_slam::d3;
#endif

#include "kd_slam/descriptor/descriptor_.h"
#include "kd_slam/descriptor/descriptor_impl_.h"
#include "kd_slam/descriptor/extractor_.h"
#include "kd_slam/descriptor/extractor_impl_.h"
using namespace kd_slam::descriptor;

using GeometryTraits=kd_slam::geometry::GeometryTraits_<VectorType::RowsAtCompileTime, Scalar>;

struct CompareResult {
  int    f_canon;
  int    m_canon;
  Scalar distance;
};

const char* kd_descriptors_banner[] = {
  "kd-descriptors test: holistic descriptor matching with PCA KD-trees",
  0};

int main(int argc, char** argv) {
  using namespace std;
  ParseCommandLine cmd_line(argv, kd_descriptors_banner);

  size_t n_items            = 5;
  Scalar resolution         = 0.05;
  Scalar range              = 20.f;
  int    min_points_in_leaf = 10;
  int    max_points_in_leaf = 50;
  float  min_normal_incidence = 0.1;

  IsGoodLeafPredicate_<NodeType> isGoodLeaf(min_points_in_leaf, min_normal_incidence);

  FlatLeafPolicy_<NodeType> flat_policy;
  flat_policy.min_points           = min_points_in_leaf;
  flat_policy.max_points           = max_points_in_leaf;
  flat_policy.min_normal_incidence = min_normal_incidence;
  flat_policy.leaf_density         = 50;
  flat_policy.propagate_normal     = true;
  flat_policy.parallel_level       = 3;

  cerr << "generate samples (fixed)" << endl;
  auto t_start = getNow();
  PointsVectorType fixed_container;
  generateScene(fixed_container, range, resolution, n_items);

  PerturbationPoseType x_gt;
  if constexpr (PerturbationPoseType::RowsAtCompileTime == 3)
    x_gt << 1, 2, 0.3;
  if constexpr (PerturbationPoseType::RowsAtCompileTime == 6)
    x_gt << -10, -20, -10, 0.1, 0.6, 0.1;

  cerr << "generate measurements (moving)" << endl;
  IsometryType X_gt     = GeometryTraits::expmap(x_gt);
  IsometryType inv_X_gt = X_gt.inverse();

  PointsVectorType moving_container = fixed_container;
  for (auto& p : moving_container)
    PointTraits::coordinates(p) = inv_X_gt * PointTraits::coordinates(p);

  cerr << "transform[gt]: " << endl << X_gt.matrix() << endl;
  cerr << "num_points: " << fixed_container.size() << endl;
  cerr << "Time[ms]: " << getDurationMs(t_start) << endl << endl;

  cerr << "build_tree[CPU] (fixed)" << endl;
  t_start = getNow();
  TreeCPUType fixed_tree_cpu(fixed_container, flat_policy);
  cerr << "Time[ms]: " << getDurationMs(t_start) << endl;
  cerr << "num_nodes: " << fixed_tree_cpu.numNodes() << endl;
  cerr << "good_leaves: " << fixed_tree_cpu.countNodes(isGoodLeaf) << endl << endl;

  cerr << "pruning (fixed)" << endl;
  t_start = getNow();
  fixed_tree_cpu.prune(isGoodLeaf);
  cerr << "Time[ms]: " << getDurationMs(t_start) << endl;
  cerr << "good_leaves: " << fixed_tree_cpu.countNodes(isGoodLeaf) << endl << endl;

  static constexpr int DescriptorLevel = 4;
  using DescriptorExtractor = Extractor_<TreeCPUType, DescriptorLevel>;
  using DescriptorType      = typename DescriptorExtractor::DescriptorType;

  cerr << "Computing descriptors|"
       << " level: " << DescriptorExtractor::Level
       << " size: "  << DescriptorExtractor::Size
       << " canonizations: " << DescriptorExtractor::NumAxesCanonizations << endl;

  // extract can0 only, build all canonizations via remap
  DescriptorType fixed_desc0  = DescriptorExtractor::extract(fixed_tree_cpu, 0);
  auto           fixed_qdesc  = fixed_desc0.buildQDescriptors();

  // consistency check: remap(c).toPose() must match explicit applyCanonization + root_mean
  cerr << "\n--- remap consistency check (fixed) ---" << endl;
  bool remap_ok = true;
  for (int c = 0; c < DescriptorExtractor::NumAxesCanonizations; ++c) {
    IsometryType from_remap = fixed_qdesc.des[c].toPose();
    IsometryType from_tree;
    from_tree.linear()      = DescriptorType::applyCanonization(fixed_tree_cpu.root_eigenvectors, c);
    from_tree.translation() = fixed_tree_cpu.root_mean;
    Scalar err = (from_remap.matrix() - from_tree.matrix()).squaredNorm();
    cerr << "  can " << c << " err: " << err;
    if (err > Scalar(1e-6)) {
      cerr << " FAIL";
      remap_ok = false;
    }
    cerr << endl;
  }
  if (remap_ok)
    cerr << "remap consistency: OK" << endl;
  else
    cerr << "remap consistency: FAILED -- remap is broken" << endl;

  cerr << "build_tree[CPU] (moving)" << endl;
  t_start = getNow();
  TreeCPUType moving_tree_cpu(moving_container, flat_policy);
  cerr << "Time[ms]: " << getDurationMs(t_start) << endl;
  cerr << "good_leaves: " << moving_tree_cpu.countNodes(isGoodLeaf) << endl << endl;

  cerr << "pruning (moving)" << endl;
  t_start = getNow();
  moving_tree_cpu.prune(isGoodLeaf);
  cerr << "Time[ms]: " << getDurationMs(t_start) << endl;
  cerr << "good_leaves: " << moving_tree_cpu.countNodes(isGoodLeaf) << endl << endl;

  DescriptorType moving_desc0 = DescriptorExtractor::extract(moving_tree_cpu, 0);
  auto           moving_qdesc = moving_desc0.buildQDescriptors();

  Scalar ok_threshold = 10;
  std::list<CompareResult> matches;

  cerr << "descriptor compare" << endl;
  for (int f = 0; f < DescriptorExtractor::NumAxesCanonizations; ++f) {
    cerr << "f: " << f << "\t";
    for (int m = 0; m < DescriptorExtractor::NumAxesCanonizations; ++m) {
      Scalar compval = fixed_qdesc.des[f].medCompare(moving_qdesc.des[m], 0);
      if (compval < ok_threshold)
        matches.push_back({f, m, compval});
      cerr << "m: " << m << " " << compval << "\t";
    }
    cerr << endl;
  }

  for (const auto& m : matches) {
    IsometryType p_fixed  = fixed_qdesc.des[m.f_canon].toPose();
    IsometryType p_moving = moving_qdesc.des[m.m_canon].toPose();
    IsometryType guess    = p_fixed * p_moving.inverse();
    cerr << "\nmatch| f_can: " << m.f_canon
         << " m_can: " << m.m_canon
         << " dist: " << m.distance
         << " err: " << (X_gt.matrix() - guess.matrix()).squaredNorm();
  }
  cerr << endl;

  return 0;
}
