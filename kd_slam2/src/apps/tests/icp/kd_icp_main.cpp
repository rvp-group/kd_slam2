#include "kd_slam/utils/time_utils.h"
using namespace kd_slam::utils;
#include <iostream>
#include <sys/time.h>
#include <fstream>
#include "srrg_system_utils/parse_command_line.h"
using namespace srrg2_core;

#ifdef _TWO_DIMENSIONS_
#include "kd_slam/d2/typedefs.h"
using namespace kd_slam::d2;

using namespace kd_slam;
#endif

#ifdef _THREE_DIMENSIONS_
#include "kd_slam/d3/typedefs.h"
using namespace kd_slam::d3;
using namespace kd_slam;
#endif

using GeometryTraits=kd_slam::geometry::GeometryTraits_<VectorType::RowsAtCompileTime, Scalar>;

const char* kd_icp_banner[] = {
  "kd-icp test: ICP registration with PCA-based KD-trees",
  "press 'r' for batch runs, 'q' to quit",
  0};

int main(int argc, char** argv) {
  using namespace std;
  ParseCommandLine cmd_line(argv, kd_icp_banner);
  ArgumentFlag gnuplot_flag(&cmd_line, "g", "gnuplot", "pipe output to gnuplot");
  cmd_line.parse();
  bool dump_gnuplot = gnuplot_flag.isSet();

  size_t n_items            = 10;
  Scalar resolution         = 0.05;
  Scalar range              = 20.f;
  int    min_points_in_leaf = 10;
  int    max_points_in_leaf = 50;
  float  min_normal_incidence = 0.1;

  IsLeafPredicate_<NodeType>     isLeaf;
  IsGoodLeafPredicate_<NodeType> isGoodLeaf(min_points_in_leaf, min_normal_incidence);

  FlatLeafPolicy_<NodeType> flat_policy;
  flat_policy.min_points           = min_points_in_leaf;
  flat_policy.max_points           = max_points_in_leaf;
  flat_policy.min_normal_incidence = min_normal_incidence;
  flat_policy.leaf_density         = 100;
  flat_policy.propagate_normal     = true;
  flat_policy.parallel_level       = 3;

  cerr << "generate samples (fixed)" << endl;
  auto t_start = getNow();
  PointsVectorType fixed_container;
  generateScene(fixed_container, range, resolution, n_items);
  assignTimeStamps<PointTraits>(fixed_container);

  PerturbationPoseType x_gt;
  if constexpr (PerturbationPoseType::RowsAtCompileTime == 3)
    x_gt << 1, 2, 0.3;
  if constexpr (PerturbationPoseType::RowsAtCompileTime == 6)
    x_gt << 1, 2, 1, 0.1, 0.3, 0.1;

  cerr << "generate measurements (moving)" << endl;
  IsometryType X_gt     = GeometryTraits::expmap(x_gt);
  IsometryType inv_X_gt = X_gt.inverse();

  PointsVectorType moving_container = fixed_container;
  for (auto& p : moving_container)
    PointTraits::coordinates(p) = inv_X_gt * PointTraits::coordinates(p);
  assignTimeStamps<PointTraits>(moving_container);

  cerr << "transform[gt]: " << endl << X_gt.matrix() << endl;
  cerr << "num_points: " << fixed_container.size() << endl;
  cerr << "Time[ms]: " << getDurationMs(t_start) << endl << endl;

  cerr << "build_tree[CPU] (fixed)" << endl;
  t_start = getNow();
  TreeCPUType fixed_tree_cpu(fixed_container, flat_policy);
  cerr << "Time[ms]: " << getDurationMs(t_start) << endl;
  cerr << "num_nodes: " << fixed_tree_cpu.numNodes() << endl;
  cerr << "num_leaves: " << fixed_tree_cpu.countNodes(isLeaf) << endl;
  cerr << "good_leaves: " << fixed_tree_cpu.countNodes(isGoodLeaf) << endl << endl;

  cerr << "pruning (fixed)" << endl;
  t_start = getNow();
  fixed_tree_cpu.prune(isGoodLeaf);
  cerr << "Time[ms]: " << getDurationMs(t_start) << endl;
  cerr << "num_nodes: " << fixed_tree_cpu.numNodes() << endl;
  cerr << "good_leaves: " << fixed_tree_cpu.countNodes(isGoodLeaf) << endl << endl;

  cerr << "build_tree[CPU] (moving)" << endl;
  t_start = getNow();
  TreeCPUType moving_tree_cpu(moving_container, flat_policy);
  cerr << "Time[ms]: " << getDurationMs(t_start) << endl;
  cerr << "pruning (moving)" << endl;
  moving_tree_cpu.prune(isGoodLeaf);
  cerr << "good_leaves: " << moving_tree_cpu.countNodes(isGoodLeaf) << endl << endl;

#ifdef __USE_CUDA__
  cerr << "build_trees[CUDA] (fixed, moving)" << endl;
  t_start = getNow();
  TreeCUDAType fixed_tree_cuda(fixed_tree_cpu);
  TreeCUDAType moving_tree_cuda(moving_tree_cpu);
  cerr << "Time[ms]: " << getDurationMs(t_start) << endl << endl;
  TreeCUDAType& fixed_tree(fixed_tree_cuda);
  TreeCUDAType& moving_tree(moving_tree_cuda);
  ICPCUDAType icp;
#endif

#ifdef __USE_CPU__
  TreeCPUType& fixed_tree(fixed_tree_cpu);
  TreeCPUType& moving_tree(moving_tree_cpu);
  ICPCPUType icp;
#endif

  cerr << "initializing icp" << endl;
  t_start = getNow();
  icp.setFixed(fixed_tree);
  icp.setMoving(moving_tree);
  icp.moving_state.X.setIdentity();
  cerr << "Time[ms]: " << getDurationMs(t_start) << endl << endl;

  cerr << "Ready: press a key, 'r' for running a batch of iterations" << endl;
  if (dump_gnuplot)
    dumpGnuplot(fixed_tree_cpu, moving_tree_cpu, isGoodLeaf, icp.moving_state.X);

  int iteration_num = 0;
  while (1) {
    char c;
    cin >> c;
    int num_rounds = 1;
    if (c == 'r') {
      cerr << "insert number of rounds" << endl;
      cin >> num_rounds;
      num_rounds = abs(num_rounds);
    } else if (c == 'q') {
      return 0;
    }
    auto t0 = getNow();
    for (int i = 0; i < num_rounds; ++i)
      if (!icp.oneRound()) { cerr << "not enough measurements" << endl; break; }
    double elapsed = getDurationMs(t0);
    icp.printStatus(cerr);
    cerr << "iter: " << iteration_num << "  Time[ms]: " << elapsed
         << " Time/Iter: " << elapsed/num_rounds << endl;
    IsometryType dX = icp.moving_state.X.inverse() * X_gt;
    cerr << "X recovered:\n" << icp.moving_state.X.matrix() << endl;
    cerr << "X_gt:\n"        << X_gt.matrix() << endl;
    cerr << "pose |t_err|:  " << dX.translation().norm() << endl;
    if (dump_gnuplot)
      dumpGnuplot(fixed_tree_cpu, moving_tree_cpu, isGoodLeaf, icp.moving_state.X);
    cerr << "press a key" << endl;
    ++iteration_num;
  }
}
