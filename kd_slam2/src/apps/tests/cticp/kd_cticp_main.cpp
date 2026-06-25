#include "kd_slam/utils/time_utils.h"
using namespace kd_slam::utils;
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include "srrg_system_utils/parse_command_line.h"
using namespace srrg2_core;


#ifdef _TWO_DIMENSIONS_
#include "kd_slam/d2/typedefs.h"
#include "kd_slam/d2/scanner.h"
using namespace kd_slam;
using namespace kd_slam::d2;

#endif

#ifdef _THREE_DIMENSIONS_
#include "kd_slam/d3/typedefs.h"
#include "kd_slam/d3/scanner.h"
using namespace kd_slam;
using namespace kd_slam::d3;
#endif

#ifdef __USE_CPU__
using CTICPType  = CTICPCPUType;
using KDTreeType = TreeCPUType;
#endif
#ifdef __USE_CUDA__
using CTICPType  = CTICPCUDAType;
using KDTreeType = TreeCUDAType;
#endif

using GeometryTraits=kd_slam::geometry::GeometryTraits_<VectorType::RowsAtCompileTime, Scalar>;

const char* kd_cticp_banner[] = {
  "kd-cticp test: Continuous-Time ICP with PCA-based KD-trees",
  "press 'r' for batch runs, 'q' to quit",
  0};

int main(int argc, char** argv) {
  using namespace std;
  ParseCommandLine cmd_line(argv, kd_cticp_banner);
  ArgumentFlag gnuplot_flag(&cmd_line, "g", "gnuplot", "pipe output to gnuplot");
  cmd_line.parse();
  bool dump_gnuplot = gnuplot_flag.isSet();

  // ---- world ---------------------------------------------------------------
  vector<VectorType> world_points;
  size_t n_items    = 10;
  Scalar resolution = 0.05f;
  Scalar range      = 20.f;
  cerr << "making world" << endl;
  generateScene(world_points, range, resolution, n_items);
  cerr << "world points: " << world_points.size() << endl;

  // ---- ground-truth pose and velocities ------------------------------------
  PerturbationPoseType x_gt_vec;
  if constexpr (PerturbationPoseType::RowsAtCompileTime == 3)
    x_gt_vec << 0.1f, 0.5f, 0.2f;
  if constexpr (PerturbationPoseType::RowsAtCompileTime == 6)
    x_gt_vec << 0.1f, 2.f, 0.f, 0.f, 0.f, 0.4f;
  IsometryType X_gt = GeometryTraits::expmap(x_gt_vec);

  VelocityType gt_velocities;
  if constexpr (VelocityType::RowsAtCompileTime == 3)
    gt_velocities << 1.f, 0.f, 2.f;
  if constexpr (VelocityType::RowsAtCompileTime == 6)
    gt_velocities << 1.f, 0.f, 0.f, 0.f, 0.f, 2.f;

  // ---- generate scans ------------------------------------------------------
  Scanner scanner;
  PointsVectorType scan_gt, scan_skewed;

  cerr << "making GT scan (identity pose, zero velocity)" << endl;
  scanner.makeScan(scan_gt, world_points,
                   IsometryType::Identity(), PerturbationPoseType::Zero());
  cerr << "GT scan points: " << scan_gt.size() << endl;

  cerr << "making skewed scan (X_gt + velocities)" << endl;
  scanner.makeScan(scan_skewed, world_points, X_gt, gt_velocities);
  cerr << "skewed scan points: " << scan_skewed.size() << endl;

  if (dump_gnuplot) {
    ofstream os("scan_gt.txt");
    for (const auto& p : scan_gt)
      os << PointTraits::coordinates(p).transpose() << "\n";
    ofstream os2("scan_skewed.txt");
    for (const auto& p : scan_skewed)
      os2 << PointTraits::coordinates(p).transpose() << "\n";
  }

  // ---- KD-tree setup -------------------------------------------------------
  int   min_pts = 10, max_pts = 50;
  float min_inc = 0.1f;

  IsGoodLeafPredicate_<NodeType> isGoodLeaf(min_pts, min_inc);

  FlatLeafPolicy_<NodeType> policy;
  policy.min_points           = min_pts;
  policy.max_points           = max_pts;
  policy.min_normal_incidence = min_inc;
  policy.leaf_density         = 100;
  policy.propagate_normal     = true;
  policy.parallel_level       = 3;

  cerr << "build + prune (gt)" << endl;
  auto t0 = getNow();
  TreeCPUType tree_gt_cpu(scan_gt, policy);
  tree_gt_cpu.prune(isGoodLeaf);
  cerr << "Time[ms]: " << getDurationMs(t0) << "  good_leaves: "
       << tree_gt_cpu.countNodes(isGoodLeaf) << endl;

  cerr << "build + prune (skewed)" << endl;
  t0 = getNow();
  TreeCPUType tree_skewed_cpu(scan_skewed, policy);
  tree_skewed_cpu.prune(isGoodLeaf);
  cerr << "Time[ms]: " << getDurationMs(t0) << "  good_leaves: "
       << tree_skewed_cpu.countNodes(isGoodLeaf) << endl;

  if (dump_gnuplot) {
    ofstream os("tree_gt.txt");      tree_gt_cpu.dumpSelectedNodes(os, isGoodLeaf);
    ofstream os2("tree_skewed.txt"); tree_skewed_cpu.dumpSelectedNodes(os2, isGoodLeaf);
  }

  KDTreeType tree_gt(tree_gt_cpu);
  KDTreeType tree_skewed(tree_skewed_cpu);

#if defined(__USE_CUDA__) && defined(_THREE_DIMENSIONS_)
  if (0) {
    auto tree_deskewed_cpu  = linearDeskew(tree_skewed_cpu, gt_velocities);
    KDTreeType tree_deskewed_cuda(tree_deskewed_cpu);
    ICPCUDAType icp_cuda;
    icp_cuda.setFixed(tree_gt);
    icp_cuda.setMoving(tree_deskewed_cuda);
    icp_cuda.moving_state.X = IsometryType::Identity();
    for (int i = 0; i < 100; ++i) icp_cuda.oneRound();
    icp_cuda.printStatus(cerr);
  }
#endif

  // ---- CT-ICP --------------------------------------------------------------
  CTICPType cticp;
  cticp.setFixed(tree_gt);
  cticp.setMoving(tree_skewed);
  cticp.moving_state.X          = IsometryType::Identity();
  cticp.moving_state.velocities = VelocityType::Zero();

  cerr << "\n--- Ground truth ---" << endl;
  cerr << "X_gt:\n"  << X_gt.matrix() << endl;
  cerr << "vel_gt: " << gt_velocities.transpose() << endl;
  cerr << "\nReady: press a key, 'r' for batch, 'q' to quit" << endl;
  if (dump_gnuplot)
    dumpGnuplot(tree_gt_cpu, tree_skewed_cpu, isGoodLeaf, cticp.moving_state.X);

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
    t0 = getNow();
    for (int i = 0; i < num_rounds; ++i)
      if (!cticp.oneRound()) { cerr << "not enough measurements" << endl; break; }
    double elapsed = getDurationMs(t0);
    cticp.printStatus(cerr);
    cerr << "iter: " << iteration_num << "  Time[ms]: " << elapsed
         << " Time/Iter: " << elapsed/num_rounds << endl;
    IsometryType dX = cticp.moving_state.X.inverse() * X_gt;
    cerr << "X recovered:\n" << cticp.moving_state.X.matrix() << endl;
    cerr << "X_gt:\n"        << X_gt.matrix() << endl;
    cerr << "pose |t_err|:  " << dX.translation().norm() << endl;
    cerr << "vel recovered: " << cticp.moving_state.velocities.transpose() << endl;
    cerr << "vel gt:        " << gt_velocities.transpose() << endl;
    cerr << "vel |error|:   " << (cticp.moving_state.velocities - gt_velocities).norm() << endl;
    if (dump_gnuplot) {
      auto tree_deskewed = linearDeskew(tree_skewed_cpu, cticp.moving_state.velocities);
      dumpGnuplot(tree_gt_cpu, tree_deskewed, isGoodLeaf, cticp.moving_state.X);
    }
    cerr << "press a key" << endl;
    ++iteration_num;
  }
}
