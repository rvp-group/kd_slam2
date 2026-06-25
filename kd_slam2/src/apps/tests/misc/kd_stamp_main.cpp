#include "kd_slam/utils/time_utils.h"
using namespace kd_slam::utils;
#include <iostream>
#include <sys/time.h>
#include <fstream>
#include <vector>
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

using GeometryTraits=kd_slam::geometry::GeometryTraits_<VectorType::RowsAtCompileTime, Scalar>;

template <typename MyTree, typename GoodLeafPredicate>
void printStamps(const MyTree& tree, const GoodLeafPredicate& isGoodLeaf) {
  using namespace std;
  if constexpr (MyTree::Traits::HasTimestamp) {
    std::vector<size_t> leaves_indices;
    tree.selectNodeIndices(leaves_indices, isGoodLeaf);
    for (const auto& i : leaves_indices) {
      const auto& n = tree._nodes_storage[i];
      cout << "leaf: " << i << " size: " << n.size()
           << " mean: " << n._mean.transpose()
           << " ts_mean: " << n._dts_mean
           << " ts_cov: " << sqrt(n._dts_covariance) << endl;
    }
  }
}

const char* kd_stamp_banner[] = {
  "kd-stamp test: timestamp propagation in PCA-based KD-trees",
  0};

int main(int argc, char** argv) {
  using namespace std;
  ParseCommandLine cmd_line(argv, kd_stamp_banner);
  cmd_line.parse();

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
  flat_policy.leaf_density         = 10;
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
  cerr << "good_leaves: " << fixed_tree_cpu.countNodes(isGoodLeaf) << endl << endl;

  cerr << "build_tree[CPU] (moving)" << endl;
  t_start = getNow();
  TreeCPUType moving_tree_cpu(moving_container, flat_policy);
  cerr << "Time[ms]: " << getDurationMs(t_start) << endl;
  cerr << "pruning (moving)" << endl;
  moving_tree_cpu.prune(isGoodLeaf);
  cerr << "good_leaves: " << moving_tree_cpu.countNodes(isGoodLeaf) << endl << endl;

  printStamps(fixed_tree_cpu, isGoodLeaf);
  printStamps(moving_tree_cpu, isGoodLeaf);
}
