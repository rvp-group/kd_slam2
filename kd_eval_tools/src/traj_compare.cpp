#include "eval.h"
#include <iostream>

// traj_compare <gt.tum> <estimate.tum> [output_dir]
int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: traj_compare <gt.tum> <estimate.tum> [output_dir]\n";
    return 1;
  }
  const std::string output_path = (argc > 3) ? argv[3] : ".";

  Eigen::Vector2f ate, rpe;
  bool ok = eval(ate, rpe,
                 argv[1], argv[2], output_path,
                 std::cerr,
                 true,   // disable gnuplot benchmark
                 true);  // interpolate
  if (!ok) return 1;
  std::cerr << "ATE  rot(deg) trans(m): " << ate.transpose() << "\n";
  std::cerr << "RPE  rot(deg) trans(%): " << rpe.transpose() << "\n";
  return 0;
}
