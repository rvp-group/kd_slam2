#include "eval.h"
#include <iostream>

// traj_compare <gt.tum> <estimate.tum> [output_dir]
int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: traj_compare <gt.tum> <estimate.tum> [output_dir]\n";
    return 1;
  }
  const std::string output_path = (argc > 3) ? argv[3] : ".";
  TrajectoryEvaluator evaluator;
  bool ok = evaluator.init(argv[1], argv[2], std::cerr);
  if (! ok) {
    cerr << "Error loading trajectories" << endl;
  }
  evaluator.disable_bench=true;
  evaluator.interpolate_on=true;
  ok=evaluator.eval(std::cerr); 
  if (!ok) return 1;
  evaluator.dump(std::cerr, output_path);
  return 0;
}
