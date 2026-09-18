#include "eval.h"
#include <iostream>
#include <sstream>
using namespace std;

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
  cout << "Batch evaluation on" << endl;
  while(cin.good()){
    char buf[1024];
    cin.getline(buf, 1024);
    istringstream is(buf);
    std::string cmd;
    is >> cmd;
    if (cmd=="eval") {
      double t_min=-1, t_max=std::numeric_limits<double>::max();
      is>>t_min;
      if (!is)
        t_min=-1;
      is>>t_max;
      if (!is)
        t_max=std::numeric_limits<double>::max();
      cerr << "evaluating range [ " << t_min << " -- " << t_max << "]" << endl;
      evaluator.eval(cerr,t_min, t_max);
    } else if (cmd=="dump") {
      evaluator.dump(cerr, output_path);
    } else if (cmd=="stretch") {
      double s;
      is>>s;
      if (!is)
        s=std::numeric_limits<double>::max();
      evaluator.align_stretch=s;
      cerr << "align_stretch: " << evaluator.align_stretch << endl;
    } else if (cmd=="quit") {
      break;
    }
  }
  return 0;
}
