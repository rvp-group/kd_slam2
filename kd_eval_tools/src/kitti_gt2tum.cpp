#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// kitti_gt2tum <poses.txt> <times.txt> <output.tum> [calib.txt [frame]]
//   poses.txt : KITTI ground truth (N lines, 12 floats, row-major 3x4 RT, camera frame)
//   times.txt : KITTI timestamps (N lines, seconds)
//   output    : TUM format (timestamp tx ty tz qx qy qz qw)
//   calib.txt : optional calibration file
//   frame     : key to look up in calib.txt (default: Tr, velodyne->camera)
//               poses are expressed in that sensor frame: T_out = T_frame_inv * T_cam * T_frame

static Eigen::Isometry3f parse3x4(std::istringstream& ss) {
  Eigen::Matrix3f R;
  Eigen::Vector3f t;
  ss >> R(0,0) >> R(0,1) >> R(0,2) >> t(0)
     >> R(1,0) >> R(1,1) >> R(1,2) >> t(1)
     >> R(2,0) >> R(2,1) >> R(2,2) >> t(2);
  Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
  T.linear()      = R;
  T.translation() = t;
  return T;
}

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: kitti_gt2tum <poses.txt> <times.txt> <output.tum> [calib.txt [frame]]\n"
              << "  frame: key in calib.txt to express poses in (default: Tr)\n";
    return 1;
  }

  std::ifstream fp(argv[1]), ft(argv[2]);
  if (!fp.good()) { std::cerr << "cannot open: " << argv[1] << "\n"; return 1; }
  if (!ft.good()) { std::cerr << "cannot open: " << argv[2] << "\n"; return 1; }

  Eigen::Isometry3f Tf     = Eigen::Isometry3f::Identity();
  Eigen::Isometry3f Tf_inv = Eigen::Isometry3f::Identity();
  bool have_calib = false;
  if (argc > 4) {
    const std::string frame_key = (argc > 5) ? std::string(argv[5]) + ":" : "Tr:";
    std::ifstream fc(argv[4]);
    if (!fc.good()) { std::cerr << "cannot open calib: " << argv[4] << "\n"; return 1; }
    std::string line;
    while (std::getline(fc, line)) {
      if (line.substr(0, frame_key.size()) != frame_key) continue;
      std::istringstream ss(line.substr(frame_key.size()));
      Tf = parse3x4(ss);
      Tf_inv = Tf.inverse();
      have_calib = true;
      break;
    }
    if (!have_calib) {
      std::cerr << "key \"" << frame_key << "\" not found in " << argv[4] << "\n";
      return 1;
    }
  }

  std::vector<Eigen::Isometry3f> poses;
  std::string line;
  while (std::getline(fp, line)) {
    if (line.empty()) continue;
    std::istringstream ss(line);
    poses.push_back(parse3x4(ss));
  }

  std::vector<double> times;
  while (std::getline(ft, line)) {
    if (line.empty()) continue;
    times.push_back(std::stod(line));
  }

  if (poses.size() != times.size()) {
    std::cerr << "pose count (" << poses.size()
              << ") != time count (" << times.size() << ")\n";
    return 1;
  }

  std::ofstream out(argv[3]);
  out << std::fixed << std::setprecision(9);
  for (size_t i = 0; i < poses.size(); ++i) {
    Eigen::Isometry3f T = have_calib ? Tf_inv * poses[i] * Tf : poses[i];
    Eigen::Quaternionf q(T.linear());
    out << times[i]
        << " " << T.translation().x()
        << " " << T.translation().y()
        << " " << T.translation().z()
        << " " << q.x()
        << " " << q.y()
        << " " << q.z()
        << " " << q.w()
        << "\n";
  }
  const std::string frame_name = (argc > 5) ? argv[5] : "Tr";
  std::cerr << "wrote " << poses.size() << " poses -> " << argv[3]
            << (have_calib ? (" (frame: " + frame_name + ")") : " (camera frame)") << "\n";
  return 0;
}
