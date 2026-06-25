// kitti_tum2poses <estimate.tum> <output.txt> [calib.txt [frame]]
//   estimate.tum : TUM trajectory (timestamp tx ty tz qx qy qz qw, velodyne frame)
//   output.txt   : KITTI poses format (N lines, 12 floats, row-major 3x4, camera frame)
//   calib.txt    : calibration file (default: identity, poses stay in velodyne frame)
//   frame        : key in calib.txt (default: Tr, velodyne->camera)
//
// Exact algebraic inverse of kitti_gt2tum:
//   kitti_gt2tum applies  T_velo = Tr_inv * T_cam * Tr
//   this tool applies     T_cam  = Tr     * T_velo * Tr_inv
//
// Poses are normalized to the first frame.

#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

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

static Eigen::Isometry3f parseTUM(const std::string& line) {
  std::istringstream ss(line);
  double ts;
  float tx, ty, tz, qx, qy, qz, qw;
  ss >> ts >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
  Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
  T.translation() = Eigen::Vector3f(tx, ty, tz);
  T.linear()      = Eigen::Quaternionf(qw, qx, qy, qz).toRotationMatrix();
  return T;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: kitti_tum2poses <estimate.tum> <output.txt> [calib.txt [frame]]\n"
              << "  frame: key in calib.txt (default: Tr, velodyne->camera)\n";
    return 1;
  }

  std::ifstream ft(argv[1]);
  if (!ft.good()) { std::cerr << "cannot open: " << argv[1] << "\n"; return 1; }

  Eigen::Isometry3f Tf     = Eigen::Isometry3f::Identity();
  Eigen::Isometry3f Tf_inv = Eigen::Isometry3f::Identity();
  bool have_calib = false;
  if (argc > 3) {
    const std::string frame_key = (argc > 4) ? std::string(argv[4]) + ":" : "Tr:";
    std::ifstream fc(argv[3]);
    if (!fc.good()) { std::cerr << "cannot open calib: " << argv[3] << "\n"; return 1; }
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
      std::cerr << "key \"" << frame_key << "\" not found in " << argv[3] << "\n";
      return 1;
    }
  }

  std::vector<Eigen::Isometry3f> poses;
  std::string line;
  while (std::getline(ft, line)) {
    if (line.empty() || line[0] == '#') continue;
    poses.push_back(parseTUM(line));
  }

  if (poses.empty()) { std::cerr << "no poses loaded\n"; return 1; }

  // convert to camera frame: T_cam = Tr * T_velo * Tr_inv
  for (auto& T : poses)
    T = have_calib ? Tf * T * Tf_inv : T;

  // normalize to first pose
  const Eigen::Isometry3f T0_inv = poses[0].inverse();
  for (auto& T : poses)
    T = T0_inv * T;

  std::ofstream out(argv[2]);
  out << std::fixed << std::setprecision(12);
  for (const auto& T : poses) {
    const Eigen::Matrix3f& R = T.linear();
    const Eigen::Vector3f& t = T.translation();
    out << R(0,0) << " " << R(0,1) << " " << R(0,2) << " " << t(0) << " "
        << R(1,0) << " " << R(1,1) << " " << R(1,2) << " " << t(1) << " "
        << R(2,0) << " " << R(2,1) << " " << R(2,2) << " " << t(2) << "\n";
  }

  const std::string frame_name = (argc > 4) ? argv[4] : "Tr";
  std::cerr << "wrote " << poses.size() << " poses -> " << argv[2]
            << (have_calib ? (" (frame: " + frame_name + ")") : " (velodyne frame, no calib)") << "\n";
  return 0;
}
