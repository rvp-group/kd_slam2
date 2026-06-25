#pragma once
#include <iomanip>
#include <string>
#include <map>
#include <Eigen/Geometry>
#include "write_isometry_.h"

// Read all poses from a text file.
// Blank lines and lines starting with '#' are ignored.
std::map<double, Eigen::Isometry3f> readTUM(const std::string& path);

// ---- TUM trajectory output -----------------------------------------------

template <typename IsometryT>
static void writeTUM(std::ostream& os, double ts, const IsometryT& T) {
  os << std::fixed << std::setprecision(9) << ts << " ";
  kd_slam::writeIsometry3(os, T);
  os << "\n";
}
