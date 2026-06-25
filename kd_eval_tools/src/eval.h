#pragma once
#include "trajectory.h"
#include <iostream>
// v[0]: rotational_error
// v[1]: translational_error
struct ResultItem {
  Eigen::Vector2f error;
  Eigen::Isometry3f gt_pose;
  Eigen::Isometry3f es_pose;
};

using ResultMap = std::map<double, ResultItem>;

inline ResultItem poseError(const Isometry3f& a,
                          const Isometry3f& b) {
  const Isometry3f delta=a.inverse()*b;
  AngleAxisf aa(delta.linear());
  return ResultItem {Vector2f(fabs(aa.angle()*180.f/M_PI), delta.translation().norm()),
                     a,
                     b};
};

inline std::ostream& operator << (std::ostream& os, const Eigen::Isometry3f& iso) {
  os << iso.translation().transpose() << " "
     << Eigen::Quaternionf(iso.linear()).coeffs().transpose();
  return os;
}

void computeATE(ResultMap& m, const Trajectory& gt, const Trajectory& ev);

void computeRPE(ResultMap& m, const Trajectory& gt, const Trajectory& ev);

void dumpResults(std::ostream& os, const ResultMap& ate, const ResultMap& rpe);

Eigen::Vector2f rmse(const ResultMap& src);

Eigen::Vector2f rpeStat(const ResultMap& src);

bool eval(Eigen::Vector2f& dest_ate,
          Eigen::Vector2f& dest_rpe,
          const std::string& gt_filename,
          const std::string& ev_filename,
          const std::string& output_path,
          std::ostream& log,
          bool disable_bench=false,
          bool interpolate_on=false);
