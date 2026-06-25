#pragma once
#include <iostream>
#include <sstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <iomanip>

using Isometry3f = Eigen::Isometry3f;
using Quaternionf = Eigen::Quaternionf;
using AngleAxisf = Eigen::AngleAxisf;
using Vector2f = Eigen::Vector2f;
using Vector3f = Eigen::Vector3f;
using Vector4f = Eigen::Vector4f;
using Vector6f = Eigen::Matrix<float, 6, 1>;
using Vector7f = Eigen::Matrix<float, 6, 1>;
using std::cin;
using std::cout;
using std::cerr;
using std::endl;


struct Trajectory: public std::map<double, Isometry3f> {

  void read(std::istream& is, std::ostream& log);
  void write(std::ostream& os) const;
  
  std::pair<bool, Isometry3f>
  getPoseByStamp(double ts, bool verbose=false) const ;

  const_iterator getValidSample(double ts) const;
  
  Eigen::Transform<float, 3, Eigen::Affine>
  computeAlignment(const Trajectory& other,
                   std::ostream& log,
                   bool with_scaling=false) const;

  void applyTransform(const Eigen::Transform<float, 3, Eigen::Affine>& t);

  using DoublePair=std::pair<double, double>;
  struct DoublePairCompare{
    inline bool operator()(const DoublePair& a, const DoublePair& b)  const {
      return a.first<b.first || (a.first==b.first && a.second < b.second);
    }
  };

  inline const double& length() const { return _length; }

  // returns -1 if outside
  double  distance2ts(double distance) const ;
  double  ts2distance(double ts) const;
  
  using DoublePairSet=std::set<DoublePair, DoublePairCompare>;
  
  DoublePairSet _d2ts;
  DoublePairSet _ts2d;
  double _length=0;
  bool enable_interpolation=false;
};

