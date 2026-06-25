#include "trajectory.h"
#include <iomanip>
#include <Eigen/Cholesky>

inline Isometry3f interpolatePose(const Isometry3f& a,
                                  const Isometry3f& b,
                                  float alpha) {
  const float epsilon=1e-5;
  Isometry3f delta=a.inverse()*b;
  AngleAxisf aa(delta.linear());
  if (fabs(aa.angle())>epsilon){
    delta.linear()=AngleAxisf(aa.angle()*alpha, aa.axis()).toRotationMatrix();
  } else {
    delta.linear().setIdentity();
  }
  delta.translation()*=alpha;
  return a*delta;
}

void Trajectory::read(std::istream& is, std::ostream& log) {
  log << std::fixed << std::setprecision(5);
  static constexpr int LINE_SIZE=1024;
  this->clear();
  int line_num;
  while (is) {
    char buf[LINE_SIZE];
    is.getline(buf, LINE_SIZE);
    ++line_num;
    std::istringstream ls(buf);
    std::string token;
    ls >> token;
    if (token.empty() || token[0]=='#')
      continue;
    double ts;
    Isometry3f iso=Isometry3f::Identity();
    ts=atof(token.c_str());
    ls >> iso.translation().x()
       >> iso.translation().y()
       >> iso.translation().z();
    if (! ls) {
      log << "WARN| broken line at " << line_num << endl;
      continue;
    }
    Quaternionf q;
    ls >> q.x()
       >> q.y()
       >> q.z()
       >> q.w();
    if (! ls) {
      log << "WARN| broken line at " << line_num << endl;
      continue;
    }
    iso.linear()=q.toRotationMatrix();
    auto it=this->find(ts);
    if (it!=this->end()){
      log << "WARN| duplicated ts [" << ts
          << "]" << "at line " << line_num << endl;
    }
    this->insert(std::make_pair(ts, iso));
  }
  
  log << "computing the distance set" << endl;
  double distance=0;
  _d2ts.clear();
  _ts2d.clear();
  Eigen::Isometry3f inv_previous_pose=Eigen::Isometry3f::Identity();
  for (const auto& p: (*this)) {
    if (_d2ts.empty()) {
      inv_previous_pose=p.second.inverse();
    }
    Eigen::Isometry3f motion=inv_previous_pose*p.second;
    distance+=motion.translation().norm();
    _d2ts.insert(std::make_pair(distance, p.first));
    _ts2d.insert(std::make_pair(p.first, distance));
    inv_previous_pose=p.second.inverse();
  }
  _length=distance;
  log << "trajectory length: " << _length << endl;

}

void Trajectory::write(std::ostream& os) const {
  os << std::fixed << std::setprecision(5);
  os << "# timestamp x y z qx ay qz qw";
  for (const auto& s: (*this)) {
    os << s.first << " ";
    const auto& pose=s.second;
    os << s.second.translation().transpose();
    Quaternionf q(pose.linear());
    os << q.x() << " "
       << q.y() << " "
       << q.z() << " "
       << q.w() << endl;
  }
}

std::pair<bool, Isometry3f> Trajectory::getPoseByStamp(double ts, bool verbose) const{
  double epsilon=1e-4;
  std::pair<bool, Isometry3f> result;
  result.first=false;
  result.second.setIdentity();
  auto lb=this->lower_bound(ts);
  auto ub=this->upper_bound(ts);
  if(lb==this->end())
    return result;

  if (! enable_interpolation) {
    result.first=true;
    if (ub==this->end())
      ub=lb;
    if ( (ts-lb->first)<(ub->first-ts)) {
      result.second=lb->second;
    } else {
      result.second=ub->second;
    }
    return result;
  }
  if (ub==this->end())
    return result;
  
  result.first=true;
  double delta=(ub->first-lb->first);
  if (delta<epsilon){
    result.second=lb->second;
  } else if (enable_interpolation) {
    float alpha=(ts-lb->first)/delta;
    result.second=interpolatePose(lb->second, ub->second, alpha);
  }
  return result;
}


Eigen::Matrix3f skew(const Eigen::Vector3f& v) {
  Eigen::Matrix3f m;
  m <<
    0,      -v.z(),  v.y(),
    v.z(),   0,     -v.x(),
    -v.y(),  v.x(),  0;
  return m;
}

Eigen::Transform<float, 3, Eigen::Affine> Trajectory::computeAlignment(const Trajectory& other, std::ostream& log, bool with_scaling) const {
  std::vector<Vector3f> this_t, other_t;
  this_t.reserve(this->size());
  other_t.reserve(this->size());
  for(const auto& p: (*this)) {
    double ts=p.first;
    auto result=other.getPoseByStamp(ts);
    if (! result.first)
      continue;
    this_t.push_back(p.second.translation());
    other_t.push_back(result.second.translation());
  }
  log << "registering with " << this_t.size() << " poses" << endl;
  using RegMatrix=Eigen::Matrix<float,3, Eigen::Dynamic>;
  Eigen::Map<RegMatrix> this_m((float*) this_t.data(), 3, this_t.size());
  Eigen::Map<RegMatrix> other_m((float*) other_t.data(), 3, this_t.size());
  Eigen::Transform<float, 3, Eigen::Affine> result;
  result.matrix()=Eigen::umeyama(this_m, other_m, with_scaling);
  // if (! with_scaling) {
  //   // nonlinear refinement
  //   int max_iters=5;
  //   for (int i=0; i<max_iters; ++i) {
  //     Eigen::Matrix<float, 6, 6> H;
  //     Eigen::Matrix<float, 6, 1> b;
  //     Eigen::Matrix<float, 6, 1> dx;
  //     H.setZero();
  //     b.setZero();
  //     double chi2=0;
  //     for (size_t i=0; i<this_t.size(); ++i) {
  //       Eigen::Vector3f z=result*this_t[i];
  //       Eigen::Vector3f e=z-other_t[i];
  //       chi2+=e.squaredNorm();
  //       Eigen::Matrix3f Jr=skew(-z);
  //       H.block<3,3>(0,0)+=Eigen::Matrix3f::Identity();
  //       H.block<3,3>(0,3)+=Jr;
  //       H.block<3,3>(3,0)+=Jr.transpose();
  //       H.block<3,3>(3,3)+=Jr.transpose()*Jr;
  //       b.block<3,1>(0,0)+=e;
  //       b.block<3,1>(3,0)+=Jr.transpose()*e;
  //     }
  //     dx=H.ldlt().solve(-b);
  //     Eigen::Vector3f rot=dx.block<3,1>(3,0);
  //     float alpha=rot.norm();
  //     Eigen::Transform<float, 3, Eigen::Affine> dT;
  //     dT.setIdentity();
  //     if (fabs(alpha>1e-5)) {
  //       Eigen::Vector3f axis=rot*(1./alpha);
  //       dT.linear()=Eigen::AngleAxisf(alpha, axis).toRotationMatrix();
  //     }
  //     dT.translation()=dx.block<3,1>(0,0);
  //     result=dT*result;
  //     cerr << "iter: " << i << "chi2: " << chi2 << endl;
  //   }
  // }
  return result;
}

void Trajectory::applyTransform(const Eigen::Transform<float, 3, Eigen::Affine>& t) {
  for (auto& p: (*this)){
    p.second.linear()=t.linear()*p.second.linear();
    p.second.translation()=t*p.second.translation();
  }
}

double Trajectory::distance2ts(double distance) const {
  DoublePair dp={distance, 0};
  auto lb=_d2ts.lower_bound(dp);
  auto ub=_d2ts.upper_bound(dp);
  if (lb==_d2ts.end() || ub==_d2ts.end()) {
    return -1;
  }
  return lb->second;
}

double Trajectory::ts2distance(double ts) const {
  DoublePair dp={ts, 0};
  auto lb=_ts2d.lower_bound(dp);
  auto ub=_ts2d.upper_bound(dp);
  if (lb==_ts2d.end() || ub==_ts2d.end()) {
    return -1;
  }
  return lb->second;
}

Trajectory::const_iterator Trajectory::getValidSample(double ts) const {
  auto lb=lower_bound(ts);
  auto ub=upper_bound(ts);
  if (lb==end()||ub==end())
    return end();
  return lb;
}
