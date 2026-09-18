#include "imu_filter.h"
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>

using namespace std;

inline Eigen::Matrix3f orthonormalize(const Eigen::Matrix3f& R) {
  Eigen::Vector3f c0 = R.col(0).normalized();
  Eigen::Vector3f c1 = (R.col(1) - c0*c0.dot(R.col(1))).normalized();
  Eigen::Vector3f c2 = c0.cross(c1);
  Eigen::Matrix3f Q;
  Q.col(0)=c0; Q.col(1)=c1; Q.col(2)=c2;
  return Q;
}

inline Eigen::Matrix3f skew(const Eigen::Vector3f& w){
  Eigen::Matrix3f m;
  m <<
    0,      -w.z(),  w.y(),
    w.z(),   0,     -w.x(),
    -w.y(),  w.x(),  0;
  return m;

}

inline Eigen::Matrix3f expSO3(const Eigen::Vector3f& w){
  static constexpr float epsilon=1e-7;
  float alpha=w.norm();
  if (alpha<epsilon)
    return Eigen::Matrix3f::Identity();
  const Eigen::Vector3f a=w*1./alpha;
  Eigen::Matrix3f S=skew(a);
  return Eigen::Matrix3f::Identity() + sin(alpha)*S + (1.f - cos(alpha))* S*S;
}

inline Eigen::Vector3f logSO3(const Eigen::Matrix3f R){
  static constexpr float epsilon=1e-7;
  float cos_a = 0.5 * (R.trace() - 1.f);
  cos_a = std::max(-1.f, std::min(1.f, cos_a));
  const float angle = std::acos(cos_a);
  const Eigen::Vector3f ax(R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1));
  if (angle < epsilon)
    return ax * 0.5f;
  return ax * (angle *.5f / std::sin(angle));
}

void IMUFilter::State::boxplus(const Eigen::Matrix<float, 15,1>& dx) {
  R=R*expSO3(dx.block<3,1>(0,0));
  R=orthonormalize(R);
  p+=dx.block<3,1>(3,0);
  v+=dx.block<3,1>(6,0);
  bg+=dx.block<3,1>(9,0);
  ba+=dx.block<3,1>(12,0);
}

void IMUFilter::predict(double ts,
             const Vector3f& w_meas,
             const Vector3f& a_meas) {
  if (prev_ts<0) {
    reset(true);
    prev_ts=ts;
    return;
  }
  float dT=ts-prev_ts;
  prev_ts=ts;
  float dT2=0.5*dT*dT;
  Vector3f w=w_meas-X.bg;
  Vector3f ac=a_meas-X.ba;
  Vector3f a=X.R*ac+g;
  Matrix3f dR=expSO3(w*dT);
  Matrix3f R1=X.R*dR;
  Vector3f v1=dR.transpose()*(X.v+ac*dT)+R1.transpose()*g*dT;
    
  Vector3f gc=R1.transpose()*g;
    
  Matrix15f A=Matrix15f::Zero();
  // dr,dp,dv,bg,ba

  // dr/dr
  A.block<3,3>(0,0)=dR.transpose();
  // dr/dbg
  A.block<3,3>(0,9)=-Eigen::Matrix3f::Identity()*dT;
    
  // dp/dr
  A.block<3,3>(3,0)=-X.R* skew(dT*X.v + dT2*ac);
  // dp/dp
  A.block<3,3>(3,3)=Eigen::Matrix3f::Identity();
  // dp/dv
  A.block<3,3>(3,6)=X.R*dT;
  // dp/dba
  A.block<3,3>(3,12)=-dT2*X.R;

  //dv/dr
  A.block<3,3>(6,0)=skew(gc*dT)*dR.transpose();
  //dv/dv
  A.block<3,3>(6,6)=dR.transpose();
  //dv/dbg
  A.block<3,3>(6,9)=-skew(v1*dT);
  //dv/dba
  A.block<3,3>(6,12)=-dR.transpose()*dT;

  //bg
  A.block<3,3>(9,9)=Eigen::Matrix3f::Identity();
  A.block<3,3>(12,12)=Eigen::Matrix3f::Identity();


  X.p+=X.R*X.v*dT+a*dT2;
  X.R=R1;
  X.v=v1;
  const auto B=A.block<15,6>(0,9);
  sigma_x=A*sigma_x*A.transpose()+B*(sigma_ga)*B.transpose();
  //sigma_x.block<9,9>(0,0)+=B*(sigma_ga)*B.transpose();
  sigma_x.block<6,6>(9,9)+=sigma_bias;
  a_meas_sum+=a_meas;
  w_meas_sum+=w_meas;
  meas_cnt++;
}

void IMUFilter::remapCovariance(const Vector3f& dr) {
    Matrix3f Jl=Eigen::Matrix3f::Identity()-skew(0.5*dr);
    Matrix15f G;
    G.setIdentity();
    G.block<3,3>(0,0)=Jl;
    sigma_x=G*sigma_x*G.transpose();
}

IMUFilter::Vector6f IMUFilter::predDelta() const {
  if (prev_ts<=0 || prev_update_ts<=0)
    return Vector6f::Zero();
  double dT=prev_ts-prev_update_ts;
  if (dT<0)
    return Vector6f::Zero();
  Eigen::Isometry3f pred_pose;
  pred_pose.linear()=X.R;
  pred_pose.translation()=X.p;
  Isometry3f pp=prev_updated_pose.inverse();
  Isometry3f pred_Delta=pp*pred_pose;
  Vector6f pred_delta;
  pred_delta.head<3>()=pred_Delta.translation();
  pred_delta.tail<3>()=logSO3(pred_Delta.linear());
  return pred_delta;
}

IMUFilter::Vector6f IMUFilter::updatedDelta() const {
  return updated_delta;
}

void IMUFilter::update(double ts,
                       const IMUFilter::Vector6f& dz,
                       const IMUFilter::Matrix6f& sigma_z) {
  if (prev_update_ts<0) {
    reset(false);
    prev_ts=ts;
    prev_update_ts=ts;
    prev_updated_pose.linear()=X.R;
    prev_updated_pose.translation()=X.p;
    still_time=0;
    return;
  }
  if (!meas_cnt)
    return;

  float dT=ts-prev_update_ts;
  if (dT<=0)
    return;
  float idT=1./dT;

  float v=dz.head<3>().norm()*idT;
  float w=dz.tail<3>().norm()*idT;

  bool is_still= (v<v_still_max && w<w_still_max);
    
  Eigen::Isometry3f pred_pose;
  pred_pose.linear()=X.R;
  pred_pose.translation()=X.p;

  Isometry3f pp=prev_updated_pose.inverse();

  if (is_still) {
    zeroUpdate(ts);
    if (still_time>0) {
      still_time+=dT;
      if (still_time>still_time_init) {
        status=Tracking;
      }
    } else {
      still_time=dT;
    }
  } else {
    dynUpdate(ts, dz, sigma_z);
    still_time=0;
  }

  Isometry3f up_pose;
  up_pose.linear()=X.R;
  up_pose.translation()=X.p;
  Isometry3f up_Delta=pp*up_pose;
  updated_delta.head<3>()=up_Delta.translation();
  updated_delta.tail<3>()=logSO3(up_Delta.linear());

}

void  IMUFilter::dynUpdate(double ts,
                           const IMUFilter::Vector6f& dz,
                           const IMUFilter::Matrix6f& sigma_z) {
  Isometry3f dZ;
  dZ.linear()=expSO3(dz.tail<3>());
  dZ.translation()=dz.head<3>();
  // inverse chained measured pose
  Isometry3f iZ=(prev_updated_pose*dZ).inverse();
    
  Isometry3f pred; // predicted pose
  pred.linear()=X.R;
  pred.translation()=X.p;

  // log error
  Isometry3f E=iZ*pred;
  Vector6f e;
  e.block<3,1>(0,0)=logSO3(E.linear());
  e.block<3,1>(3,0)=E.translation();

  // meas jacobian
  Eigen::Matrix<float,6,15> C;
  C.setZero();
  C.block<3,3>(0,0).setIdentity();
  C.block<3,3>(3,3)=iZ.linear();
  Eigen::Matrix<float, 15, 6> K = sigma_x*C.transpose()*(C*sigma_x*C.transpose()+sigma_z).inverse();
  Vector15f dx=-K*e;
  Vector3f dr=dx.block<3,1>(0,0);
  X.boxplus(dx);
  sigma_x=(Matrix15f::Identity()-K*C)*sigma_x;
  remapCovariance(dr);
  prev_updated_pose.translation()=X.p;
  prev_updated_pose.linear()=X.R;
  prev_update_ts=ts;
  prev_ts=ts;
  meas_cnt=0;
  a_meas_sum.setZero();
  w_meas_sum.setZero();
}

void IMUFilter::zeroUpdate(double ts) {
  float icnt=1./meas_cnt;
  Vector3f ac=a_meas_sum*icnt-X.ba;
  Vector3f w=w_meas_sum*icnt-X.bg;

  
  Eigen::Matrix<float, 9,9> sigma_zero;
  sigma_zero.setIdentity();
  sigma_zero.block<3,3>(0,0)*=sigma_zupd_v;
  sigma_zero.block<3,3>(3,3)=sigma_ga.block<3,3>(0,0)*icnt*1e3;
  sigma_zero.block<3,3>(6,6)=sigma_ga.block<3,3>(3,3)*icnt*1e3;
  Vector9f e;
  e.block<3,1>(0,0)=-X.v;
  e.block<3,1>(3,0)=-w;
  e.block<3,1>(6,0)=-(X.R*ac+g);
  
  Eigen::Matrix<float, 9, 15> C;
  C.setZero();
  C.block<3,3>(0,6)=-Eigen::Matrix3f::Identity();
  C.block<3,3>(3,9)=Eigen::Matrix3f::Identity();
  C.block<3,3>(6,0)=X.R*skew(ac);
  C.block<3,3>(6,12)=X.R;
  Eigen::Matrix<float, 15, 9> K = sigma_x*C.transpose()*(C*sigma_x*C.transpose()+sigma_zero).inverse();

  Vector15f dx=-K*e;
  Vector3f dr=dx.block<3,1>(0,0);
  X.boxplus(dx);
  sigma_x=(Matrix15f::Identity()-K*C)*sigma_x;
  remapCovariance(dr);
  prev_updated_pose.translation()=X.p;
  prev_updated_pose.linear()=X.R;
  prev_update_ts=ts;
  prev_ts=ts;
  meas_cnt=0;
  a_meas_sum.setZero();
  w_meas_sum.setZero();
}

IMUFilter::
IMUFilter (){
  status=Initializing;
  setup();
  prev_ts=-1;
  reset(true);
}

void IMUFilter::reset(bool hard) {
    status=Initializing;
    
    if (hard) {
      X.R.setIdentity();
      X.p.setZero();
      sigma_x=sigma_x_reset;
      prev_ts=-1;
      prev_update_ts=-1;
    }
    updated_delta.setZero();
    X.v.setZero();
    X.ba.setZero();
    X.bg.setZero();
    w_meas_sum.setZero();
    a_meas_sum.setZero();
    meas_cnt=0;
    still_time=0;
}

const Eigen::Vector3f IMUFilter::g=Vector3f(0.f,0.f,-9.81);
