#include "imu_filter.h"

void IMUFilter::setup() {
  sigma_ga.setZero();
  sigma_ga.block<3,3>(3,3)=Matrix3f::Identity()*1e-3;
  sigma_ga.block<3,3>(3,3)=Matrix3f::Identity()*1e-2;
  sigma_bias.setZero();
  sigma_bias.block<3,3>(0,0)=Matrix3f::Identity()*1e-10;
  sigma_bias.block<3,3>(3,3)=Matrix3f::Identity()*1e-10;
  sigma_dX.setIdentity();
  //r
  sigma_dX.block<3,3>(0,0)*=1e-3;
  //t
  sigma_dX.block<3,3>(3,3)*=1e-2;

  //r

  // Sigma_x after reset
  sigma_x_reset.setIdentity();
  sigma_x_reset.block<3,3>(0,0)*=1e-2;
  //p
  sigma_x_reset.block<3,3>(3,3)*=1e-9;
  //v
  sigma_x_reset.block<3,3>(6,6)*=1e-3;
  //bg
  sigma_x_reset.block<3,3>(9,9)*=1e-6;
  //ba
  sigma_x_reset.block<3,3>(12,12)*=1e-1;

  // zupd params
  v_still_max=0.5;
  w_still_max=0.1;
  still_time_init=5;
  sigma_zupd_v=1;

}
