#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

struct IMUFilter {
  enum Status {Initializing, Tracking};
  static constexpr const char* status2str[]={"Initializing","Tracking"};
  using Vector3f=Eigen::Vector3f;
  using Matrix3f=Eigen::Matrix3f;
  using Vector6f=Eigen::Matrix<float, 6,1>;
  using Vector9f=Eigen::Matrix<float, 9,1>;
  using Isometry3f=Eigen::Isometry3f;
  using Vector15f=Eigen::Matrix<float, 15,1>;
  using Matrix15f=Eigen::Matrix<float, 15,15>;
  using Matrix6f=Eigen::Matrix<float, 6,6>;
  struct State {
    Matrix3f R;
    Vector3f p,v,bg,ba;
    inline void boxplus(const Vector15f& dx);
  };

  //constants
  static const Vector3f g;
  
  //parameters assigned in setup()
  Matrix15f sigma_x_reset; // reset covariance
  Matrix6f  sigma_dX;     // measurement update default cov
  Matrix6f  sigma_ga;     // gyro_acc noise
  Matrix6f  sigma_bias;   // bias random walk
  float     sigma_zupd_v;
  float     v_still_max;
  float     w_still_max;
  double    still_time_init;
  
  // running variables
  Status status;
  State X;
  Matrix15f sigma_x;    // estimate
  Vector3f w_meas_sum, a_meas_sum;
  int meas_cnt;
  double    still_time=0;
  double prev_update_ts=-1;
  double prev_ts=-1;
  Isometry3f prev_updated_pose=Eigen::Isometry3f::Identity();
  Vector6f updated_delta;
  Vector6f predDelta() const;
  Vector6f updatedDelta() const;
  
  void predict(double ts,
               const Vector3f& w_meas,
               const Vector3f& a_meas);

  
  void update(double ts,
              const Vector6f& delta){
    update(ts, delta, sigma_dX);
  }

  void update(double ts,
              const Vector6f& delta,
              const Matrix6f& sigma_z);


  void dynUpdate(double ts,
                 const Vector6f& delta,
                 const Matrix6f&  sigma_z);

  void zeroUpdate(double ts);

  void remapCovariance(const Vector3f& dr);

  void reset(bool hard);
  void setup();
  IMUFilter();
};
