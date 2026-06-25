#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace kd_slam {
  namespace utils {

    // ---------------------------------------------------------------------------
    // IMU EKF on manifold (float precision).
    //
    // State (18-DOF, right perturbation on SO(3)):
    //   tangent order: [delta_R(0:3)  delta_t(3:6)  delta_omega(6:9)
    //                   delta_v(9:12) delta_bg(12:15) delta_ba(15:18)]
    //
    // predict() -- called for each IMU sample (gyro + accel)
    // update()  -- called once per LiDAR frame with RELATIVE measurements:
    //   6-DOF overload  : delta pose since last update  (ICP delta)
    //   12-DOF overload : pose + velocity               (CT-ICP, absolute)
    // ---------------------------------------------------------------------------
    struct ImuEKF {

      // --- state ---------------------------------------------------------------
      Eigen::Matrix3f R     = Eigen::Matrix3f::Identity();
      Eigen::Vector3f t     = Eigen::Vector3f::Zero();
      Eigen::Vector3f omega = Eigen::Vector3f::Zero();  // angular velocity (body)
      Eigen::Vector3f v     = Eigen::Vector3f::Zero();  // linear velocity (world)
      Eigen::Vector3f bg    = Eigen::Vector3f::Zero();  // gyroscope bias
      Eigen::Vector3f ba    = Eigen::Vector3f::Zero();  // accelerometer bias

      // --- reference state at last update (for relative measurement) ----------
      Eigen::Matrix3f R_prev = Eigen::Matrix3f::Identity();
      Eigen::Vector3f t_prev = Eigen::Vector3f::Zero();

      // --- covariance (tangent space, 18x18) -----------------------------------
      Eigen::Matrix<float,18,18> P =
          Eigen::Matrix<float,18,18>::Identity() * 1e-4f;

      // --- noise parameters (tunable) ------------------------------------------
      float sigma_gyro  = 1e-3f;   // rad/s/sqrt(Hz)   -- gyro measurement noise
      float sigma_accel = 1e-2f;   // m/s^2/sqrt(Hz)   -- accel measurement noise
      float sigma_bg    = 1e-5f;   // rad/s^2/sqrt(Hz) -- gyro bias random walk
      float sigma_ba    = 1e-4f;   // m/s^3/sqrt(Hz)   -- accel bias random walk

      Eigen::Vector3f gravity = {0.f, 0.f, 9.81f};  // world frame

      // re-orthogonalize R every N predict steps (prevents float drift)
      int recondition_every = 10;

      // -------------------------------------------------------------------------
      void reset();

      // Returns true when:
      //  - bias sigma has converged (_valid latch)
      //  - rotation covariance is small (P reconverged after any fix())
      //  - gravity direction is sane: R.col(2).z() > min_gravity_z
      //    (body z not horizontal or inverted in world z-up frame)
      bool isValid() const {
        return _valid
            && R.allFinite() && t.allFinite()
            && P.block<3,3>(0,0).diagonal().maxCoeff() < sigma_R_threshold
            && R.col(2).z() > min_gravity_z;
      }

      // IMU predict -- one call per IMU sample
      // omega_m: gyro measurement [rad/s], a_m: accel measurement [m/s^2]
      void predict(const Eigen::Vector3f& omega_m,
                   const Eigen::Vector3f& a_m,
                   float dt);

      // Relative pose update (ICP): dR_meas = R_prev^T * R_now  (ICP delta rotation)
      //                              dt_meas = t_now - t_prev    (ICP delta translation)
      // Sigma_pose: 6x6 covariance in the ICP local frame
      void update(const Eigen::Matrix3f& dR_meas,
                  const Eigen::Vector3f& dt_meas,
                  const Eigen::Matrix<float,6,6>& Sigma_pose);

      // Pose + velocity update (CT-ICP)
      // Sigma_full: 12x12 covariance matrix encoding cross-correlations
      void update(const Eigen::Matrix3f& R_meas,
                  const Eigen::Vector3f& t_meas,
                  const Eigen::Vector3f& omega_meas,
                  const Eigen::Vector3f& v_meas,
                  const Eigen::Matrix<float,12,12>& Sigma_full);

      float bias_sigma_threshold = 1e-3f;  // bias sigma convergence threshold
      float sigma_R_threshold    = 1e-2f;  // rotation covariance threshold (rad^2)
      float min_gravity_z        = 0.5f;   // min R.col(2).z() -- rejects 90-deg flips
      float icp_sigma_scale      = 100.f; // inflate ICP covariance (data assoc. errors)
      float zupt_min_t           = 0.05f;  // ZUPT: zero v/omega when |dt_meas| < this (m)
      float zupt_min_alpha       = 0.001f;  // ZUPT: zero v/omega when |dR_meas| < this (rad)
      float sigma_gravity        = 0.5f;   // accel gravity update noise (m/s^2)
      float accel_gate_low       = 0.96f;  // gravity update: min |a_smooth|/|g| ratio
      float accel_gate_high      = 1.1f;  // gravity update: max |a_smooth|/|g| ratio
      float accel_lpf_alpha      = 0.1f;   // EMA smoothing factor for accel (0=frozen, 1=raw)

      Eigen::Vector3f a_smooth   = Eigen::Vector3f::Zero();  // low-pass filtered accel
      int sampleCount() const {return _sample_count;}
    private:
      int _predict_count = 0;
      size_t _sample_count = 0;
      bool _valid = false;
      void _reconditionR();
      void _applyDelta(const Eigen::Matrix<float,18,1>& delta);

      // shared Kalman update given innovation e and measurement jacobian H
      template<int MeasDim>
      void _kalmanUpdate(const Eigen::Matrix<float,MeasDim,1>& e,
                         const Eigen::Matrix<float,MeasDim,18>& H,
                         const Eigen::Matrix<float,MeasDim,MeasDim>& R_noise);
    };

  } // namespace utils
} // namespace kd_slam

/*
 drive: 1000 0.05 0.01 0.3 0.97 1.02 0.1
 handeld: 
 */
