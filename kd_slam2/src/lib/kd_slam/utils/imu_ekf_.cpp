#include "imu_ekf_.h"
#include "geometry_3d_impl_.h"
#include <Eigen/SVD>
#include <cmath>

namespace kd_slam {
  namespace utils {

    using Traits = geometry::GeometryTraits_<3, float>;

    // SVD-based re-orthogonalization
    void ImuEKF::_reconditionR() {
      auto svd = R.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
      R = svd.matrixU() * svd.matrixV().transpose();
    }

    // -------------------------------------------------------------------------
    // Apply tangent-space correction delta to state (right perturbation on R)
    // -------------------------------------------------------------------------
    void ImuEKF::_applyDelta(const Eigen::Matrix<float,18,1>& delta) {
      R     = R * Traits::rodrigues(delta.segment<3>(0));
      t    += delta.segment<3>(3);
      omega+= delta.segment<3>(6);
      v    += delta.segment<3>(9);
      bg   += delta.segment<3>(12);
      ba   += delta.segment<3>(15);
    }

    // -------------------------------------------------------------------------
    // Shared Kalman update
    // -------------------------------------------------------------------------
    template<int MeasDim>
    void ImuEKF::_kalmanUpdate(const Eigen::Matrix<float,MeasDim,1>& e,
                                const Eigen::Matrix<float,MeasDim,18>& H,
                                const Eigen::Matrix<float,MeasDim,MeasDim>& R_noise) {
      // S = H*P*H^T + R_noise
      const Eigen::Matrix<float,MeasDim,MeasDim> S = H * P * H.transpose() + R_noise;
      // K = P*H^T * S^{-1}
      const Eigen::Matrix<float,18,MeasDim> K = P * H.transpose() * S.inverse();
      // state update
      _applyDelta(K * e);
      // covariance update (Joseph form for numerical stability)
      const Eigen::Matrix<float,18,18> IKH = Eigen::Matrix<float,18,18>::Identity() - K * H;
      P = IKH * P * IKH.transpose() + K * R_noise * K.transpose();
    }

    // Explicit instantiations
    template void ImuEKF::_kalmanUpdate<3>(
        const Eigen::Matrix<float,3,1>&,
        const Eigen::Matrix<float,3,18>&,
        const Eigen::Matrix<float,3,3>&);
    template void ImuEKF::_kalmanUpdate<6>(
        const Eigen::Matrix<float,6,1>&,
        const Eigen::Matrix<float,6,18>&,
        const Eigen::Matrix<float,6,6>&);
    template void ImuEKF::_kalmanUpdate<12>(
        const Eigen::Matrix<float,12,1>&,
        const Eigen::Matrix<float,12,18>&,
        const Eigen::Matrix<float,12,12>&);

    // -------------------------------------------------------------------------
    // reset
    // -------------------------------------------------------------------------
    void ImuEKF::reset() {
      R      = Eigen::Matrix3f::Identity();
      R_prev = Eigen::Matrix3f::Identity();
      t      = Eigen::Vector3f::Zero();
      t_prev = Eigen::Vector3f::Zero();
      omega  = Eigen::Vector3f::Zero();
      v      = Eigen::Vector3f::Zero();
      bg     = Eigen::Vector3f::Zero();
      ba     = Eigen::Vector3f::Zero();
      P        = Eigen::Matrix<float,18,18>::Identity() * 1e-4f;
      a_smooth = Eigen::Vector3f::Zero();
      _predict_count = 0;
      _sample_count  = 0;
      _valid = false;
    }

    // -------------------------------------------------------------------------
    // predict
    // -------------------------------------------------------------------------
    void ImuEKF::predict(const Eigen::Vector3f& omega_m,
                          const Eigen::Vector3f& a_m,
                          float dt) {
      if (! _sample_count) {
        Eigen::Vector3f gz = a_m.normalized();
        Eigen::Vector3f ex = Eigen::Vector3f::UnitX();
        if (std::abs(gz.dot(ex)) > 0.9f)
          ex = Eigen::Vector3f::UnitY();
        Eigen::Vector3f ey = gz.cross(ex).normalized();
        ex = ey.cross(gz).normalized();
        R.row(0) = ex;
        R.row(1) = ey;
        R.row(2) = gz;
        a_smooth = a_m;  // warm-start LPF so gravity gate fires immediately
        ++_sample_count;
        return;
      }

      // corrected measurements
      const Eigen::Vector3f w  = omega_m - bg;      // corrected angular velocity
      const Eigen::Vector3f ac = a_m - ba;           // corrected acceleration
      const Eigen::Vector3f a_world = R * ac - gravity;  // world-frame accel

      // --- state propagation ---
      const Eigen::Matrix3f R_new = R * Traits::rodrigues(w * dt);
      t     = t + v * dt + 0.5f * a_world * (dt * dt);
      v     = v + a_world * dt;
      omega = w;
      R     = R_new;

      // --- F matrix (18x18) ---
      // tangent order: [dR, dt, domega, dv, dbg, dba]
      Eigen::Matrix<float,18,18> F = Eigen::Matrix<float,18,18>::Zero();

      const Eigen::Matrix3f Ra  = R;
      const Eigen::Matrix3f Rac = Ra * Traits::skew(ac);
      const float dt2 = dt * dt;

      F.block<3,3>(0, 0)  =  Eigen::Matrix3f::Identity();
      F.block<3,3>(0, 12) = -Eigen::Matrix3f::Identity() * dt;

      F.block<3,3>(3, 0)  = -0.5f * Rac * dt2;
      F.block<3,3>(3, 3)  =  Eigen::Matrix3f::Identity();
      F.block<3,3>(3, 9)  =  Eigen::Matrix3f::Identity() * dt;
      F.block<3,3>(3, 15) = -Ra * (0.5f * dt2);

      F.block<3,3>(6, 12) = -Eigen::Matrix3f::Identity();

      F.block<3,3>(9, 0)  = -Rac * dt;
      F.block<3,3>(9, 9)  =  Eigen::Matrix3f::Identity();
      F.block<3,3>(9, 15) = -Ra * dt;

      F.block<3,3>(12, 12) = Eigen::Matrix3f::Identity();
      F.block<3,3>(15, 15) = Eigen::Matrix3f::Identity();

      // --- Q matrix (process noise) ---
      Eigen::Matrix<float,18,18> Q = Eigen::Matrix<float,18,18>::Zero();
      const float sg2  = sigma_gyro  * sigma_gyro  * dt;
      const float sa2  = sigma_accel * sigma_accel * dt;
      const float sbg2 = sigma_bg    * sigma_bg    * dt;
      const float sba2 = sigma_ba    * sigma_ba    * dt;

      Q.block<3,3>(0, 0)  = Eigen::Matrix3f::Identity() * sg2;
      Q.block<3,3>(6, 6)  = Eigen::Matrix3f::Identity() * sg2;
      Q.block<3,3>(9, 9)  = Eigen::Matrix3f::Identity() * sa2;
      Q.block<3,3>(12,12) = Eigen::Matrix3f::Identity() * sbg2;
      Q.block<3,3>(15,15) = Eigen::Matrix3f::Identity() * sba2;

      // --- covariance propagation ---
      P = F * P * F.transpose() + Q;

      // --- reconditioning ---
      ++_predict_count;
      ++_sample_count;
      if (recondition_every > 0 && _predict_count >= recondition_every) {
        _reconditionR();
        _predict_count = 0;
      }

      // --- gravity pseudo-measurement update ---
      // use low-pass filtered accel for gate and innovation (raw a_m used only for dynamics)
      a_smooth = (1.f - accel_lpf_alpha) * a_smooth + accel_lpf_alpha * a_m;
      const float ratio = a_smooth.norm() / gravity.norm();
      if (ratio > accel_gate_low && ratio < accel_gate_high) {
        const Eigen::Vector3f g_b = R.transpose() * gravity;
        const Eigen::Vector3f ac_smooth = a_smooth - ba;
        const Eigen::Matrix<float,3,1> e_g = ac_smooth - g_b;

        Eigen::Matrix<float,3,18> Hg = Eigen::Matrix<float,3,18>::Zero();
        Hg.block<3,3>(0,  0) = Traits::skew(g_b);           // dh/d(delta_R)
        Hg.block<3,3>(0, 15) = Eigen::Matrix3f::Identity();  // dh/d(delta_ba)

        const Eigen::Matrix3f Rg = Eigen::Matrix3f::Identity() * (sigma_gravity * sigma_gravity);
        _kalmanUpdate<3>(e_g, Hg, Rg);
      }
    }

    // -------------------------------------------------------------------------
    // update -- pose only (6-DOF)
    // -------------------------------------------------------------------------
    void ImuEKF::update(const Eigen::Matrix3f& dR_meas,
                         const Eigen::Vector3f& dt_meas,
                         const Eigen::Matrix<float,6,6>& Sigma_pose) {
      // EKF predicted delta since last update (expressed in prev body frame)
      const Eigen::Matrix3f dR_ekf = R_prev.transpose() * R;
      const Eigen::Vector3f dt_ekf = R_prev.transpose() * (t - t_prev);

      Eigen::Matrix<float,6,1> e;
      e.segment<3>(0) = Traits::logSO3(dR_ekf.transpose() * dR_meas);
      e.segment<3>(3) = dt_meas - dt_ekf;

      Eigen::Matrix<float,6,18> H = Eigen::Matrix<float,6,18>::Zero();
      H.block<3,3>(0, 0) = Eigen::Matrix3f::Identity();
      H.block<3,3>(3, 3) = Eigen::Matrix3f::Identity();

      _kalmanUpdate<6>(e, H, Sigma_pose);

      R_prev = R;
      t_prev = t;
      if (!_valid)
        _valid = P.block<6,6>(12,12).diagonal().maxCoeff() < bias_sigma_threshold;

      // ZUPT: when vehicle is nearly still, zero velocity and angular rate
      if (dt_meas.norm() < zupt_min_t &&
          Traits::logSO3(dR_meas).norm() < zupt_min_alpha) {
        v.setZero();
        omega.setZero();
      }
    }

    // -------------------------------------------------------------------------
    // update -- pose + velocity (12-DOF, CT-ICP)
    // -------------------------------------------------------------------------
    void ImuEKF::update(const Eigen::Matrix3f& R_meas,
                         const Eigen::Vector3f& t_meas,
                         const Eigen::Vector3f& omega_meas,
                         const Eigen::Vector3f& v_meas,
                         const Eigen::Matrix<float,12,12>& Sigma_full) {
      Eigen::Matrix<float,12,1> e;
      e.segment<3>(0) = Traits::logSO3(R.transpose() * R_meas);
      e.segment<3>(3) = t_meas - t;
      e.segment<3>(6) = omega_meas - omega;
      e.segment<3>(9) = v_meas - v;

      Eigen::Matrix<float,12,18> H = Eigen::Matrix<float,12,18>::Zero();
      H.block<3,3>(0,  0) = Eigen::Matrix3f::Identity();
      H.block<3,3>(3,  3) = Eigen::Matrix3f::Identity();
      H.block<3,3>(6,  6) = Eigen::Matrix3f::Identity();
      H.block<3,3>(9,  9) = Eigen::Matrix3f::Identity();

      _kalmanUpdate<12>(e, H, Sigma_full);
    }

  } // namespace utils
} // namespace kd_slam
