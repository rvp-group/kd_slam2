#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace kd_slam {
  namespace slam {

    // Madgwick complementary filter for attitude estimation.
    // Gravity-only variant (no magnetometer): yaw is unobservable.
    // Convention: q rotates vectors from body frame to world frame.
    //   world z-axis = [0, 0, 1] = gravity up direction.
    //   gravityInBody() = R^T * [0,0,1] = direction of gravity in sensor frame.
    struct MadgwickFilter {
      Eigen::Quaternionf q    = Eigen::Quaternionf::Identity();
      float              beta = 0.1f;

      void reset() {
        q = Eigen::Quaternionf::Identity();
      }

      // gyro: angular velocity in rad/s (body frame)
      // accel: linear acceleration in m/s^2 (body frame); direction used, norm ignored
      // dt: time step in seconds
      void update(const Eigen::Vector3f& gyro,
                  const Eigen::Vector3f& accel,
                  float dt) {
        float accel_norm = accel.norm();
        if (accel_norm < 1e-6f)
          return;

        const Eigen::Vector3f a = accel / accel_norm;

        const float q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();

        // Objective function: rotate world gravity [0,0,1] into body frame,
        // compare with measured accel direction.
        // F = q^{-1} * [0,0,1] - a
        Eigen::Vector3f F;
        F(0) = 2.f*(q1*q3 - q0*q2) - a.x();
        F(1) = 2.f*(q0*q1 + q2*q3) - a.y();
        F(2) = 2.f*(0.5f - q1*q1 - q2*q2) - a.z();

        // Jacobian dF/d[q0, q1, q2, q3]  (3x4)
        Eigen::Matrix<float,3,4> J;
        J << -2*q2,  2*q3, -2*q0,  2*q1,
              2*q1,  2*q0,  2*q3,  2*q2,
              0,    -4*q1, -4*q2,  0;

        // Gradient in quaternion space
        Eigen::Vector4f grad = J.transpose() * F;
        float gn = grad.norm();
        if (gn > 1e-10f)
          grad /= gn;

        // Quaternion rate from gyroscope: q_dot = 0.5 * q * omega_q
        Eigen::Vector4f q_dot;
        q_dot(0) = 0.5f*(              - q1*gyro.x() - q2*gyro.y() - q3*gyro.z());
        q_dot(1) = 0.5f*(q0*gyro.x()                 + q2*gyro.z() - q3*gyro.y());
        q_dot(2) = 0.5f*(q0*gyro.y() + q3*gyro.x()                 - q1*gyro.z());
        q_dot(3) = 0.5f*(q0*gyro.z() - q2*gyro.x()  + q1*gyro.y()              );

        // Feedback correction
        q_dot -= beta * grad;

        // Integrate
        Eigen::Vector4f q_vec(q0, q1, q2, q3);
        q_vec += q_dot * dt;
        q_vec.normalize();

        q.w() = q_vec(0);
        q.x() = q_vec(1);
        q.y() = q_vec(2);
        q.z() = q_vec(3);
      }

      // Returns the direction of gravity ([0,0,1] world) expressed in body frame.
      Eigen::Vector3f gravityInBody() const {
        return q.conjugate() * Eigen::Vector3f(0.f, 0.f, 1.f);
      }
    };

  } // namespace slam
} // namespace kd_slam
