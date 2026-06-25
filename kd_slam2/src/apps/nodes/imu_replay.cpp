// Standalone IMU EKF replay.
// Reads a dump produced by SLAM_::imuPredict / imuUpdate and replays the
// filter offline, printing EKF state after every update.
//
// Record format:
//   IMU_RESET
//   IMU_PREDICT  <ts>  <wx> <wy> <wz>  <ax> <ay> <az>
//   IMU_UPDATE   <ts>  <tx> <ty> <tz>  <rx> <ry> <rz>
//                <s00> <s01> <s02> <s03> <s04> <s05>
//                      <s11> <s12> <s13> <s14> <s15>
//                            <s22> <s23> <s24> <s25>
//                                  <s33> <s34> <s35>
//                                        <s44> <s45>
//                                              <s55>
//   (21 upper-triangle elements of 6x6 sigma, row-major)

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <Eigen/Core>
#include "kd_slam/utils/imu_ekf_.h"
#include "kd_slam/utils/geometry_3d_impl_.h"

using namespace std;
using namespace kd_slam::utils;
using Traits = kd_slam::geometry::GeometryTraits_<3, float>;

static void printState(const ImuEKF& pre, const ImuEKF& post, double ts,
                       const Eigen::Vector3f& meas_dR, const Eigen::Vector3f& meas_dt) {
  Eigen::Vector3f r_log = Traits::logSO3(post.R);
  Eigen::Vector3f dr    = Traits::logSO3(pre.R.transpose() * post.R);
  Eigen::Vector3f dt    = post.t  - pre.t;
  Eigen::Vector3f dv    = post.v  - pre.v;
  Eigen::Vector3f dbg   = post.bg - pre.bg;
  Eigen::Vector3f dba   = post.ba - pre.ba;
  printf("  UPDATE ts=%.6f  valid=%d\n"
         "    meas_dR  = %8.5f %8.5f %8.5f\n"
         "    meas_dt  = %8.5f %8.5f %8.5f\n"
         "    R_log    = %8.4f %8.4f %8.4f\n"
         "    g_dir    = %8.4f %8.4f %8.4f\n"
         "    t        = %8.4f %8.4f %8.4f\n"
         "    v        = %8.4f %8.4f %8.4f\n"
         "    bg       = %8.5f %8.5f %8.5f\n"
         "    ba       = %8.5f %8.5f %8.5f\n"
         "    dR       = %8.5f %8.5f %8.5f\n"
         "    dt       = %8.5f %8.5f %8.5f\n"
         "    dv       = %8.5f %8.5f %8.5f\n"
         "    dbg      = %8.5f %8.5f %8.5f\n"
         "    dba      = %8.5f %8.5f %8.5f\n",
         ts,
         (int)post.isValid(),
         meas_dR.x(), meas_dR.y(), meas_dR.z(),
         meas_dt.x(), meas_dt.y(), meas_dt.z(),
         r_log.x(), r_log.y(), r_log.z(),
         post.R(0,2), post.R(1,2), post.R(2,2),
         post.t.x(), post.t.y(), post.t.z(),
         post.v.x(), post.v.y(), post.v.z(),
         post.bg.x(), post.bg.y(), post.bg.z(),
         post.ba.x(), post.ba.y(), post.ba.z(),
         dr.x(),  dr.y(),  dr.z(),
         dt.x(),  dt.y(),  dt.z(),
         dv.x(),  dv.y(),  dv.z(),
         dbg.x(), dbg.y(), dbg.z(),
         dba.x(), dba.y(), dba.z());
  Eigen::Vector3f pcov = post.P.block<3,3>(0,0).diagonal();
  printf("    P_R_diag = %.2e %.2e %.2e\n", pcov.x(), pcov.y(), pcov.z());
}

int main(int argc, char** argv) {
  if (argc < 2) {
    fprintf(stderr, "usage: imu_replay <dump_file> [sigma_scale] [min_t] [min_alpha] [sigma_gravity] [gate_low] [gate_high] [lpf_alpha]\n");
    return 1;
  }

  ifstream f(argv[1]);
  if (!f) {
    fprintf(stderr, "cannot open %s\n", argv[1]);
    return 1;
  }
  ImuEKF ekf;

  float sigma_scale      = (argc >= 3) ? atof(argv[2]) : ekf.icp_sigma_scale;
  float min_t            = (argc >= 4) ? atof(argv[3]) : ekf.zupt_min_t;
  float min_alpha        = (argc >= 5) ? atof(argv[4]) : ekf.zupt_min_alpha;
  float sigma_gravity    = (argc >= 6) ? atof(argv[5]) : ekf.sigma_gravity;
  float accel_gate_low   = (argc >= 7) ? atof(argv[6]) : ekf.accel_gate_low;
  float accel_gate_high  = (argc >= 8) ? atof(argv[7]) : ekf.accel_gate_high;
  float accel_lpf_alpha  = (argc >= 9) ? atof(argv[8]) : ekf.accel_lpf_alpha;
  fprintf(stderr, "sigma_scale=%.4f  min_t=%.4f  min_alpha=%.4f  sigma_gravity=%.4f  gate=[%.3f,%.3f]  lpf=%.3f\n",
          sigma_scale, min_t, min_alpha, sigma_gravity, accel_gate_low, accel_gate_high, accel_lpf_alpha);

  ekf.sigma_gravity   = sigma_gravity;
  ekf.accel_gate_low  = accel_gate_low;
  ekf.accel_gate_high = accel_gate_high;
  ekf.accel_lpf_alpha = accel_lpf_alpha;
  double last_ts = 0.0;
  int predict_count = 0;
  int update_count  = 0;
  int reset_count   = 0;

  string line;
  while (getline(f, line)) {
    if (line.empty()) continue;
    istringstream ss(line);
    string tag;
    ss >> tag;

    if (tag == "IMU_RESET") {
      ekf.reset();
      last_ts = 0.0;
      ++reset_count;
      printf("RESET #%d\n", reset_count);
      continue;
    }

    if (tag == "IMU_PREDICT") {
      double ts;
      float wx, wy, wz, ax, ay, az;
      ss >> ts >> wx >> wy >> wz >> ax >> ay >> az;
      float dt = (last_ts > 0.0) ? (float)(ts - last_ts) : 0.f;
      ekf.predict(Eigen::Vector3f(wx, wy, wz),
                  Eigen::Vector3f(ax, ay, az),
                  dt);
      last_ts = ts;
      ++predict_count;
      continue;
    }

    if (tag == "IMU_UPDATE") {
      double ts;
      float tx, ty, tz, rx, ry, rz;
      ss >> ts >> tx >> ty >> tz >> rx >> ry >> rz;

      // read 21 upper-triangle elements
      Eigen::Matrix<float,6,6> sigma = Eigen::Matrix<float,6,6>::Zero();
      for (int r = 0; r < 6; ++r) {
        for (int c = r; c < 6; ++c) {
          float v;
          ss >> v;
          sigma(r, c) = v;
          sigma(c, r) = v;
        }
      }

      Eigen::Matrix3f dR = Traits::rodrigues(Eigen::Vector3f(rx, ry, rz));
      Eigen::Vector3f dt_meas(tx, ty, tz);
      ImuEKF pre = ekf;
      ekf.update(dR, dt_meas, sigma * sigma_scale);
      if (dt_meas.norm() < min_t && Eigen::Vector3f(rx, ry, rz).norm() < min_alpha) {
        ekf.v.setZero();
        ekf.omega.setZero();
      }
      ++update_count;
      printState(pre, ekf, ts, Eigen::Vector3f(rx, ry, rz), dt_meas);
      continue;
    }

    // unknown tag -- skip
  }

  printf("\nDone: %d resets, %d predicts, %d updates\n",
         reset_count, predict_count, update_count);
  return 0;
}
