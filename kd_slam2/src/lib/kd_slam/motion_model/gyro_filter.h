#pragma once
namespace kd_slam{
  namespace slam {

    struct GyroFilter {
      double prev_update_ts=-1;
      Eigen::Vector3f bias;
      Eigen::Matrix3f sigma_process;
      Eigen::Matrix3f sigma_meas;
      Eigen::Matrix3f sigma_bias;
      Eigen::Vector3f m_sum;
      Eigen::Vector3f estimate;
      int sum_cnt=0;
      GyroFilter (){
        sigma_process=Eigen::Matrix3f::Identity()*1e-9;
        sigma_meas=Eigen::Matrix3f::Identity()*1e-6;
        reset();
      }
      
      void reset() {
        sigma_bias.setZero();
        m_sum.setZero();
        sum_cnt=0;
        bias.setZero();
        estimate.setZero();
      }

      inline void predict(double ts, const Eigen::Vector3f& gyro) {
        m_sum+=gyro;
        sigma_bias+=sigma_process;
        ++sum_cnt;
      }

      inline std::pair<bool, Eigen::Vector3f> getPrediction() {
        Eigen::Vector3f ret=Eigen::Vector3f::Zero();
        if (prev_update_ts<0)
          return std::make_pair(false, ret);
        if (!sum_cnt) 
          return std::make_pair(false, ret);
        Eigen::Vector3f avg=m_sum*(1./sum_cnt);
        return std::make_pair(true, Eigen::Vector3f(avg+bias));
      }

      inline bool update(double ts, const Eigen::Matrix<float, 6, 1>& delta_log) {
        if (prev_update_ts<0) {
          prev_update_ts=ts;
          reset();
          return false;
        }
        if (!sum_cnt) 
          return false;
        m_sum*=(1./sum_cnt);
        double dt=ts-prev_update_ts;
        estimate=m_sum+bias;
        Eigen::Vector3f z=delta_log.tail<3>()*(1./dt);
        Eigen::Matrix3f K=sigma_bias*(sigma_bias+sigma_meas).inverse();
        bias+=K*(z-estimate);
        sigma_bias-=K*sigma_bias;
        m_sum.setZero();
        sum_cnt=0;
        prev_update_ts=ts;
        return true;
      }
    };
  }
}
