#pragma once
#include "velocity_prior_factor_.h"

namespace kd_slam {
  namespace map {
    using namespace srrg2_solver;

    template <typename VelocityVariableType_>
    VelocityPriorFactor_<VelocityVariableType_>::VelocityPriorFactor_() {
      MeasurementOwnerType::_measurement.setZero();
    }

    template <typename VelocityVariableType_>
    void VelocityPriorFactor_<VelocityVariableType_>::errorAndJacobian(bool error_only) {
      this->_e = this->variables().template at<0>()->estimate() - this->_measurement;
      if (!error_only)
        this->template jacobian<0>().setIdentity();
    }

  } // namespace slam
} // namespace kd_slam
