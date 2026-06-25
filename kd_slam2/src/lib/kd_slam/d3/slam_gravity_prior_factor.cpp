#include <srrg_solver/solver_core/variable_impl.cpp>
#include <srrg_solver/solver_core/error_factor_impl.cpp>
#include <srrg_solver/solver_core/measurement_owner_impl.cpp>
#include "kd_slam/utils/geometry_3d_.h"
#include "kd_slam/utils/geometry_3d_impl_.h"
#include "slam_gravity_prior_factor.h"
namespace kd_slam {
  namespace map {
    using namespace srrg2_solver;

    
    GravityPriorFactor::GravityPriorFactor() {
      MeasurementOwnerType::_measurement.setZero();
    }

    void GravityPriorFactor::errorAndJacobian(bool error_only) {
      Eigen::Matrix<Scalar, 3, 1> Rz=this->variables().template at<0>()->estimate().linear().col(2);
      this->_e = Rz - this->_measurement;
      if (!error_only) {
        this->template jacobian<0>().setZero();
        this->template jacobian<0>().template block<3,3>(0,3)
          =geometry::GeometryTraits_<3, Scalar>::skew(-Rz);
      }
    }
  } // namespace slam
} // namespace kd_slam
