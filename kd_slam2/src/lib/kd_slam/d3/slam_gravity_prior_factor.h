#pragma once
#include <srrg_solver/solver_core/error_factor.h>
#include <srrg_solver/solver_core/measurement_owner.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3.h>
namespace kd_slam {
  namespace map {
    using namespace srrg2_solver;

    struct GravityPriorFactor
      : public ErrorFactor_<3, VariableSE3ExpmapLeft>,
        public MeasurementOwnerEigen_<Eigen::Vector3f> {
      using Scalar               = typename VariableSE3ExpmapLeft::EstimateType::Scalar;
      using BaseType             = ErrorFactor_<3, VariableSE3ExpmapLeft>;
      using MeasurementOwnerType = MeasurementOwnerEigen_<Eigen::Vector3f>;
      using InformationMatrixType = typename BaseType::InformationMatrixType;

      GravityPriorFactor();

      void errorAndJacobian(bool error_only = false) override;
    };

  } // namespace slam
} // namespace kd_slam
