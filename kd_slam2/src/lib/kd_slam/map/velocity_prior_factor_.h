#pragma once
#include <srrg_solver/solver_core/error_factor.h>
#include <srrg_solver/solver_core/measurement_owner.h>
#include <srrg_solver/variables_and_factors/types_common/variable_vector.h>

namespace kd_slam {
  namespace map {
    using namespace srrg2_solver;

    // Unary prior that pulls a VariableVector_<N> toward a measured velocity.
    // Set the information matrix to damping * I before adding to the graph.
    template <typename VelocityVariableType_>
    struct VelocityPriorFactor_
      : public ErrorFactor_<VelocityVariableType_::Dim, VelocityVariableType_>,
        public MeasurementOwnerEigen_<typename VelocityVariableType_::EstimateType> {

      static constexpr int VDim = VelocityVariableType_::Dim;
      using BaseType             = ErrorFactor_<VDim, VelocityVariableType_>;
      using EstimateType         = typename VelocityVariableType_::EstimateType;
      using MeasurementOwnerType = MeasurementOwnerEigen_<EstimateType>;
      using InformationMatrixType = typename BaseType::InformationMatrixType;
      VelocityPriorFactor_();

      void errorAndJacobian(bool error_only = false) override;
    };

  } // namespace slam
} // namespace kd_slam
