#pragma once
#include <srrg_solver/solver_core/factor.h>
#include "kd_slam/map/map_.h"

namespace kd_slam {
  namespace map {
    using namespace srrg2_solver;

    template <typename Node_>
    struct MultiViewCTICPFactor_
      : public Factor_<VariablePtrTuple_<typename MapTraits_<Node_>::SolverVariableType,
                                         typename MapTraits_<Node_>::SolverVelocityVariableType,
                                         typename MapTraits_<Node_>::SolverVariableType,
                                         typename MapTraits_<Node_>::SolverVelocityVariableType>> {

      using SolverVariableType         = typename MapTraits_<Node_>::SolverVariableType;
      using SolverVelocityVariableType = typename MapTraits_<Node_>::SolverVelocityVariableType;
      using BaseSolverType             = Factor_<VariablePtrTuple_<SolverVariableType,
                                                                    SolverVelocityVariableType,
                                                                    SolverVariableType,
                                                                    SolverVelocityVariableType>>;
      using KDFactorType               = typename Map_<Node_>::MultiViewCTICPFactor;

      using BaseSolverType::_enabled;
      using BaseSolverType::variables;
      using BaseSolverType::_H_blocks;
      using BaseSolverType::_b_blocks;
      using BaseSolverType::robustify;
      using BaseSolverType::_stats;

      void compute(bool chi_only = false, bool force = false) override;

      int  measurementDim() const override { return _measurement_dim; }
      bool isValid()        const override { return _is_valid; }
      void serialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& ctx) override;
      void deserialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& ctx) override;
      
      KDFactorType* kd_factor     = nullptr;
      int           _measurement_dim = 0;
      bool          _is_valid        = true;
    };

  } // namespace slam
} // namespace kd_slam
