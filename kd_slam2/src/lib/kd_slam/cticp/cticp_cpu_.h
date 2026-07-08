#pragma once
#include "cticp_.h"
#include "kd_slam/tree/tree_cpu_.h"


namespace kd_slam {
  namespace cticp {
    using namespace kd_slam::icp;

    // cpu implementation of KD_CTICP, specializes the base class
    // and operates on KDTrees on cpu
    // usage:
    // 1. create an instance
    // 2. assign a moving and a fixed (setMoving, setFixed)
    // 3. set initial guess in moving_state (X + velocities)
    // 4. start calling oneRound for each iteration

    template <typename Base_>
    struct CTICP_CPU_ : public Base_ {
      using Base                 = Base_;
      using Traits               = typename Base::Traits;
      using NodeType               = typename Traits::NodeType;
      using PointType            = typename Traits::PointType;
      using Scalar               = typename Traits::Scalar;
      using IsometryType         = typename Traits::IsometryType;
      using PerturbationPoseType = typename Traits::PerturbationPoseType;
      using JacobianPoseType     = typename Traits::JacobianPoseType;
      using VelocityType         = typename Traits::VelocityType;
      using JacobianVelocityType = typename Traits::JacobianVelocityType;
      using HessianType          = typename Traits::HessianType;
      using CoefficientType      = typename Traits::CoefficientType;
      using TreeBaseType       = typename Base::TreeBaseType;
      using KDTreeType           = TreeCPU_<Tree_<NodeType>>;
      using StatsType            = typename Base::StatsType;
      using DiagPoseType         = typename Base::DiagPoseType;
      using DiagVelType          = typename Base::DiagVelType;
      using CrossType            = typename Base::CrossType;
      using PointsVectorType     = std::vector<PointType>;
      using NodesVectorType      = std::vector<NodeType>;
      using State                = typename Base::State;
      PARAM(srrg2_core::PropertyInt,   num_threads,        "max threads 0/1: disable mt", 0, nullptr);

      static constexpr bool IsGPU=false;
      bool isGPU() const override {return IsGPU;}

      // inputs

      void setMoving(const TreeBaseType& moving) override;
      void setFixed(const TreeBaseType& fixed) override;
      void applyVelocitiesInPlace(TreeBaseType& target,
                                  const State& s, bool skip_leaves=false) override;

    protected:
      void _buildQuadraticForm(bool stats_mode=false) override;
      void _buildQuadraticFormDual(bool stats_mode=false) override;
      
    };

  }
} // namespace kd_slam::cticp
