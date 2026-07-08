#pragma once
#include <memory>
#include "cticp_.h"
#include "kd_slam/tree/tree_cuda_.h"


namespace kd_slam { namespace cticp {
    using namespace kd_slam::icp;

    template<typename CTICP_BaseType_>
    struct CTICP_CUDAWorkspace_;

    // cuda implementation of KD_CTICP, specializes the base class
    // and operates on KDTrees on GPU
    // usage:
    // 1. create an instance
    // 2. assign a moving and a fixed (setMoving, setFixed)
    // 3. set initial guess in moving_state (X + velocities)
    // 4. start calling oneRound for each iteration

    template <typename Base_>
    struct CTICP_CUDA_ : public Base_ {
      using Base                 = Base_;
      using Traits               = typename Base::Traits;
      using NodeType               = typename Traits::NodeType;
      using PointType            = typename Traits::PointType;
      using Scalar               = typename Traits::Scalar;
      using IsometryType         = typename Traits::IsometryType;
      using PerturbationPoseType = typename Traits::PerturbationPoseType;
      using JacobianPoseType     = typename Traits::JacobianPoseType;
      using HessianType          = typename Traits::HessianType;
      using CoefficientType      = typename Traits::CoefficientType;
      using TreeBaseType       = typename Base::TreeBaseType;
      using KDTreeType           = TreeCUDA_<Tree_<NodeType>>;
      using WorkspaceType        = CTICP_CUDAWorkspace_<Base_>;
      using State                = typename Base::State;

      static constexpr bool IsGPU=true;
      bool isGPU() const override {return IsGPU;}

      void setMoving(const TreeBaseType& moving) override;
      void setFixed(const TreeBaseType& fixed) override;
      void applyVelocitiesInPlace(TreeBaseType& target,
                                  const State& s, bool skip_leaves=false) override;
  
      WorkspaceType* _workspace = 0;
      CTICP_CUDA_();
      ~CTICP_CUDA_();
    protected:
      void _buildQuadraticForm(bool stats_mode=false) override;
      void _buildQuadraticFormDual(bool stats_mode=false) override;
  
    };

  }
} // namespace kd_slam::cticp
