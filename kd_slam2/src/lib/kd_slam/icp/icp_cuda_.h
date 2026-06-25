#pragma once
#include <memory>
#include "icp_.h"
#include "kd_slam/tree/tree_cuda_.h"


namespace kd_slam { namespace icp {

template<typename ICP_BaseType_>
struct ICP_CUDAWorkspace_;

// cuda implementation of KD_ICP, specializes the base class
// and operates on KDTrees on GPU
// usage:
// 1. create an instance
// 2. assign a moving and a fixed (setMoving, setFixed)
// 3. set an initial guess in X
// 4. start calling oneRound for each iteration;

template <typename Base_>
struct ICP_CUDA_: public Base_ {
  using Base                = Base_;
  using Traits              = typename Base::Traits;
  using NodeType              = typename Traits::NodeType;
  using PointType           = typename Traits::PointType;
  using Scalar              = typename Traits::Scalar;
  using IsometryType        = typename Traits::IsometryType;
  using PerturbationPoseType = typename Traits::PerturbationPoseType;
  using JacobianPoseType    = typename Traits::JacobianPoseType;
  using HessianType         = typename Traits::HessianType;
  using CoefficientType     = typename Traits::CoefficientType;
  using TreeBaseType      = typename Base::TreeBaseType;
  using KDTreeType          = TreeCUDA_<Tree_<NodeType>>;
  using WorkspaceType       = ICP_CUDAWorkspace_<Base_>;

  static constexpr bool IsGPU=true;
  bool isGPU() const override {return IsGPU;}
  ICP_CUDA_();
  ~ICP_CUDA_();

  // inputs
  void setMoving(const TreeBaseType& moving) override;
  void setFixed(const TreeBaseType& fixed) override;
protected:
  void _buildQuadraticForm() override;
  WorkspaceType* _workspace=0;
};


} } // namespace kd_slam::icp
