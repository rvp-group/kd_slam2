#pragma once
#include "icp_.h"
#include "kd_slam/tree/tree_cpu_.h"


namespace kd_slam { namespace icp {

// cpu implementation of KD_ICP, specializes the base class
// and operates on KDTrees on cpu
// usage:
// 1. create an instance
// 2. assign a moving anf a fixed (setMoving, setFixed)
// 3. set an initial guess in X
// 4. start calling oneRound for each iteration;

template <typename Base_>
struct ICP_CPU_: public Base_ {
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
  using KDTreeType          = TreeCPU_<Tree_<NodeType>>;
  using StatsType           = typename Base::StatsType;
  using DiagPoseType        = typename Base::DiagPoseType;
  using PointsVectorType    = std::vector<PointType>;
  using NodesVectorType     = std::vector<NodeType>;
  PARAM(srrg2_core::PropertyInt,   num_threads,        "max threads 0/1: disable mt", 0, nullptr);
  
  static constexpr bool IsGPU=false;
  bool isGPU() const override {return IsGPU;}

  
  // inputs
  void setMoving(const TreeBaseType& moving) override;
  void setFixed(const TreeBaseType& fixed) override;
  
protected:
  void _buildQuadraticForm() override;

};

} } // namespace kd_slam::icp
