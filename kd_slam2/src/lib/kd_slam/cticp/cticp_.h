#pragma once
#include <iostream>
#include <sstream>
#include <iomanip>
#include "kd_slam/tree/tree_.h"
#include "kd_slam/tree/tree_predicates_.h"
#include "kd_slam/icp/icp_.h"


namespace kd_slam {
  namespace cticp {
    using namespace kd_slam::icp;

    template <typename Scalar_>
    struct CTICPParams_: public ICPParams_<Scalar_> {
      using Scalar = Scalar_;
      Scalar velocity_damping=100;
    };

    template <typename Traits_>
    struct CTICP_ : public TreeAlignerBase_<typename Traits_::NodeType>, public Traits_ {
      // ---- inner types -------------------------------------------------------
      using Traits               = Traits_;
      using NodeType               = typename Traits::NodeType;
      using PointTraits          = typename Traits::PointTraits;
      using PointType            = typename Traits::PointType;
      using VectorType           = typename Traits::VectorType;
      using Scalar               = typename Traits::Scalar;
      using IsometryType         = typename Traits::IsometryType;
      using PerturbationPoseType = typename Traits::PerturbationPoseType;
      using JacobianPoseType     = typename Traits::JacobianPoseType;
      using VelocityType         = typename Traits::VelocityType;
      using JacobianVelocityType = typename Traits::JacobianVelocityType;
      using HessianType          = typename Traits::HessianType;
      using CoefficientType      = typename Traits::CoefficientType;
      using ParamsType           = CTICPParams_<Scalar>;
      using TreeBaseType       = Tree_<NodeType>;
      using PoseHessianType    = typename TreeAlignerBase_<NodeType>::PoseHessianType;

      // ---- workspace sub-item types ------------------------------------------
      using StatsType         = ICPWsItemStats_<Traits>;
      using DiagPoseType      = ICPWsItemDiag_<JacobianPoseType>;
      using DiagVelType       = ICPWsItemDiag_<JacobianVelocityType>;
      using CrossType         = ICPWsItemCross_<JacobianPoseType, JacobianVelocityType>;
      // Dual-specific: J_A_vel^T J_B_vel (the only type not already covered above;
      // DiagVelType reused for B-vel diag, CrossType reused for J_A_pose^T J_B_vel)
      using CrossVelAVelBType = ICPWsItemCross_<JacobianVelocityType, JacobianVelocityType>;

      using AlignerBaseType = TreeAlignerBase_<typename Traits_::NodeType>;
      
      static constexpr AlignerType aligner_type=AlignerCTICP;
      using Traits::adjoint;
      using Traits::transformOmega;
      using Traits::GeometryTraits::poseToPoseError;
      using Traits::GeometryTraits::dPoseToPoseError;

      using AlignerBaseType::_fixed_forest;
      using AlignerBaseType::_pose_prior_Z;
      using AlignerBaseType::_pose_prior_info;

      PARAM(srrg2_core::PropertyFloat, velocity_damping, "velocity regularization weight", 100.f, &this->_params_changed);

      AlignerType alignerType() const override {return aligner_type;}

      CTICP_() {
        moving_state.X.setIdentity();
        moving_state.velocities.setZero();
        fixed_state.X.setIdentity();
        fixed_state.velocities.setZero();
      }
      ParamsType params;

      void syncParams() override {
        if (!this->_params_changed)
          return;
        AlignerBaseType::syncParams();
        params.kernel_threshold      = this->param_kernel_threshold.value();
        params.damping               = this->param_damping.value();
        params.max_error             = this->param_max_error.value();
        params.min_normal_cos        = this->param_min_normal_cos.value();
        params.min_good_measurements = this->param_min_good_measurements.value();
        params.max_iterations        = this->param_max_iterations.value();
        params.velocity_damping      = param_velocity_damping.value();
        this->_params_changed = false;
      }

      VelocityType velocities() const override {return moving_state.velocities;}
      void setVelocities(const VelocityType& v ) override { moving_state.velocities = v;}

      // ---- destination view (pointer bundle written by quadraticTerm) --------
      struct DestView {
        StatsType*    stats;
        DiagPoseType* diag_pose;
        DiagVelType*  diag_vel;
        CrossType*    cross;

        __CUDA_EXPORT_INLINE__ void clear(int tid) {
          stats[tid].clear();
          diag_pose[tid].clear();
          diag_vel[tid].clear();
          cross[tid].clear();
        }
      };

      // ---- dual destination view (adds B-velocity terms) ---------------------
      // J_B_pose = -J_A_pose (symmetric error), so B-pose blocks are derivable.
      // Only J_B_vel is genuinely new and requires separate accumulation.
      struct DestViewDual {
        StatsType*         stats;
        DiagPoseType*      diag_pose;        // A-pose (same as non-dual)
        DiagVelType*       diag_vel;         // A-vel  (same as non-dual)
        CrossType*         cross;            // J_A_pose^T J_A_vel (same as non-dual)
        DiagVelType*       diag_vel_B;       // J_B_vel^T J_B_vel + J_B_vel^T e
        CrossType*         cross_pose_vel_B; // J_A_pose^T J_B_vel  (CrossType = PDimxVDim)
        CrossVelAVelBType* cross_vel_AB;     // J_A_vel^T  J_B_vel

        __CUDA_EXPORT_INLINE__ void clear(int tid) {
          stats[tid].clear();
          diag_pose[tid].clear();
          diag_vel[tid].clear();
          cross[tid].clear();
          diag_vel_B[tid].clear();
          cross_pose_vel_B[tid].clear();
          cross_vel_AB[tid].clear();
        }
      };

      static constexpr int PerturbationPoseDim = Traits::PerturbationPoseDim;
      static constexpr int VelocityDim         = Traits::VelocityDim;
      static constexpr int OptimizationDim     = Traits::OptimizationDim;

      // ---- estimated state: pose + velocity ----------------------------------
      // Points are assumed to be pre-transformed into robot frame before
      // building the KD-tree, so no sensor_in_robot offset is needed here.
      struct State {
        IsometryType X          = IsometryType::Identity();
        VelocityType velocities = VelocityType::Zero();
      };

      using FixedEntryBase = typename AlignerBaseType::FixedEntry;
      using FixedEntry  = typename AlignerBaseType::FixedEntry_<Traits::OptimizationDim, State>;
      using FixedEntryPtr = std::shared_ptr<FixedEntry>;

      std::shared_ptr<FixedEntryBase> makeFixedEntry(const TreeBaseType& tree, const IsometryType& iso) override {
        FixedEntry entry;
        entry.fixed_state.X=iso;
        entry.fixed_tree=&tree;
        return std::make_shared<FixedEntry>(entry);
      }

      // ---- per-iteration cache (precomputed from both states) ----------------
      struct QuadraticTermCache {
        IsometryType Xm;   // moving_state.X  (robot pose, moving cloud in robot frame)
        IsometryType Xf;   // fixed_state.X   (robot pose, fixed cloud in robot frame)
        IsometryType iXf;  // Xf.inverse()
        IsometryType Xmf;  // iXf * Xm
      };

      State moving_state;
      State fixed_state;

      // ---- outputs -----------------------------------------------------------
      HessianType     H;
      CoefficientType b;
      CoefficientType dx;

      // returns the pose-only (PDim x PDim) information matrix via Schur complement,
      // marginalizing out the velocity block
      PoseHessianType poseHessian() const {
        static constexpr int PDim = Traits::PerturbationPoseDim;
        static constexpr int VDim = Traits::VelocityDim;
        const auto H_pp = H.template block<PDim, PDim>(0,    0);
        const auto H_pv = H.template block<PDim, VDim>(0,    PDim);
        const auto H_vv = H.template block<VDim, VDim>(PDim, PDim);
        using MatVV = Eigen::Matrix<Scalar, VDim, VDim>;
        return H_pp - H_pv * (H_vv + Scalar(1e-9) * MatVV::Identity()).inverse() * H_pv.transpose();
      }

      // ---- dual outputs (populated by buildQuadraticFormDual) --------------- (populated by buildQuadraticFormDual).
      // B-pose blocks are derivable: H_BB_pose = H.block<PDim,PDim>(0,0),
      //   b_B_pose = -b.head<PDim>(), H_AB_pose = -H.block<PDim,PDim>(0,0).
      // B-vel blocks require separate accumulation:
      typename DiagVelType::HessianType      H_dual_vel_B;            // VDimxVDim
      typename DiagVelType::CoefficientType  b_dual_vel_B;            // VDim
      typename CrossType::HessianType        H_dual_cross_pose_vel_B; // PDimxVDim  J_A_pose^T J_B_vel
      typename CrossVelAVelBType::HessianType H_dual_cross_vel_AB;    // VDimxVDim  J_A_vel^T  J_B_vel

      // ---- core interface ----------------------------------------------------
      static __CUDA_EXPORT_INLINE__
      void quadraticTerm(DestView& dest, int tid,
                         const State& fixed_state,
                         const State& moving_state,
                         const QuadraticTermCache& cache,
                         const NodeType* fixed_nodes_ptr,
                         const NodeType& moving_leaf,
                         const ParamsType& params,
                         bool stats_mode=false);

      // Like quadraticTerm but also computes J_B_vel via errorAndJacobianVelocityB.
      // Populates all DestViewDual fields in a single pass.
      static __CUDA_EXPORT_INLINE__
      void quadraticTermDual(DestViewDual& dest, int tid,
                             const State& fixed_state,
                             const State& moving_state,
                             const QuadraticTermCache& cache,
                             const NodeType* fixed_nodes_ptr,
                             const NodeType& moving_leaf,
                             const ParamsType& params,
                         bool stats_mode=false);

      bool oneRound(bool stats_mode=false) override;
      std::ostream& printStatus(std::ostream& os);

      // applies the velocity motion-compensation to all nodes of a target cloud
      virtual void applyVelocitiesInPlace(TreeBaseType& target,
                                          const State& s, bool skip_leaves=false) = 0;


      // ---- TreeAlignerBase_ virtual implementations --------------------------
      IsometryType    getX()               const override { return moving_state.X; }
      void            setX(const IsometryType& X) override { moving_state.X = X; }
      void            setFixedX(const IsometryType& X) override {
        fixed_state.X = X;
        fixed_state.velocities.setZero();
      }
      PoseHessianType getPoseHessian()     const override { return poseHessian(); }


      // void prepareFrame(TreeBaseType& t) override {
      //   applyVelocitiesInPlace(t, moving_state);
      // }
      void resetMovingState() override { moving_state.velocities.setZero(); }

      virtual ~CTICP_() {}
      QuadraticTermCache _cache;

      void buildQuadraticForm(bool stats_mode=false) override;

      // Single-pass dual: populates H, b (standard) + dual B-vel outputs.
      void buildQuadraticFormDual(bool stats_mode=false);

      void buildQuadraticForm(FixedEntryBase& fixed, bool stats_mode=false) override;

    protected:
      void updateCache();
      std::string iterationLogHeader() const override;

      void saveStartState() override {
        _saved_start_moving_state=moving_state;
        _saved_start_fixed_state=fixed_state;
      }

      virtual void _buildQuadraticForm(bool stats_mode=false) = 0;

      // Single-pass dual: populates H, b (standard) + dual B-vel outputs.
      virtual void _buildQuadraticFormDual(bool stats_mode=false) = 0;
      FixedEntry _acc_fixed;
      State _saved_start_moving_state;
      State _saved_start_fixed_state;

    };

    template <typename Node_>
    struct CTICPType_{
      using NodeType=Node_;
    };

  }
} // namespace kd_slam::cticp
