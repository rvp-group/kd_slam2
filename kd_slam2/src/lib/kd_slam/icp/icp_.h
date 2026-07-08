#pragma once
#include "tree_aligner_base_.h"
#include "icp_ws_item_.h"

namespace kd_slam {
  namespace icp {

    template <typename Traits_>
    struct ICP_: public TreeAlignerBase_<typename Traits_::NodeType>, public Traits_ {
      // ---- inner types ---------------------------------------------------------
      using Traits           = Traits_;
      using NodeType           = typename Traits::NodeType;
      using PointTraits      = typename Traits::PointTraits;
      using PointType        = typename Traits::PointType;
      using VectorType       = typename Traits::VectorType;
      using Scalar           = typename Traits::Scalar;
      using IsometryType     = typename Traits::IsometryType;
      using PerturbationPoseType = typename Traits::PerturbationPoseType;
      using JacobianPoseType = typename Traits::JacobianPoseType;
      using HessianType      = typename Traits::HessianType;
      using CoefficientType  = typename Traits::CoefficientType;
      using ParamsType       = ICPParams_<Scalar>;
      using TreeBaseType   = Tree_<NodeType>;
      using PoseHessianType = typename TreeAlignerBase_<NodeType>::PoseHessianType;
      // ---- workspace sub-item types --------------------------------------------
      using StatsType    = ICPWsItemStats_<Traits>;
      using DiagPoseType = ICPWsItemDiag_<JacobianPoseType>;
      using AlignerBaseType = TreeAlignerBase_<typename Traits_::NodeType>;
            
      static constexpr AlignerType aligner_type=AlignerICP;
      virtual bool isGPU() const =0;
      AlignerType alignerType() const override {return aligner_type;}
     
      using Traits::adjoint;
      using Traits::transformOmega;
      using Traits::GeometryTraits::poseToPoseError;
      using Traits::GeometryTraits::dPoseToPoseError;
      using AlignerBaseType::_fixed_forest;
      using AlignerBaseType::_pose_prior_Z;
      using AlignerBaseType::_pose_prior_info;
      
      // ---- TreeAlignerBase_ virtual implementations -----------------------
      IsometryType    getX()               const override { return moving_state.X; }
      void            setX(const IsometryType& X) override { moving_state.X = X; }
      void            setFixedX(const IsometryType& X) override { fixed_state.X = X; }
      // For ICP: HessianType == PoseHessianType (PPDim == PerturbationPoseDim)
      PoseHessianType getPoseHessian()     const override { return H; }
      
      ICP_() {
        moving_state.X.setIdentity();
        fixed_state.X.setIdentity();
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
        this->_params_changed = false;
      }

      // ---- destination view (pointer bundle written by quadraticTerm) ----------
      struct DestView {
        StatsType*    stats;
        DiagPoseType* diag_pose;

        __CUDA_EXPORT_INLINE__ void clear(int tid) {
          stats[tid].clear();
          diag_pose[tid].clear();
        }
      };



      // ---- estimated state -----------------------------------------------------
      struct State {
        IsometryType X = IsometryType::Identity();
      };

      using FixedEntryBase = typename AlignerBaseType::FixedEntry;
      using FixedEntry  = typename AlignerBaseType::FixedEntry_<Traits::PerturbationPoseDim, State>;
      using FixedEntryPtr = std::shared_ptr<FixedEntry>;

      std::shared_ptr<FixedEntryBase> makeFixedEntry(const TreeBaseType& tree, const IsometryType& iso) override {
        FixedEntry entry;
        entry.fixed_state.X=iso;
        entry.fixed_tree=&tree;
        return std::make_shared<FixedEntry>(entry);
      }
      
      struct QuadraticTermCache {
        IsometryType Xm;
        IsometryType Xf;
        IsometryType iXf;
        IsometryType Xmf;
      };


      State moving_state;
      State fixed_state;

      // ---- outputs -------------------------------------------------------------
      HessianType     H;
      CoefficientType b;
      CoefficientType dx;

      // returns the pose-only information matrix (6x6); for ICP this is H itself
      const HessianType& poseHessian() const { return H; }

      // ---- dual outputs (populated by buildQuadraticFormDual) ------------------
      // By symmetry of e = (pA-pB).(nA+nB): J_B = -J_A always.
      // Hence H_fixed = H, b_fixed = -b, H_cross (J_A^T J_B) = -H.
      HessianType     H_fixed;   // = H
      CoefficientType b_fixed;   // = -b
      HessianType     H_cross;   // = -H  (J_A^T J_B off-diagonal block)

      // ---- core interface ------------------------------------------------------
      static __CUDA_EXPORT_INLINE__
      void quadraticTerm(DestView& dest, int tid,
                         const State& fixed_state,
                         const State& moving_state,
                         const QuadraticTermCache& cache,
                         const NodeType* fixed_nodes_ptr,
                         const NodeType& moving_leaf,
                         const ParamsType& params,
                         bool stats_mode=false);

      void buildQuadraticForm(bool stats_mode=false) override;
      void buildQuadraticForm(FixedEntryBase& fixed, bool stats_mode=false) override;
      // Non-virtual: delegates to buildQuadraticForm() then fills H_fixed/b_fixed/H_cross
      // using J_B = -J_A symmetry -- no extra workspace needed.
      void buildQuadraticFormDual(bool stats_mode=false);
      bool oneRound(bool stats_mode=false) override;
      std::ostream& printStatus(std::ostream& os);
      virtual ~ICP_();
      void saveStartState() override {
        _saved_start_moving_state=moving_state;
        _saved_start_fixed_state=fixed_state;
      }
      std::string iterationLogHeader() const override;
    protected:
      void updateCache();
      virtual void _buildQuadraticForm(bool stats_mode=false) = 0;
      QuadraticTermCache _cache;
      FixedEntry _acc_fixed;
      State _saved_start_moving_state;
      State _saved_start_fixed_state;
    };

    template <typename Node_>
    struct ICPType_{
      using NodeType=Node_;
    };
  }
} // namespace kd_slam::icp
