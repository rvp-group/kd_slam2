#pragma once
#include "icp_stats_.h"
#include "srrg_config/configurable.h"
#include "srrg_config/property_configurable.h"
#include <functional>
#include "kd_slam/tree/tree_.h"
#include <fstream>

namespace kd_slam {
  namespace icp {

    template <typename Scalar_>
    struct ICPParams_{
      using Scalar=Scalar_;
      Scalar kernel_threshold=0.1;
      Scalar damping=1;
      Scalar max_error=1;
      Scalar min_normal_cos=cos(25./180.*M_PI);
      int min_good_measurements=10;
      int max_iterations=20;
    };

    enum AlignerType {AlignerICP, AlignerCTICP};

    template <typename Node_>
    struct TreeAlignerBase_ : public srrg2_core::Configurable {
      using NodeType        = Node_;
      using Scalar          = typename NodeType::Scalar;
      using TreeBaseType    = Tree_<NodeType>;
      using Stats           = ICPStats_<Scalar>;
      static constexpr int Dim   = Node_::Dim;
      static constexpr int PPDim = Dim * (Dim + 1) / 2;
      using IsometryType    = Eigen::Transform<Scalar, Dim, Eigen::Isometry>;
      using PoseHessianType = Eigen::Matrix<Scalar, PPDim, PPDim>;
      using VelocityVectorType = typename NodeType::Traits::GeometryTraits::VelocityVectorType;
      using PerturbationPoseType = typename NodeType::Traits::GeometryTraits::PerturbationPoseType;
      using TerminationCriteria = ICPTerminationCriteriaBase_<Scalar>;
      
      PARAM(srrg2_core::PropertyFloat, kernel_threshold,      "chi2 kernel threshold",         0.1f,                      &_params_changed);
      PARAM(srrg2_core::PropertyFloat, damping,               "linear system damping",         1.0f,                      &_params_changed);
      PARAM(srrg2_core::PropertyFloat, max_error,             "max point-to-plane error",      1.0f,                      &_params_changed);
      PARAM(srrg2_core::PropertyFloat, min_normal_cos,        "min cosine of normal angle",    float(cos(25./180.*M_PI)), &_params_changed);
      PARAM(srrg2_core::PropertyInt,   min_good_measurements, "min inliers to accept a round", 10,                        &_params_changed);
      PARAM(srrg2_core::PropertyInt,   max_iterations,        "max ICP iterations",            20,                        &_params_changed);
      PARAM(srrg2_core::PropertyString, log_file, "file where to dump aligner log", "", &_params_changed);

      PARAM(srrg2_core::PropertyConfigurable_<TerminationCriteria>, term_crit, "termination criteria",  nullptr, &_params_changed);

      const TreeBaseType* _fixed  = nullptr;
      const TreeBaseType* _moving = nullptr;

      virtual void setFixed (const TreeBaseType& fixed)  = 0;
      virtual void setMoving(const TreeBaseType& moving) = 0;

      virtual bool oneRound() = 0;
      virtual AlignerType alignerType() const = 0;
      struct FixedEntry {
        Stats stats;
        FixedEntry() {}
        const TreeBaseType* fixed_tree = nullptr;
      };
      using FixedEntryPtr = std::shared_ptr<FixedEntry>;
      
      template <int Dim, typename State_>
      struct FixedEntry_ : public FixedEntry {
        using HessianType = Eigen::Matrix<Scalar, Dim, Dim>;
        using CoefficientType = Eigen::Matrix<Scalar, Dim, 1>;
        using State = State_;
        using ThisType = FixedEntry_<Dim, State>;
        HessianType H=HessianType::Zero();
        CoefficientType b=CoefficientType::Zero();
        State fixed_state;
        using FixedEntry::stats;
      };

      virtual FixedEntryPtr makeFixedEntry(const TreeBaseType& tree, const IsometryType& iso) = 0;
      virtual void syncParams() {
        if (! _log_stream && !param_log_file.value().empty()){
          _log_stream.reset(new std::ofstream(param_log_file.value()));
        }
        if (_log_stream && param_log_file.value().empty()){
          _log_stream.reset();
        }
      }

      // ---- pose state interface -------------------------------------------
      virtual IsometryType    getX()                        const = 0;
      virtual void            setX(const IsometryType& X)        = 0;
      virtual void            setFixedX(const IsometryType& X)   = 0;
      virtual PoseHessianType getPoseHessian()              const = 0;

      // ---- keyframe hook --------------------------------------------------
      virtual VelocityVectorType velocities() const { return VelocityVectorType::Zero(); }
      virtual void setVelocities(const VelocityVectorType& v ) {}
      virtual void resetMovingState() {}
      virtual void buildQuadraticForm() = 0;
      virtual void buildQuadraticForm(FixedEntry& fixed) {}
      virtual void saveStartState() = 0;
      
      int compute() {
        syncParams();
        stats_log.clear();
        stats_log.reserve(param_max_iterations.value());
        saveStartState();
        for (int i = 0; i < param_max_iterations.value(); ++i) {
          bool result = oneRound();
          stats_log.push_back(stats);
          if (!result)
            return -i;
          if (param_term_crit.value() && param_term_crit->hasToStop(stats, i))
            return i;
        }
        return param_max_iterations.value();
      }

      virtual bool isGPU() const = 0;
      virtual ~TreeAlignerBase_() = default;
      virtual std::string iterationLogHeader() const = 0;
      void dumpIterationLog() {
        if (!_log_stream)
          return;
        *_log_stream << iterationLogHeader() << std::endl;
        for (size_t i=0; i<stats_log.size(); ++i)
          *_log_stream << "  iter: " << i << " stats: " << stats_log[i] << std::endl;     }

      void setPosePrior(const IsometryType& X_ref=IsometryType::Identity(),
                        const IsometryType& Z=IsometryType::Identity(),
                        const PoseHessianType& info=PoseHessianType::Zero()){
        _pose_prior_X_ref=X_ref;
        _pose_prior_Z=Z;
        _pose_prior_info=info;
      }
      Stats stats;
      std::vector<Stats> stats_log;
      std::unordered_map<int, FixedEntryPtr> _fixed_forest;
      std::unique_ptr<std::ostream>& logStream() {return _log_stream;}
    protected:
      void posePriorBuildQuadraticForm() {
        if (_pose_prior_info==PoseHessianType::Zero()) {
          _pose_prior_H.setZero();
          _pose_prior_b.setZero();
          _pose_prior_chi2=0;
          return;
        }
        IsometryType ZXi=(_pose_prior_X_ref*_pose_prior_Z).inverse();
        PerturbationPoseType e = NodeType::Traits::GeometryTraits::poseToPoseError(ZXi,getX());
        PoseHessianType J = NodeType::Traits::GeometryTraits::dPoseToPoseError(ZXi,getX());
        _pose_prior_H=J.transpose()*_pose_prior_info*J;
        _pose_prior_b=J.transpose()*_pose_prior_info*e;
        _pose_prior_chi2=e.transpose()*_pose_prior_info*e;
      }
      bool _params_changed = true;
      mutable std::unique_ptr<std::ostream> _log_stream;
      IsometryType _pose_prior_X_ref=IsometryType::Identity();
      IsometryType _pose_prior_Z=IsometryType::Identity();
      PoseHessianType _pose_prior_info=PoseHessianType::Zero();
      PoseHessianType _pose_prior_H=PoseHessianType::Zero();
      PerturbationPoseType _pose_prior_b=PerturbationPoseType::Zero();
      Scalar _pose_prior_chi2=0;
    };
  }
}
