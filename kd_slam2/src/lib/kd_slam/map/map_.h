#pragma once
#include <srrg_solver/solver_core/factor_graph.h>
#include "kd_slam/icp/tree_aligner_base_.h"
#include "kd_slam/frame/frame_tree_.h"
#include "kd_slam/descriptor/extractor_.h"
#include "kd_slam/descriptor/matcher_.h"
#include "kd_slam/frame/frame_graph.h"
#include "kd_slam/icp/icp_.h"
#include "kd_slam/cticp/cticp_.h"
#include "kd_slam/tree/node_traits_.h"
#include "map_common.h"
#include "map_traits_.h"

namespace kd_slam {
  namespace map {
    using namespace kd_slam::icp;
    using namespace kd_slam::frame;

    template <typename Node_>
    struct SLAM_;
    
    template <typename Node_>
    struct Map_: public FrameGraph {
      static constexpr int  MAX_FRAMES=1000000;
      
      using NodeType        = Node_;
      using Scalar          = typename NodeType::Scalar;
      static constexpr int  ExtractorLevel = NodeTraits_<Node_>::ExtractorLevel;
      using TreeBaseType    = Tree_<NodeType>;
      using AlignerBase     = TreeAlignerBase_<NodeType>;
      using ICPStats        = typename AlignerBase::Stats;
      using IsometryType    = typename AlignerBase::IsometryType;
      using PoseHessianType = typename AlignerBase::PoseHessianType;
      using FrameTree       = frame::FrameTree_<NodeType>;
      using FrameTreePtr    = std::shared_ptr<FrameTree>;
      using VelocityVectorType = typename NodeType::GeometryTraits::VelocityVectorType;
      using DescriptorExtractor = descriptor::Extractor_<TreeCPU_<TreeBaseType>, ExtractorLevel>;
      using DescriptorType      = typename DescriptorExtractor::DescriptorType;

      
      using SolverVariableType             = typename MapTraits_<Node_>::SolverVariableType;
      using SolverPGOFactorType            = typename MapTraits_<Node_>::SolverPGOFactorType;
      using SolverVelocityVariableType     = typename MapTraits_<Node_>::SolverVelocityVariableType;
      using SolverVelocityPriorFactorType  = typename MapTraits_<Node_>::SolverVelocityPriorFactorType;
      using SolverGravityPriorFactorType   = typename MapTraits_<Node_>::SolverGravityPriorFactorType;
      
      struct Frame : public FrameTree {
        using InformationMatrixType = PoseHessianType;
        DescriptorType descriptor;
        IsometryType   pose_in_world;
        VelocityVectorType velocity=VelocityVectorType::Zero();
        static constexpr int Dim = Node_::Dim;
        mutable InformationMatrixType marginal_covariance;
        mutable Scalar marginal_determinant;
        mutable Scalar distance_from_root;
        mutable int    hops_from_root = 0;
        bool imu_R_good = false;
        Eigen::Matrix<Scalar, Dim, Dim> imu_R = Eigen::Matrix<Scalar, Dim, Dim>::Identity();
        int frame_count=-1;
        SolverVariableType*            solver_variable       = nullptr;
        SolverVelocityVariableType*    solver_velocity       = nullptr;
        SolverVelocityPriorFactorType* solver_velocity_prior = nullptr;
        SolverGravityPriorFactorType*  solver_gravity_prior  = nullptr;
        using FrameTree::ts;
        using FrameTree::tree;
        Frame(double ts,
              std::shared_ptr<TreeBaseType> t,
              const DescriptorType& desc,
              const IsometryType& piw,
              const int fc=-1):
          FrameTree(ts, t),
          descriptor(desc),
          pose_in_world(piw),
          frame_count(fc)
        {}

        Frame(const FrameTree& ft,
              const DescriptorType& desc,
              const IsometryType& piw,
              const int fc=-1):
          FrameTree(ft),
          descriptor(desc),
          pose_in_world(piw),
          frame_count(fc)
        {}

        
        void save() {
          if (! tree) return;
          if (_temp_tree)
            throw std::runtime_error("already saved");
#ifdef HAVE_CUDA
          if (tree->isGPU()) {
            _temp_tree=std::make_shared<TreeCUDA_<Tree_<NodeType>>>(*tree);
          } else {
#endif
            _temp_tree=std::make_shared<TreeCPU_<Tree_<NodeType>>>(*tree);
#ifdef HAVE_CUDA
          }
#endif
        }

        void restore() {
          if (! _temp_tree) 
            throw std::runtime_error("no backup");
          if (tree && _temp_tree) {
            _temp_tree->copyNodes(*tree);
            _temp_tree.reset();
          }
        }
        void applyVelocity() {
          if (velocity!=VelocityVectorType::Zero() && tree) {
            tree->applyVelocitiesInPlace(velocity);
          } 
          velocity.setZero();
          if (solver_velocity_prior)
            solver_velocity_prior->setMeasurement(velocity);
       }
        std::shared_ptr<TreeBaseType> _temp_tree;
      };
      using FramePtr = std::shared_ptr<Frame>;

      struct SolverFactorBridge: public frame::FactorBase{
        SolverFactorBridge(KDFactorType t) {type=t;}
        srrg2_solver::FactorBase* solver_factor=nullptr;
        KDFactorType type;
     };
      using SolverFactorBridgePtr = std::shared_ptr<SolverFactorBridge>;
        
      struct PGOFactor : public SolverFactorBridge {
        using InformationMatrixType = PoseHessianType;
        ICPStats        stats;
        IsometryType    Z_from_to;
        IsometryType    Z_to_from;
        PoseHessianType omega_from_to;
        PoseHessianType omega_to_from;
        Scalar          det_from_to;
        Scalar          det_to_from;
        PGOFactor(): SolverFactorBridge(Odometry){}
      };
      using PGOFactorPtr = std::shared_ptr<PGOFactor>;

      struct MultiViewICPFactor : public SolverFactorBridge {
        using ICP  = typename icp::ICPType_<Node_>::ICP;
        using SLAM = SLAM_<Node_>;
        using Tree = Tree_<Node_>;
        ICP*  icp         = nullptr;
        Tree* tree_moving = nullptr;
        Tree* tree_fixed  = nullptr;
        Scalar disable_inlier_ratio = 1;
        MultiViewICPFactor(): SolverFactorBridge(ICPMultiView){}
      };
      using MultiViewICPFactorPtr = std::shared_ptr<MultiViewICPFactor>;

      struct MultiViewCTICPFactor : public SolverFactorBridge {
        using CTICP = typename cticp::CTICPType_<Node_>::ICP;
        using SLAM  = SLAM_<Node_>;
        using Tree  = Tree_<Node_>;
        CTICP* cticp        = nullptr;
        int    vel_from_ref = -1;
        int    vel_to_ref   = -1;
        Tree*  tree_moving  = nullptr;
        Tree*  tree_fixed   = nullptr;
        Scalar disable_inlier_ratio = 1;
        MultiViewCTICPFactor(): SolverFactorBridge(CTICPMultiView){}
      };
      
      using MultiViewCTICPFactorPtr = std::shared_ptr<MultiViewCTICPFactor>;
      
      // shadow copy of solver's graph
      std::shared_ptr<srrg2_solver::FactorGraph> _factor_graph=std::make_shared<srrg2_solver::FactorGraph>();

      void clear() override {
        _factor_graph->clear();
        FrameGraph::clear();
      }
      bool addFactor(PGOFactorPtr f);
      bool addFactor(MultiViewICPFactorPtr f);
      bool addFactor(MultiViewCTICPFactorPtr f);

    };

  } // namespace map
} // namespace kd_slam
