#pragma once
#include "multiview_cticp_factor_.h"

namespace kd_slam {
  namespace map {

    template <typename Node_>
    void MultiViewCTICPFactor_<Node_>::compute(bool chi_only, bool force) {
      if (!_enabled)
        return;
      if (!this->isActive() && !force)
        return;
      if (!kd_factor)
        throw std::runtime_error("MultiViewCTICPFactor: no kd_factor");

      auto tree_fixed  = kd_factor->tree_fixed;
      auto tree_moving = kd_factor->tree_moving;
      auto cticp       = kd_factor->cticp;
      if (!tree_fixed)  throw std::runtime_error("MultiViewCTICPFactor: no fixed cloud");
      if (!tree_moving) throw std::runtime_error("MultiViewCTICPFactor: no moving cloud");
      if (!cticp)       throw std::runtime_error("MultiViewCTICPFactor: no cticp engine");

      auto& x0 = variables().template at<0>(); // pose_A
      auto& v0 = variables().template at<1>(); // vel_A
      auto& x1 = variables().template at<2>(); // pose_B
      auto& v1 = variables().template at<3>(); // vel_B

      cticp->setFixed(*tree_fixed);
      cticp->setMoving(*tree_moving);
      cticp->fixed_state.X           = x0->estimate();
      cticp->fixed_state.velocities  = v0->estimate();
      cticp->moving_state.X          = x1->estimate();
      cticp->moving_state.velocities = v1->estimate();
      cticp->buildQuadraticFormDual();

      auto& m          = cticp->stats;
      _enabled         = m.inlierRatio() > kd_factor->disable_inlier_ratio;
      kd_factor->is_enabled = _enabled;
      _stats.chi       = m.chi2;
      _measurement_dim = m.num_inliers;
      this->robustify();

      if (chi_only || !isValid())
        return;

      static constexpr int PDim = SolverVariableType::PerturbationDim;
      static constexpr int VDim = SolverVelocityVariableType::PerturbationDim;

      using HPP = Eigen::Matrix<float, PDim, PDim>;
      using HPV = Eigen::Matrix<float, PDim, VDim>;
      using HVP = Eigen::Matrix<float, VDim, PDim>;
      using HVV = Eigen::Matrix<float, VDim, VDim>;
      using bP  = Eigen::Matrix<float, PDim, 1>;
      using bV  = Eigen::Matrix<float, VDim, 1>;

      const HPP H_pp   = cticp->H.template block<PDim, PDim>(0,    0);
      const HPV H_pv   = cticp->H.template block<PDim, VDim>(0,    PDim);
      const HVV H_vv   = cticp->H.template block<VDim, VDim>(PDim, PDim);
      const bP  b_p    = cticp->b.template head<PDim>();
      const bV  b_v    = cticp->b.template tail<VDim>();

      const HPV& H_pVb  = cticp->H_dual_cross_pose_vel_B;
      const HVV& H_vAb  = cticp->H_dual_cross_vel_AB;
      const HVV& H_VbVb = cticp->H_dual_vel_B;
      const bV&  b_Vb   = cticp->b_dual_vel_B;

      // H blocks (upper-triangular, order: x0=fixed v0=fixed x1=moving v1=moving)
      { auto h = _H_blocks[this->template blockOffset<0,0>()]; if (h) Eigen::Map<HPP>(h->storage()) += H_pp; }
      { auto h = _H_blocks[this->template blockOffset<0,1>()]; if (h) Eigen::Map<HPV>(h->storage()) -= H_pVb; }
      { auto h = _H_blocks[this->template blockOffset<0,2>()]; if (h) Eigen::Map<HPP>(h->storage()) -= H_pp; }
      { auto h = _H_blocks[this->template blockOffset<0,3>()]; if (h) Eigen::Map<HPV>(h->storage()) -= H_pv; }
      { auto h = _H_blocks[this->template blockOffset<1,1>()]; if (h) Eigen::Map<HVV>(h->storage()) += H_VbVb; }
      { auto h = _H_blocks[this->template blockOffset<1,2>()]; if (h) Eigen::Map<HVP>(h->storage()) += H_pVb.transpose(); }
      { auto h = _H_blocks[this->template blockOffset<1,3>()]; if (h) Eigen::Map<HVV>(h->storage()) += H_vAb.transpose(); }
      { auto h = _H_blocks[this->template blockOffset<2,2>()]; if (h) Eigen::Map<HPP>(h->storage()) += H_pp; }
      { auto h = _H_blocks[this->template blockOffset<2,3>()]; if (h) Eigen::Map<HPV>(h->storage()) += H_pv; }
      { auto h = _H_blocks[this->template blockOffset<3,3>()]; if (h) Eigen::Map<HVV>(h->storage()) += H_vv; }

      // b blocks
      { auto b = _b_blocks[0]; if (b) Eigen::Map<bP>(b->storage()) += b_p; }
      { auto b = _b_blocks[1]; if (b) Eigen::Map<bV>(b->storage()) -= b_Vb; }
      { auto b = _b_blocks[2]; if (b) Eigen::Map<bP>(b->storage()) -= b_p; }
      { auto b = _b_blocks[3]; if (b) Eigen::Map<bV>(b->storage()) -= b_v; }
    }

    template <typename Node_>
    void MultiViewCTICPFactor_<Node_>::serialize(ObjectData& odata, IdContext& ctx)  {
      Identifiable::serialize(odata, ctx);
      odata.setInt("graph_id", BaseSolverType::graphId());
      odata.setBool("enabled", BaseSolverType::enabled());
      odata.setInt("level", this->level());

      ArrayData* adata = new ArrayData;
      for (int pos = 0; pos < BaseSolverType::NumVariables; ++pos) {
        adata->add((int) BaseSolverType::variableId(pos));
      }
      odata.setField("variables", adata);

    }

    
    template <typename Node_>
    void MultiViewCTICPFactor_<Node_>::deserialize(ObjectData& odata,
                                                 IdContext& context) {
      Identifiable::deserialize(odata, context);
      srrg2_solver::FactorBase::_graph_id = odata.getInt("graph_id");
      if (odata.getField("enabled")) {
        srrg2_solver::FactorBase::_enabled = odata.getBool("enabled");
      }
      if (odata.getField("level")) {
        srrg2_solver::FactorBase::setLevel(odata.getInt("level"));
      }
      ArrayData* adata = dynamic_cast<ArrayData*>(odata.getField("variables"));
      int pos          = 0;
      for (auto it = adata->begin(); it != adata->end(); ++it) {
        MultiViewCTICPFactor_<Node_>::_variables.setGraphId(pos, (*it)->getInt());
        ++pos;
      }
    }

    
  } // namespace slam
} // namespace kd_slam
