#pragma once
#include "multiview_icp_factor_.h"

namespace kd_slam {
  namespace map {
    using namespace srrg2_core;
    using namespace srrg2_solver;
    
    template <typename Node_>
    void MultiViewICPFactor_<Node_>::compute(bool chi_only, bool force) {
      if (!_enabled)
        return;
      if (!this->isActive() && !force)
        return;
      if (!kd_factor)
        throw std::runtime_error("MultiViewICPFactor: no kd_factor");

      auto tree_fixed  = kd_factor->tree_fixed;
      auto tree_moving = kd_factor->tree_moving;
      auto icp         = kd_factor->icp;
      if (!tree_fixed)  throw std::runtime_error("MultiViewICPFactor: no fixed cloud");
      if (!tree_moving) throw std::runtime_error("MultiViewICPFactor: no moving cloud");
      if (!icp)         throw std::runtime_error("MultiViewICPFactor: no icp engine");

      auto& x0 = variables().template at<0>();
      auto& x1 = variables().template at<1>();
      icp->setFixed(*tree_fixed);
      icp->setMoving(*tree_moving);
      icp->fixed_state.X  = x0->estimate();
      icp->moving_state.X = x1->estimate();
      icp->buildQuadraticForm();

      auto& m          = icp->stats;
      _enabled         = m.inlierRatio() > kd_factor->disable_inlier_ratio;
      kd_factor->is_enabled = _enabled;
      _stats.chi       = m.chi2;
      _measurement_dim = m.num_inliers;
      this->robustify();

      if (chi_only || !isValid())
        return;

      using HType = Eigen::Matrix<float, SolverVariableType::PerturbationDim,
                                         SolverVariableType::PerturbationDim>;
      using bType = Eigen::Matrix<float, SolverVariableType::PerturbationDim, 1>;

      for (int r = 0; r < 2; ++r) {
        for (int c = r; c < 2; ++c) {
          int h_idx = this->blockOffset(r, c);
          if (!_H_blocks[h_idx]) continue;
          Eigen::Map<HType> H_rc(_H_blocks[h_idx]->storage());
          if (r == c) H_rc += icp->H;
          else        H_rc -= icp->H;
        }
      }
      for (int r = 0; r < 2; ++r) {
        if (!_b_blocks[r]) continue;
        Eigen::Map<bType> b_r(_b_blocks[r]->storage());
        if (r == 0) b_r += icp->b;
        else        b_r -= icp->b;
      }
    }

    template <typename Node_>
    void MultiViewICPFactor_<Node_>::serialize(ObjectData& odata, IdContext& ctx)  {
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
    void MultiViewICPFactor_<Node_>::deserialize(ObjectData& odata,
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
        MultiViewICPFactor_<Node_>::_variables.setGraphId(pos, (*it)->getInt());
        ++pos;
      }
    }

  } // namespace slam
} // namespace kd_slam
