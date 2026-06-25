#include "map_.h"

namespace kd_slam {
  namespace map {
    
    template <typename Node_>
    bool Map_<Node_>::addFactor(PGOFactorPtr f) {
      FrameGraph::addFactor(f);
      if (! _factor_graph->variable(f->from_ref)
          || ! _factor_graph->variable(f->to_ref))
        return false;

      auto pgo = std::make_shared<SolverPGOFactorType>();
      pgo->setVariableId(0, f->from_ref);
      pgo->setVariableId(1, f->to_ref);
      pgo->setMeasurement(f->Z_from_to);
      pgo->setInformationMatrix(f->omega_from_to);
      _factor_graph->addFactor(pgo);
      f->solver_factor = pgo.get();
      return true;
    }
    

    template <typename Node_>
    bool Map_<Node_>::addFactor(MultiViewICPFactorPtr f) {
      using SolverICPFactorType = MultiViewICPFactor_<Node_>;
      FrameGraph::addFactor(f);
      auto from_frame = std::dynamic_pointer_cast<Frame>(frame(f->from_ref));
      auto to_frame   = std::dynamic_pointer_cast<Frame>(frame(f->to_ref));
      if (!from_frame || !to_frame)
        return false;
      if (!_factor_graph->variable(f->from_ref)
          || !_factor_graph->variable(f->to_ref))
        return false;

      auto sf = std::make_shared<SolverICPFactorType>();
      f->solver_factor=sf.get();
      sf->setVariableId(0, f->from_ref);
      sf->setVariableId(1, f->to_ref);
      sf->kd_factor = f.get();
      _factor_graph->addFactor(sf);
      return true;
    }

    template <typename Node_>
    bool Map_<Node_>::addFactor(MultiViewCTICPFactorPtr f) {
      using SolverCTICPFactorType = MultiViewCTICPFactor_<Node_>;
      FrameGraph::addFactor(f);
      auto from_frame = std::dynamic_pointer_cast<Frame>(frame(f->from_ref));
      auto to_frame   = std::dynamic_pointer_cast<Frame>(frame(f->to_ref));
      if (!from_frame || !to_frame)
        return false;
      if (!_factor_graph->variable(f->from_ref)
          || !_factor_graph->variable(f->to_ref))
        return false;

      auto sf = std::make_shared<SolverCTICPFactorType>();
      f->solver_factor=sf.get();
      sf->setVariableId(0, f->from_ref);
      sf->setVariableId(1, f->vel_from_ref);
      sf->setVariableId(2, f->to_ref);
      sf->setVariableId(3, f->vel_to_ref);
      sf->kd_factor = f.get();
      _factor_graph->addFactor(sf);
      return true;
    }

    
  }
}
