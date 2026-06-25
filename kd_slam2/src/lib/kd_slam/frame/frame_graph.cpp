#include "frame_graph.h"
#include <limits>
#include <queue>
#include <stdexcept>
#include <functional>
#include <iostream>

namespace kd_slam {
  namespace frame {
    using namespace std;
    
    void FrameGraph::clear() {
      _frame_to_factor.clear();
      _frames.clear();
      _factors.clear();
    }
    void FrameGraph::addFactor(FactorBasePtr factor) {
      if (factor->from_ref==factor->to_ref) {
        cerr << "from_Ref: " << factor->from_ref << " to_ref: "<< factor->to_ref << endl;
        throw std::runtime_error("FrameGraph| addFactor, from==to");
      }

      // first check that the frames in the factor exist;
      auto it_from=_frames.find(factor->from_ref);
      if (it_from==_frames.end())
        throw std::runtime_error("FrameGraph| addFactor, non existing 'from' frame");
      auto it_to=_frames.find(factor->to_ref);
      if (it_to==_frames.end())
        throw std::runtime_error("FrameGraph| addFactor, non existing 'to' frame");

      auto it = _factors.find(factor);
      if (it!=_factors.end())
        throw std::runtime_error("FrameGraph| addFactor, factor in graph");
      _factors.insert(factor);
      _frame_to_factor[factor->from_ref].insert(factor);
      _frame_to_factor[factor->to_ref].insert(factor);
    }

    void FrameGraph::removeFactor(FactorBasePtr factor) {
      auto it=_factors.find(factor);
      if (it==_factors.end())
        throw std::runtime_error("no factor");
      _factors.erase(factor);
      _frame_to_factor[factor->from_ref].erase(factor);
      _frame_to_factor[factor->to_ref].erase(factor);
    }

    void FrameGraph::addFrame(FrameBasePtr frame) {
      if (frame->_ref>=0)
        throw std::runtime_error("FrameGraph| addFrame frame not in detached state");
      int new_ref=0;
      if (! _frames.empty()) {
        new_ref=_frames.rbegin()->first+1;
      }
      frame->_ref=new_ref;
      _frames[new_ref]=frame;
      _frame_to_factor[new_ref]=FactorSet();
    }
    
    void FrameGraph::removeFrame(FrameBasePtr frame) {
      if (frame->_ref<0)
        throw std::runtime_error("FrameGraph| removeFrame in detached state");
      auto it=_frames.find(frame->_ref);
      if (it==_frames.end())
        throw std::runtime_error("FrameGraph| frame to remove not in graph");
      if (it->second!=frame)
        throw std::runtime_error("FrameGraph| removeFrame, frame ptr mismatch");
      auto f_it=_frame_to_factor.find(frame->_ref);
      if (f_it!=_frame_to_factor.end()) {
        auto& m=f_it->second;
        while (! m.empty()){
          auto& f=*m.begin();
          removeFactor(f);
        }
      }
      frame->_ref=-1;
      _frames.erase(it);
    }

    void FrameGraph::visitBFS(int src_ref, VisitActions& actions) {
      // cost management
      struct CostPayload {
        double cost;
        int prev_ref;
        FactorBasePtr factor;
      };
      using CostMap=std::map<int, CostPayload>;
      CostMap costs;
      auto cost = [&](int ref) {
        auto it=costs.find(ref);
        if (it==costs.end())
          return std::numeric_limits<double>::max();
        return it->second.cost;
      };

      auto setCost= [&](int ref, double cost=0, int prev_ref=-1, FactorBasePtr f=nullptr) {
        CostPayload cp= {cost, prev_ref, f};
        costs[ref]=cp;
      };

      // priority queue
      using ExpandedNode=std::pair<double, int>;
      struct PQExpansionCompare {
        inline bool operator()(const ExpandedNode&a, const ExpandedNode& b) {
          return a.first>b.first;
        }
      };
      using PriorityQueue=std::priority_queue<ExpandedNode, std::vector<ExpandedNode>, PQExpansionCompare>;
      PriorityQueue q;

      // initialization
      auto current=frame(src_ref);
      if (! current)
        throw std::runtime_error("FrameGraph| visitBFS, non existing source");
      actions.init(*this);
      costs.clear();
      setCost(current->ref(), 0);

      

      q.push(ExpandedNode(0, current->ref()));
      while (! q.empty()&&!actions.stop()) {
        auto [current_cost, current_ref]=q.top();
        q.pop();
        if (current_cost > cost(current_ref)) 
           continue;
        current=frame(current_ref);
        auto& facts=factors(current_ref);
        for (auto& f: facts) {
          int next_ref = f->from_ref==current_ref ? f->to_ref : f->from_ref;
          auto next= frame(next_ref);
          double trav_cost=actions.traversalCost(*current, *next, *f);
          double next_cost=cost(next_ref);
          double new_next_cost=current_cost+trav_cost;
          if (new_next_cost>=next_cost)
            continue;
          setCost(next_ref, new_next_cost, current_ref, f);
          q.push(ExpandedNode(new_next_cost, next_ref));
        }
      }

      // done with the visit. computing applyTraversal on all the tree;

      struct BFSEdge {
        int next_key;
        FactorBasePtr f;
        double cost;
      };

      std::multimap<int, BFSEdge> bfs_edges;

      for (auto& it: costs) {
        auto to_ref=it.first;
        auto from_ref=it.second.prev_ref;
        auto cost=it.second.cost;
        auto fact=it.second.factor;
        BFSEdge e={to_ref, fact, cost};
        bfs_edges.insert(std::make_pair(from_ref,e));
      }

      std::function<void(int)> visitDF;

      visitDF=[&](int node_key)->void {
        auto node=frame(node_key);
        auto [first, last] =bfs_edges.equal_range(node_key);
        while (first!=last) {
          BFSEdge& e=first->second;
          auto next_node=frame(e.next_key);
          actions.traverseFactor(*node, *next_node, *e.f);
          visitDF(e.next_key);
          ++first;
        }
      };

      visitDF(src_ref);
    }
    
  }
}
