#pragma once
#include <map>
#include <set>
#include <memory>
#include <stdexcept>
#include "frame_base.h"
#include "factor_base.h"
#include <unordered_set>

namespace kd_slam {
  namespace frame {
    
    struct IndexPair {
      int from_ref;
      int to_ref;
      bool operator==(const IndexPair& other) const {
        return from_ref==other.from_ref && to_ref==other.to_ref;
      }
      bool operator<(const IndexPair& other) const {
        return from_ref<other.from_ref ||
          (from_ref==other.from_ref && to_ref<=other.to_ref);
      }
    };
    struct IndexPairHash {
      size_t operator()(const IndexPair& ip) const {
        return (((size_t)ip.from_ref) <<32 )|((size_t)ip.to_ref);
      }
    };
    using IndexPairSet = std::unordered_set<IndexPair, IndexPairHash>;

    
    struct FrameGraph {
      using FrameMap  = std::map <int, FrameBasePtr>;
      using FactorSet = std::set<FactorBasePtr>;
      using FrameFactorMap=std::map<int, FactorSet>;


      inline FrameMap& frames() {return _frames;}
      inline const FrameMap& frames() const {return _frames;}

      inline FrameBasePtr frame(int ref) {
        auto it = _frames.find(ref);
        if (it==_frames.end())
          return nullptr;
        return it->second;
      }

      inline const FrameBasePtr frame(int ref) const {
        auto it = _frames.find(ref);
        if (it==_frames.end())
          return nullptr;
        return it->second;
      }

      inline FactorSet& factors(int ref) {
        if (!frame(ref)){
          throw std::runtime_error("FrameGraph| factors(key), no frame");
        }
        return _frame_to_factor[ref];
      }

      inline const FactorSet& factors(int ref) const {
        if (!frame(ref)){
          throw std::runtime_error("FrameGraph| factors(key), no frame");
        }
        const auto& it=_frame_to_factor.find(ref);
        return it->second;
      }

      inline const FactorSet& factors() const {
        return _factors;
      }

      inline FactorSet& factors() {
        return _factors;
      }
  
      void addFactor(FactorBasePtr factor);
      void removeFactor(FactorBasePtr factor);
      void addFrame(FrameBasePtr frame);    
      void removeFrame(FrameBasePtr frame);    
      virtual void clear();
      struct VisitActions {
        //called once before the search
        void init (FrameGraph& graph) {_graph=&graph;}
        // returns the cost of 
        virtual double traversalCost(FrameBase& src, FrameBase& dest, FactorBase& factor) =0;
        virtual void traverseFactor(FrameBase& src, FrameBase& dest, FactorBase& factor) =0;
        virtual bool stop()=0;
      protected:
        FrameGraph* _graph=nullptr;
      };

      inline bool areConnected(int a_ref, int b_ref) const {
        for (const auto& f : factors(a_ref))
          if ((f->from_ref==a_ref && f->to_ref==b_ref) ||
              (f->from_ref==b_ref && f->to_ref==a_ref))
            return true;
        return false;
      }
      void visitBFS(int src_key, VisitActions& actions);

    protected:
      FrameMap _frames;
      FactorSet _factors;
      FrameFactorMap _frame_to_factor;
    

    };

  }
}
