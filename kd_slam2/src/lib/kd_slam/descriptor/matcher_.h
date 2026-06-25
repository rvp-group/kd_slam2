#pragma once
#include "descriptor_.h"
#include "srrg_config/configurable.h"
#include <limits>
#include <vector>

namespace kd_slam {
  namespace descriptor {
    enum CompareType { Euclidean = 0x0, MED = 0x1 };

    template <typename Scalar_>
    struct MatcherParams_ {
      using Scalar=Scalar_;
      Scalar normal_weight=0;
      Scalar max_distance=std::numeric_limits<Scalar>::max();
      CompareType cmp=MED;
      int comp_size=-1;
    };

    // Abstract base
    template <typename Descriptor_>
    struct Matcher_ : public srrg2_core::Configurable {
      using DescriptorType = Descriptor_;
      using Scalar         = typename DescriptorType::Scalar;
      using Params         = MatcherParams_<Scalar>;

      PARAM(srrg2_core::PropertyFloat, max_distance,  "max descriptor distance",           std::numeric_limits<float>::max(), &_params_changed);
      PARAM(srrg2_core::PropertyFloat, normal_weight, "normal direction weight",            0.f,                              &_params_changed);
      PARAM(srrg2_core::PropertyInt,   cmp,           "compare type (0=Euclidean, 1=MED)",  1,                                &_params_changed);
      PARAM(srrg2_core::PropertyInt,   comp_size,     "descriptor comparison size (-1=all)", -1,                              &_params_changed);

      void syncParams() const {
        if (!_params_changed)
          return;
        params.max_distance  = param_max_distance.value();
        params.normal_weight = param_normal_weight.value();
        params.cmp           = static_cast<CompareType>(param_cmp.value());
        params.comp_size     = param_comp_size.value();
        _params_changed = false;
      }

      struct Match {
        size_t q_ref;
        size_t ref_match;
        size_t idx_match;
        Scalar dist_match;
        int    q_canon;
      };

      mutable Params params;
      size_t      _num_descriptors = 0;
      DescriptorType* _descriptors_ptr = nullptr;

      size_t getNumDescriptors() const { return _num_descriptors; }
      virtual void addDescriptor(const DescriptorType& item, int ref) = 0;
      virtual void clear() = 0;

      using QDescriptors = typename DescriptorType::QDescriptors;

      // match for all possible canonizations of a descriptor
      // only the "best" canonization is retained
      virtual void match(std::vector<Match>& matches,
                         const QDescriptors& queries) const = 0;

      // match for all possible canonizations of a descriptor
      // only the "best" canonization is retained
      virtual void match(std::vector<Match>& matches,
                         const std::vector<int>& candidates,
                         const QDescriptors& queries) const = 0;

      virtual bool isGPU() const = 0;

    protected:
      mutable bool _params_changed = true;
    };

  }
} // namespace kd_slam::descriptor
