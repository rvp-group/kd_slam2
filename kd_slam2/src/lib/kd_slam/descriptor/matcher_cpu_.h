#pragma once
#include "descriptor_.h"
#include <vector>
#include "matcher_.h"

namespace kd_slam {
  namespace descriptor {

    // CPU concrete -- declarations only; method bodies in kd_descriptor_matcher_impl_.h
    template <typename MatcherBase_>
    struct MatcherCPU_ : public MatcherBase_ {
      using MatcherBase = MatcherBase_;
      using DescriptorType  = typename MatcherBase_::DescriptorType;
      using Scalar      = typename DescriptorType::Scalar;
      using Match     = typename MatcherBase_::Match;
      using QDescriptors = typename MatcherBase::QDescriptors;
      using Params       = typename MatcherBase::Params;

      using MatcherBase::params;
      
      static constexpr bool IsGPU=false;
      bool isGPU() const override {return IsGPU;}
      void addDescriptor(const DescriptorType& item, int ref) override;
      void clear() override;

      void match(std::vector<Match>& matches,
                 const QDescriptors& queries) const override;

      void match(std::vector<Match>& matches,
                 const std::vector<int>& candidates,
                 const QDescriptors& queries) const override;

    protected:
      std::vector<DescriptorType> _stored_descriptors;
    };

  }
} // namespace kd_slam::descriptor

