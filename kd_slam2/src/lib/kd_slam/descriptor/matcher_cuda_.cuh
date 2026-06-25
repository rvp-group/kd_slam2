#pragma once
#include "matcher_.h"
#include "cuda/prefix_sum_.h"


namespace kd_slam {
  namespace descriptor {

    // CUDA concrete matcher -- declarations only.
    // Method bodies + kernel are in kd_descriptors_cuda.cu.
    template <typename MatcherBase_>
    struct MatcherCUDA_ : public MatcherBase_ {
      using MatcherBase      = MatcherBase_;
      using DescriptorType       = typename MatcherBase_::DescriptorType;
      using Scalar           = typename DescriptorType::Scalar;
      using Match          = typename MatcherBase_::Match;
      using QDescriptors     = typename MatcherBase::QDescriptors;
      using Params       = typename MatcherBase::Params;

      using MatcherBase::params;

      explicit MatcherCUDA_(size_t capacity = 10240);
      ~MatcherCUDA_();

      static constexpr bool IsGPU=true;
      bool isGPU() const override {return IsGPU;}

      void addDescriptor(const DescriptorType& item, int ref) override;
      void clear() override;

      void match(std::vector<Match>& matches,
                 const QDescriptors& queries) const override;

      void match(std::vector<Match>& matches,
                 const std::vector<int>& candidates,
                 const QDescriptors& queries) const override;


    protected:
      size_t               _capacity;
      DescriptorType*          _stored_desc_ptr  = nullptr;
      Match*             _matches_ptr      = nullptr;
      Match*             _matches_src_ptr  = nullptr;
      Match*             _matches_dest_ptr = nullptr;
      int*                 _marks_ptr        = nullptr;
      mutable int          _num_matches      = 0;
      mutable cuda::PrefixSum_<int> _prefix_adder;
    };

  }
} // namespace kd_slam::descriptor
