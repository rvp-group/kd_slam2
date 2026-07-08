#pragma once
#include "matcher_cpu_.h"


namespace kd_slam {
  namespace descriptor {

    template <typename MatcherBase_>
    void DescriptorDBCPU_<MatcherBase_>::addDescriptor(const DescriptorType& item, int ref) {
      DescriptorType d = item;
      if (ref!=(int)this->_num_descriptors)
        throw std::runtime_error("MatcherCPU_| bookkeeping error on addDescriptoor");
      d.ref = ref;
      _stored_descriptors.push_back(d);
      this->_num_descriptors = _stored_descriptors.size();
      this->_descriptors_ptr = &_stored_descriptors[0];
    }

    template <typename MatcherBase_>
    void DescriptorDBCPU_<MatcherBase_>::clear() {
      _stored_descriptors.clear();
      this->_num_descriptors = 0;
      this->_descriptors_ptr = nullptr;
    }

    template <typename MatcherBase_>
    void MatcherCPU_<MatcherBase_>::addDescriptor(const DescriptorType& item, int ref) {
      if (! _descriptor_db) {
        throw std::runtime_error ("MatcherCPU_| no desc db");
      }
      _descriptor_db->addDescriptor(item, ref);
    }

    template <typename MatcherBase_>
    void MatcherCPU_<MatcherBase_>::clear() {
      if (! _descriptor_db) {
        throw std::runtime_error ("MatcherCPU_| no desc db");
      }
      _descriptor_db->clear();
    }

    template <typename MatcherBase_>
    void MatcherCPU_<MatcherBase_>::match(std::vector<Match>& matches,
                                          const QDescriptors& queries) const {
      this->syncParams();
      if (! _descriptor_db) {
        throw std::runtime_error ("MatcherCPU_| no desc db");
      }
      auto& stored_descriptors=_descriptor_db->_stored_descriptors;
      matches.reserve(stored_descriptors.size());
      matches.clear();
      for (size_t i = 0; i < stored_descriptors.size(); ++i) {
        const auto& item = stored_descriptors[i];
        Match best_match{0,0,0,0,-1}; // invalid best match

        for(int c=0; c<DescriptorType::NumAxesCanonizations; ++c) {
          const auto& query=queries.des[c];
          Scalar d=0;
          switch (params.cmp) {
          case CompareType::MED:
            d = query.medCompare(item, params.normal_weight, params.comp_size);
            break;
          case
            CompareType::Euclidean:
            d = query.eucCompare(item, params.normal_weight, params.comp_size);
            break;
          }
          if (best_match.q_canon<0 || d < best_match.dist_match) {
            best_match = {query.ref, item.ref, i, Scalar(sqrt(d)), query.axes_canonization};
          }
        }
        if (best_match.q_canon>=0 && best_match.dist_match<params.max_distance)
          matches.push_back(best_match);
      }
    }

    template <typename MatcherBase_>
    void MatcherCPU_<MatcherBase_>::match(std::vector<Match>& matches,
                                          const std::vector<int>& candidates,
                                          const QDescriptors& queries) const {
      this->syncParams();
      if (! _descriptor_db) {
        throw std::runtime_error ("MatcherCPU_| no desc db");
      }
      auto& stored_descriptors=_descriptor_db->_stored_descriptors;
      matches.reserve(candidates.size());
      matches.clear();
      for (const auto c_idx: candidates) {
        if (c_idx<0 || c_idx>=(int)stored_descriptors.size())
          continue;
        const auto& item = stored_descriptors[c_idx];
        Match best_match{0,0,0,0,-1}; // invalid best match

        for(int c=0; c<DescriptorType::NumAxesCanonizations; ++c) {
          const auto& query=queries.des[c];
          Scalar d=0;
          switch (params.cmp) {
          case CompareType::MED:
            d = query.medCompare(item, params.normal_weight, params.comp_size);
            break;
          case
            CompareType::Euclidean:
            d = query.eucCompare(item, params.normal_weight, params.comp_size);
            break;
          }
          if (best_match.q_canon<0 || d < best_match.dist_match) {
            best_match = {query.ref, item.ref, (size_t) c_idx, Scalar(sqrt(d)), query.axes_canonization};
          }
        }
        if (best_match.q_canon>=0 && best_match.dist_match<params.max_distance)
          matches.push_back(best_match);
      }
    }

  }
} // namespace kd_slam::descriptor
