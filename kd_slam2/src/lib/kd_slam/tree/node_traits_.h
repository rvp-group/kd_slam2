#pragma once
// No includes needed: NodeTraits_ is a pure template with no dependencies.

namespace kd_slam {

  // NodeTraits_ provides per-node-type compile-time constants.
  // Specialise for a concrete node type to override defaults.
  template <typename Node_>
  struct NodeTraits_ {
    // Tree depth used to build descriptors (number of levels above leaves)
    static constexpr int ExtractorLevel = 5;
    // Descriptor size derived from extractor level: (2^Level) - 2
    static constexpr int DescriptorSize = (1 << ExtractorLevel) - 2;
  };

} // namespace kd_slam
