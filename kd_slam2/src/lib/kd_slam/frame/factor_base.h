#pragma once
#include <memory>

namespace kd_slam {
  namespace frame {
    struct FactorBase {
      int from_ref=-1;
      int to_ref=-1;
      virtual ~FactorBase();
      bool is_enabled = true;
      bool is_valid = true;
      bool force_valid  = false;
    };
    using FactorBasePtr=std::shared_ptr<FactorBase>;
  }
}
