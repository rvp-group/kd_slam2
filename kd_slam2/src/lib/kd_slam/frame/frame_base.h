#pragma once
#include <memory>

namespace kd_slam {
  namespace frame {
    struct FrameBase {
      double ts = 0;
      inline bool operator<(const FrameBase& other) const { return this->ts<other.ts; }
      friend struct FrameGraph;
      int ref() const {return _ref;}
      virtual ~FrameBase();
    protected:
      int _ref=-1; // detached state;
    };

    using FrameBasePtr=std::shared_ptr<FrameBase>;

  }
}
