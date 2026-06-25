#pragma once
#include "event.h"
#include <ostream>
#include <iostream>

namespace kd_slam {
  namespace event {

    struct EventLogger : public EventSink {
      EventLogger(std::ostream& os = std::cerr) : _os(os) {}
      void pushEvent(EventBasePtr ev) override { ev->print(_os); }
      void setDone() override {}
      std::ostream& _os;
    };

  }
}
