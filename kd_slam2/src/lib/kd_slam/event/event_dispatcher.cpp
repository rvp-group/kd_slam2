#include "event_dispatcher.h"

namespace kd_slam {
  namespace event {

    void EventDispatcher::pushEvent(EventBasePtr e) {
        auto id=typeid(*e).hash_code();
        auto it=handlers.find(id);
        if (it==handlers.end())
          return;
        it->second->invoke(e);
      } 

  }
}
