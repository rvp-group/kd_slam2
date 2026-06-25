#pragma once
#include <memory>
#include <ostream>
#include <list>

namespace kd_slam {
  namespace event {
    struct EventBase {
      virtual ~EventBase() = default;
      virtual void print(std::ostream& os) const = 0;
      friend std::ostream& operator<<(std::ostream& os, const EventBase& e) {
        e.print(os); return os;
      }
    };
    using EventBasePtr = std::shared_ptr<EventBase>;

    struct EventSink {
      virtual void pushEvent(EventBasePtr ev) = 0;
      virtual void setDone() = 0;
    };
    using EventSinkPtr=std::shared_ptr<EventSink>;
    
    struct EventSource {
      virtual EventBasePtr popEvent() = 0;
      virtual bool isDone() const = 0;
    };

    struct EventPublisher: public EventSink {
      void pushEvent(EventBasePtr ev)  override {
        if (! ev) return;
        for (auto s: event_sinks) if (s) s->pushEvent(ev);
      }
      void setDone()  override {
        for (auto s: event_sinks) if (s) s->setDone();
      }
      std::list <EventSinkPtr> event_sinks;
    };
  }
}
