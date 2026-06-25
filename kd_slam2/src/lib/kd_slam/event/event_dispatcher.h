#pragma once
#include "event.h"
#include <unordered_map>
#include <functional>
#include <typeinfo>
#include <iostream>

namespace kd_slam {
  namespace event {

    struct EventDispatcher: public EventSink {
      struct HandlerBase {
        virtual void invoke(EventBasePtr) = 0;
      };
      using HandlerMap = std::unordered_map<size_t, std::unique_ptr<HandlerBase>>;

      template <typename T>
      struct Handler_ : public HandlerBase {
        Handler_(std::function<void(std::shared_ptr<T>)> fn_): fn(fn_) {}
        void invoke(EventBasePtr p) override {
          fn(std::static_pointer_cast<T>(p));
        }
        std::function<void(std::shared_ptr<T>)> fn;
      };

      template <typename T>
      void registerCallback(std::function<void(std::shared_ptr<T>)> fn_) {
        auto id = typeid(T).hash_code();
        handlers[id] = std::make_unique<Handler_<T>>(fn_);
        //std::cerr << "EventDispatcher| registerCallback, hash:" << id << std::endl;
      }

      void pushEvent(EventBasePtr e) override;
      void setDone() override {}

      HandlerMap handlers;
    };
  }
}
