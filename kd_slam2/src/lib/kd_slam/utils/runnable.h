#pragma once
#include <string>
#include <condition_variable>
#include <mutex>

namespace kd_slam {
  namespace utils {
    struct Runnable {
      virtual void reset(){}
      virtual void run(bool);
      virtual bool running();
      virtual void setStepMode(bool step_mode);
      virtual bool stepMode();
      void waitRun(); // to be called in the processing thread for step mode/run control
      virtual ~Runnable();
    protected:
      std::condition_variable _run_cv;
      std::mutex _run_mtx;
      bool _running=true;
      bool _step_mode=false;
    };
  }
}
