#include "runnable.h"

namespace kd_slam {
  namespace utils {

    void Runnable::run(bool rn){
      std::unique_lock lk(_run_mtx);
      _running=rn;
      if (_running)
        _run_cv.notify_one();
    }

    bool Runnable::running() {
      std::unique_lock lk(_run_mtx);
      return _running;
    }

    void Runnable::setStepMode(bool sm){
      std::unique_lock lk(_run_mtx);
      _step_mode=sm;
      _running=!_step_mode;
      if (_running)
        _run_cv.notify_one();
    }

    bool Runnable::stepMode() {
      std::unique_lock lk(_run_mtx);
      return _step_mode;
    }

    void Runnable::waitRun(){
      std::unique_lock lk(_run_mtx);
      _run_cv.wait(lk,[&](){return _running;});
      if (_step_mode)
        _running=false;
    }
    
    Runnable::~Runnable() {}
  }
}
