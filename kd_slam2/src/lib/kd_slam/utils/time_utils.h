#pragma once
#include <chrono>

namespace kd_slam::utils {

  inline auto getNow() {
    return std::chrono::steady_clock::now();
  }

  template <typename T>
  inline double getDurationMs(T t0) {
    return std::chrono::duration<double, std::milli>(getNow() - t0).count();
  }

} // namespace kd_slam::utils
