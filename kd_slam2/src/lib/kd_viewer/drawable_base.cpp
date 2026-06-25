#include "drawable_base.h"
#include <cstdarg>
#include <algorithm>
#include <iostream>

namespace kd_slam {
  using namespace std;
  
  const char* DrawableBase::getHUDtext() const  {
    std::lock_guard<std::mutex> guard(draw_mutex);
    *read_hud_end=0;
    return read_hud_start;
  }

  void DrawableBase::writeHUD(const char* fmt, ...) {
    ptrdiff_t remaining = MAX_HUD_CHARS - (write_hud_end - write_hud_start);
    if (remaining <= 1) return;
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(write_hud_end, (size_t)remaining, fmt, args);
    va_end(args);
    if (n > 0)
      write_hud_end += std::min(n, (int)remaining - 1);
  }

  void DrawableBase::postHUD() {
    std::swap(write_hud_end, read_hud_end);
    std::swap(write_hud_start, read_hud_start);
    *write_hud_start=0;
    write_hud_end=write_hud_start;
  }
}
