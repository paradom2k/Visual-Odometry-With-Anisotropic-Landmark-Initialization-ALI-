// Force-included into every translation unit of route_e (`/FI compat/jo_compat.h`): the handful of POSIX and
// GCC-isms the ported tree relies on, in exactly one place.
//
//   * `usleep(microseconds)` -- used ~30 times; Windows has `Sleep(milliseconds)`.
//   * `M_PI` / `M_PI_2` -- MSVC's <cmath> only defines them with `_USE_MATH_DEFINES`, and the ported tree
//     assumes they are there.
//   * `COMPILEDWITHC11` -- upstream defines it only when a GCC-style probe succeeds; without it the ported
//     code selects `std::chrono::monotonic_clock`, which the MSVC STL does not have (it is `steady_clock`).
//   * `winsock2.h` before `windows.h`, because some ported headers pull in `windows.h` first and the include
//     order then breaks the socket types.
#pragma once

#if defined(_WIN32)

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#define NOMINMAX 1
// glog (pulled in through the Ceres tree that also carries Eigen) refuses to compile once windows.h has
// defined the ERROR macro unless this is set.
#define GLOG_NO_ABBREVIATED_SEVERITIES 1
#include <winsock2.h>
#include <windows.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define COMPILEDWITHC11 1

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif
#ifndef M_PI_4
#define M_PI_4 0.78539816339744830962
#endif

/// `usleep` is deprecated/absent on Windows; the ported tree's timing calls are not on the critical path, so
/// a millisecond-granularity `Sleep` is the honest stand-in (and `Sleep(0)` for sub-millisecond requests).
inline int usleep(unsigned int microseconds) {
  if (microseconds >= 1000u) {
    Sleep(microseconds / 1000u);
  } else if (microseconds > 0u) {
    Sleep(1u);
  }
  return 0;
}

/// g2o's string_tools.cpp calls `vasprintf`, which MSVC does not have; this forwards to `_vsnprintf_s` on a
/// buffer the caller then owns (g2o frees it, so the allocation is the C runtime's).
inline int vasprintf(char** out, const char* format, va_list args) {
  va_list copy;
  va_copy(copy, args);
  const int needed = _vscprintf(format, copy);
  va_end(copy);
  if (needed < 0) {
    return -1;
  }
  char* buffer = static_cast<char*>(std::malloc(static_cast<std::size_t>(needed) + 1));
  if (buffer == nullptr) {
    return -1;
  }
  const int written = vsnprintf(buffer, static_cast<std::size_t>(needed) + 1, format, args);
  *out = buffer;
  return written;
}

#endif  // _WIN32
