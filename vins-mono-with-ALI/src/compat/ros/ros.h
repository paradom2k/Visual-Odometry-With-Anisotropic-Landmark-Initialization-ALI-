#pragma once
// Minimal ROS compatibility layer for the ROS-free VINS-Mono port.
//
// Only the logging and assertion names used by the estimator core are provided.  The node wrappers,
// message types and publishers are deliberately absent: this build reads images/IMU from files and writes
// a TUM trajectory, so anything that would need a ROS transport must fail to compile rather than silently
// link against a stub.

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

namespace ros {
struct Time {
  double t = 0.0;
  double toSec() const { return t; }
};
}  // namespace ros

#define VINS_LOG(level, ...)                                     \
  do {                                                           \
    std::fprintf(stderr, "[" level "] ");                        \
    std::fprintf(stderr, __VA_ARGS__);                           \
    std::fprintf(stderr, "\n");                                  \
  } while (0)

#define ROS_DEBUG(...) do {} while (0)
#define ROS_INFO(...) VINS_LOG("info", __VA_ARGS__)
#define ROS_WARN(...) VINS_LOG("warn", __VA_ARGS__)
#define ROS_ERROR(...) VINS_LOG("error", __VA_ARGS__)
#define ROS_FATAL(...) VINS_LOG("fatal", __VA_ARGS__)
#define ROS_DEBUG_STREAM(x) do {} while (0)
#define ROS_INFO_STREAM(x) do { std::cerr << "[info] " << x << std::endl; } while (0)
#define ROS_WARN_STREAM(x) do { std::cerr << "[warn] " << x << std::endl; } while (0)
#define ROS_ERROR_STREAM(x) do { std::cerr << "[error] " << x << std::endl; } while (0)
#define ROS_BREAK() std::abort()
#define ROS_ASSERT(x) assert(x)
#define ROS_ASSERT_MSG(x, ...) assert(x)
