#pragma once
#include <string>
#include "ros/ros.h"

namespace std_msgs {
struct Header {
  ros::Time stamp;
  std::string frame_id;
};
struct Float32 {
  float data = 0.0f;
};
}  // namespace std_msgs
