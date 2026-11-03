#pragma once

#include "ali_conf.h"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace ali {

/// One camera frame of the run: when it was taken and how to get its pixels.
struct FrameRef {
  double stamp = 0.0;
  std::string path;   ///< from a directory or a manifest
  int bag_index = -1; ///< or the index of the bag message, when the source is a bag
  /// Which bag `bag_index` counts in, when `conf.bag` names a DIRECTORY of bags.
  ///
  /// These recordings are thirty-six one-minute bags, and a trajectory needs all of them; the reader takes
  /// one bag at a time, so the join happens here.  Empty means `conf.bag` itself, which is the single-file
  /// case and the one everything else in the tree uses.
  std::string bag;
  /// The bag topic this frame came from.  Stereo bags hold both cameras, so the topic cannot be inferred
  /// from the frame's position; without it `readFrame` would read the left image for both sides.
  std::string topic;
};

/// One inertial sample, in the units ORB-SLAM3's IMU::Point uses.
struct ImuSample {
  double stamp = 0.0;
  cv::Point3f gyro;   ///< rad/s
  cv::Point3f accel;  ///< m/s^2
};

/// The left camera's frames, in time order.
bool collectFrames(const Conf& conf, std::vector<FrameRef>* out, std::string* error);

/// The right camera's frames, for the stereo modes, in time order.
///
/// For a directory or manifest source the right camera is its own list and the two are paired by stamp; for
/// a bag source it is the right topic's own message list.  Nothing here assumes the pair is rectified.
bool collectFramesRight(const Conf& conf, std::vector<FrameRef>* out, std::string* error);

/// Read one frame's pixels.  Generic over the two sources, so the run loop never has to know which it is.
bool readFrame(const Conf& conf, const FrameRef& frame, cv::Mat* image, std::string* error);

/// The inertial samples, from whichever source the configuration names, in time order.
bool collectImu(const Conf& conf, std::vector<ImuSample>* out, std::string* error);

/// The largest stamp in a sample list, or 0 when empty.  Used to report the IMU's coverage, because an
/// inertial run whose IMU stops before the images do is a run that will drift and the log must say so.
double lastStamp(const std::vector<ImuSample>& samples);

}  // namespace ali
