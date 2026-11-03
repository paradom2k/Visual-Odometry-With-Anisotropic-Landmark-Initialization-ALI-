// GIO_GVIL modification notice (2026-10-21).
// Derived from ORB-SLAM3 (GPLv3) and modified by GIO_GVIL; see patches/orbslam3/README.md.
// route E patch 12 (first half): the reference-trajectory holder used by the GNSS anchor.
//
// Header-only on purpose: it carries no ORB-SLAM3 types, so it can be compiled and exercised before the
// optimiser is touched at all.  The second half (the soft position edge in G2oTypes.h and the addEdge loops in
// Optimizer.cc) is sketched in PATCH_12_SKETCH.md.
//
// The file format is the one the Python data layer already writes (`tools\extract_gps.py`,
// `tools\read_ref_truth.py`): `t,x,y,z` per line, `#` comments, seconds on the estimator's own clock.
#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace ORB_SLAM3 {

struct GnssSample {
  double t = 0.0;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
};

class GnssAnchor {
 public:
  /// Load `t,x,y,z` (comma separated, `#` comments).  Returns false when nothing usable was read.
  static bool loadCsv(const std::string& path, std::vector<GnssSample>* out) {
    std::ifstream input(path);
    if (!input) return false;
    std::vector<GnssSample> samples;
    std::string line;
    while (std::getline(input, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::istringstream row(line);
      std::string field;
      std::vector<double> values;
      while (std::getline(row, field, ',')) values.push_back(std::atof(field.c_str()));
      if (values.size() < 4) continue;
      GnssSample sample;
      sample.t = values[0];
      sample.p = Eigen::Vector3d(values[1], values[2], values[3]);
      samples.push_back(sample);
    }
    if (samples.size() < 2) return false;
    std::sort(samples.begin(), samples.end(),
              [](const GnssSample& a, const GnssSample& b) { return a.t < b.t; });
    *out = samples;
    return true;
  }

  static void setReference(const std::vector<GnssSample>& samples, double sigma, double max_gap) {
    reference() = samples;
    sSigma() = sigma > 0.0 ? sigma : 2.0;
    sMaxGap() = max_gap > 0.0 ? max_gap : 0.2;
  }

  static bool enabled() { return reference().size() >= 2; }
  static double sigma() { return sSigma(); }
  static double maxGap() { return sMaxGap(); }

  /// Interpolated reference position for `stamp`; false when the nearest sample is further than `maxGap`.
  static bool lookupInterpolated(double stamp, Eigen::Vector3d* out) {
    const std::vector<GnssSample>& samples = reference();
    if (samples.size() < 2) return false;
    if (stamp < samples.front().t - maxGap() || stamp > samples.back().t + maxGap()) return false;
    const auto upper = std::lower_bound(samples.begin(), samples.end(), stamp,
                                        [](const GnssSample& s, double t) { return s.t < t; });
    if (upper == samples.begin()) {
      if (std::fabs(samples.front().t - stamp) > maxGap()) return false;
      *out = samples.front().p;
      return true;
    }
    if (upper == samples.end()) {
      if (std::fabs(samples.back().t - stamp) > maxGap()) return false;
      *out = samples.back().p;
      return true;
    }
    const GnssSample& b = *upper;
    const GnssSample& a = *(upper - 1);
    if (stamp - a.t > maxGap() && b.t - stamp > maxGap()) return false;
    const double span = b.t - a.t;
    const double w = span > 0.0 ? (stamp - a.t) / span : 0.0;
    *out = a.p + w * (b.p - a.p);
    return true;
  }

 private:

public:
  // ---- route E patch 12, step 0 -------------------------------------------------------------------------
  // The reference lives in its own local frame while ORB-SLAM3's world frame is whatever the first keyframe
  // made it, so an absolute-position edge without this transform destroys the trajectory (measured round 188:
  // scale 8.303, RMS 1753 m).  The transform is a 2-D yaw plus translation with the scale held at 1, matching
  // the gauge the scorer uses; `estimateTransform` is the same Kabsch form as tools\gauge_free_score.py.
  static bool hasTransform() { return sHasTransform(); }

  static void setTransform(double yaw, double tx, double ty) {
    sYaw() = yaw; sTx() = tx; sTy() = ty; sHasTransform() = true;
  }

  /// Reference position as loaded, in the reference's own frame (used to estimate the transform itself).
  static bool lookupRaw(double stamp, Eigen::Vector3d* out) { return lookupInterpolated(stamp, out); }

  /// Estimate yaw + translation (scale fixed at 1) mapping `map_xy` onto `ref_xy`; false if degenerate.
  static bool estimateTransform(const std::vector<Eigen::Vector2d>& map_xy,
                                const std::vector<Eigen::Vector2d>& ref_xy) {
    if (map_xy.size() < 3 || map_xy.size() != ref_xy.size()) return false;
    Eigen::Vector2d mean_map = Eigen::Vector2d::Zero(), mean_ref = Eigen::Vector2d::Zero();
    for (std::size_t i = 0; i < map_xy.size(); ++i) { mean_map += map_xy[i]; mean_ref += ref_xy[i]; }
    mean_map /= static_cast<double>(map_xy.size());
    mean_ref /= static_cast<double>(ref_xy.size());
    double sxx = 0.0, sxy = 0.0, syx = 0.0, syy = 0.0;
    for (std::size_t i = 0; i < map_xy.size(); ++i) {
      const Eigen::Vector2d a = map_xy[i] - mean_map, b = ref_xy[i] - mean_ref;
      sxx += a.x() * b.x(); sxy += a.x() * b.y();
      syx += a.y() * b.x(); syy += a.y() * b.y();
    }
    const double yaw = std::atan2(sxy - syx, sxx + syy);
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double tx = mean_ref.x() - (c * mean_map.x() - s * mean_map.y());
    const double ty = mean_ref.y() - (s * mean_map.x() + c * mean_map.y());
    setTransform(yaw, tx, ty);
    return true;
  }

  /// Interpolated reference position expressed in the map frame (raw position when no transform is set).
  static bool lookup(double stamp, Eigen::Vector3d* out) {
    Eigen::Vector3d raw;
    if (!lookupInterpolated(stamp, &raw)) return false;
    if (!sHasTransform()) { *out = raw; return true; }
    const double c = std::cos(sYaw()), s = std::sin(sYaw());
    out->x() = c * raw.x() - s * raw.y() + sTx();
    out->y() = s * raw.x() + c * raw.y() + sTy();
    out->z() = raw.z();
    return true;
  }

 private:
  static bool& sHasTransform() { static bool value = false; return value; }
  static double& sYaw() { static double value = 0.0; return value; }
  static double& sTx() { static double value = 0.0; return value; }
  static double& sTy() { static double value = 0.0; return value; }
  static std::vector<GnssSample>& reference() { static std::vector<GnssSample> samples; return samples; }
  static double& sSigma() { static double value = 2.0; return value; }
  static double& sMaxGap() { static double value = 0.2; return value; }
};

}  // namespace ORB_SLAM3
