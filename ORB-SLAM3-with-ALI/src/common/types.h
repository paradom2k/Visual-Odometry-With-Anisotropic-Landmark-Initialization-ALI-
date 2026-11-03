// Core value types shared across JO_GVINS.
//
// Frame conventions are fixed here once and must not be redefined elsewhere
// (see plan document section 2.2):
//   b  body frame,  Right-Forward-Up  (RFU)
//   n  navigation,  East-North-Up    (ENU)
//   e  earth fixed, ECEF
//   c  camera,     OpenCV convention (x right, y down, z forward)
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <string>
#include <vector>

namespace jo {

using Vec3 = Eigen::Vector3d;
using Vec2 = Eigen::Vector2d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;

/// Geodetic coordinate, WGS84.  Angles in radians, height in metres.
struct Geodetic {
  double lat = 0.0;
  double lon = 0.0;
  double height = 0.0;
};

/// A sample from an inertial measurement unit, in the body frame.
/// `delta_theta` is a rotation vector (rad) and `delta_velocity` a velocity
/// increment (m/s) when the source file uses increment mode; otherwise they
/// hold angular rate (rad/s) and specific force (m/s^2).  `is_increment`
/// records which convention was decoded so downstream code never has to guess.
struct ImuSample {
  double time = 0.0;  ///< GPS seconds of week
  Vec3 gyro = Vec3::Zero();
  Vec3 accel = Vec3::Zero();
  bool is_increment = false;
  std::uint8_t flags = 0;
};

/// Which GNSS observation types a record carries.
enum class ObsType : std::uint8_t {
  kPseudorange = 1u << 0,
  kCarrierPhase = 1u << 1,
  kDoppler = 1u << 2,
  kSignalStrength = 1u << 3,
};

/// One satellite's observations for one epoch, one signal band.
struct GnssObs {
  std::string satellite;  ///< e.g. "G05", "C21", "E11"
  std::string band;       ///< e.g. "L1", "L2", "L5"
  /// RINEX observation code suffix such as "1C" or "2I", identifying the actual
  /// signal.  The frequency slot in `band` is not enough on its own: BeiDou's B1I
  /// sits in slot 1 under RINEX 3.02 and in slot 2 under 3.04, and shares slot 1
  /// with B1C, which is a different frequency.  Two receivers in the same dataset
  /// can therefore record the same physical signal in different slots, which
  /// matters as soon as observations are differenced.
  std::string observation_code;
  double pseudorange = 0.0;
  double carrier_phase = 0.0;
  double doppler = 0.0;
  double signal_strength = 0.0;
  std::uint8_t lli = 0;  ///< loss-of-lock indicator from RINEX
  bool has_pseudorange = false;
  bool has_carrier_phase = false;
  bool has_doppler = false;
};

/// All observations of one receiver at one epoch.
struct GnssEpoch {
  double time = 0.0;  ///< GPS seconds of week
  std::vector<GnssObs> observations;
};

/// A decoded camera frame.  Only metadata by default; the compressed payload is
/// kept so the caller can decide when to pay for decoding.
struct ImageFrame {
  double time = 0.0;  ///< sensor header stamp, in the bag clock domain
  std::uint32_t sequence = 0;
  std::string frame_id;
  std::string encoding;
  int width = 0;
  int height = 0;
  /// Bytes per row when `raw_raster` is set (`sensor_msgs/Image` carries its own stride, which is
  /// not always `width * channels`).
  int step = 0;
  /// `payload` is a raw raster rather than a compressed stream.  The two datasets differ here: the
  /// G-drive camera bags publish `sensor_msgs/CompressedImage`, the 4Seasons recording publishes
  /// `sensor_msgs/Image` on `/camera/image_raw`.
  bool raw_raster = false;
  std::vector<std::uint8_t> payload;  ///< JPEG bytes, or the raster when `raw_raster`
};

/// Position, velocity and attitude of the body in the earth frame.
/// `q_eb` rotates a vector from body to ECEF.
struct NavState {
  double time = 0.0;
  Vec3 p_ecef = Vec3::Zero();
  Vec3 v_ecef = Vec3::Zero();
  Quat q_eb = Quat::Identity();
  Vec3 accel_bias = Vec3::Zero();
  Vec3 gyro_bias = Vec3::Zero();
};

/// Geodetic output form, convenient for plotting and for comparison with the
/// Inertial Explorer reference files.
struct NavStateGeodetic {
  double time = 0.0;
  Geodetic position;
  Vec3 v_enu = Vec3::Zero();
  Vec3 rpy = Vec3::Zero();  ///< roll, pitch, yaw (rad), yaw about -Up
};

/// Horizontal and vertical protection levels, metres.
struct ProtectionLevel {
  double horizontal = 0.0;
  double vertical = 0.0;
  bool valid = false;
};

/// Integrity verdict produced alongside every navigation solution.
enum class IntegrityStatus : std::uint8_t {
  kNominal = 0,
  kAlert = 1,
  kUnavailable = 2,
};

/// One row of the trajectory the GUI plots and the evaluator compares.
struct TrajectoryPoint {
  double time = 0.0;
  Vec3 p_ecef = Vec3::Zero();
  /// Horizontal position in the local east-north-up frame the solution file
  /// carries, which is what the viewer plots.  It is *not* derived from p_ecef:
  /// the solution columns 1..3 are already ENU, and feeding them back through
  /// ecefToEnu (which the viewer used to do, treating them as ECEF) turns a
  /// three-hundred-metre path into a meaningless vertical line.
  Vec3 p_enu = Vec3::Zero();
  Vec3 v_enu = Vec3::Zero();
  Vec3 rpy = Vec3::Zero();
  int num_satellites = 0;
  int ambiguity_state = 0;  ///< 0 float, 1 fixed, 2 single, 3 none
  IntegrityStatus integrity = IntegrityStatus::kUnavailable;
  double protection_horizontal = 0.0;
  double protection_vertical = 0.0;
};

}  // namespace jo
