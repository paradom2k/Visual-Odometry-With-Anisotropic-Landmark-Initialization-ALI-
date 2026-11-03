#pragma once

#include "common/types.h"

namespace jo {

/// WGS84 ellipsoid constants, from the plan document section 2.2.
struct Wgs84 {
  static constexpr double kSemiMajorAxis = 6378137.0;
  static constexpr double kFlattening = 1.0 / 298.257223563;
  static constexpr double kEccentricitySquared =
      kFlattening * (2.0 - kFlattening);
  static constexpr double kEarthRotationRate = 7.2921151467e-5;  ///< rad/s
  static constexpr double kGravityConstant = 3.986004418e14;     ///< m^3/s^2
  static constexpr double kSpeedOfLight = 299792458.0;           ///< m/s
};

/// ECEF position from geodetic coordinates.
Vec3 geodeticToEcef(const Geodetic& geodetic);

/// Geodetic coordinates from ECEF position.  Uses Bowring's method followed by
/// a few fixed-point refinements; converges to sub-millimetre on Earth.
Geodetic ecefToGeodetic(const Vec3& ecef);

/// Rotation matrix taking a vector from ENU to ECEF at the given location.
Mat3 enuToEcefRotation(const Geodetic& geodetic);

/// Rotation matrix taking a vector from ECEF to ENU at the given location.
Mat3 ecefToEnuRotation(const Geodetic& geodetic);

/// Rotation matrix taking a vector from ECEF to the local navigation frame,
/// i.e. the north-east-down frame.  Used by the inertial mechanisation, which
/// is formulated in NED even though the public outputs are ENU.
Mat3 ecefToNedRotation(const Geodetic& geodetic);

/// Rotation matrix taking a vector from the local north-east-down frame into
/// ECEF, i.e. the inverse of ecefToNedRotation.  Columns are the north, east and
/// down unit vectors, in that order.  Taking the ENU matrix and negating its
/// third column is *not* this matrix: that leaves the north and east columns the
/// wrong way round, which silently swaps the horizontal axes.
Mat3 nedToEcefRotation(const Geodetic& geodetic);

/// Radius of curvature in the prime vertical (east-west).
double primeVerticalRadius(double lat);

/// Radius of curvature in the meridian (north-south).
double meridianRadius(double lat);

/// Normal gravity magnitude at the given latitude and height, from the series
/// in the inertial mechanisation reference.  Returns m/s^2, positive downward.
double normalGravity(double lat, double height);

/// Rotation matrix from roll-pitch-yaw (ZYX) to a body-to-navigation rotation.
/// Yaw is measured from north, positive towards east.
Mat3 rpyToRotation(const Vec3& rpy);

/// Roll-pitch-yaw (ZYX) from a body-to-navigation rotation.
Vec3 rotationToRpy(const Mat3& rotation);

/// Convert an ECEF position into ENU offsets relative to a local origin.
Vec3 ecefToEnu(const Vec3& ecef, const Geodetic& origin);

/// Inverse of ecefToEnu.
Vec3 enuToEcef(const Vec3& enu, const Geodetic& origin);

/// Rotate an ECEF vector (velocity, acceleration, displacement) into ENU at the
/// given location.  This is a pure rotation: unlike ecefToEnu it must not
/// translate by the origin, which is a distinction that is easy to get wrong.
Vec3 ecefVectorToEnu(const Vec3& ecef_vector, const Geodetic& origin);

/// Rotate an ECEF vector into the north-east-down frame.
Vec3 ecefVectorToNed(const Vec3& ecef_vector, const Geodetic& origin);

/// Rotate a north-east-down vector into ECEF.
Vec3 nedVectorToEcef(const Vec3& ned, const Geodetic& origin);

/// Rotation matrix mapping body to NED from roll, pitch and yaw in radians.
/// Composition is ZYX: C_bn = Rz(yaw) * Ry(pitch) * Rx(roll), the standard
/// aeronautical convention with yaw measured from north towards east.
Mat3 bodyToNedFromRpy(const Vec3& rpy_rad);

/// Rotation matrix mapping body to ECEF from roll, pitch and yaw in radians.
Mat3 bodyToEcefFromRpy(const Vec3& rpy_rad, const Geodetic& geodetic);

}  // namespace jo
