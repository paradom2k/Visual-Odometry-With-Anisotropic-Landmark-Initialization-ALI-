#include "common/geodesy.h"

#include <cmath>

namespace jo {
namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

Vec3 geodeticToEcef(const Geodetic& g) {
  const double sin_lat = std::sin(g.lat);
  const double cos_lat = std::cos(g.lat);
  const double sin_lon = std::sin(g.lon);
  const double cos_lon = std::cos(g.lon);

  const double e2 = Wgs84::kEccentricitySquared;
  const double n = Wgs84::kSemiMajorAxis / std::sqrt(1.0 - e2 * sin_lat * sin_lat);

  Vec3 ecef;
  ecef.x() = (n + g.height) * cos_lat * cos_lon;
  ecef.y() = (n + g.height) * cos_lat * sin_lon;
  ecef.z() = (n * (1.0 - e2) + g.height) * sin_lat;
  return ecef;
}

Geodetic ecefToGeodetic(const Vec3& ecef) {
  const double a = Wgs84::kSemiMajorAxis;
  const double b = a * (1.0 - Wgs84::kFlattening);
  const double e2 = Wgs84::kEccentricitySquared;
  const double ep2 = (a * a - b * b) / (b * b);

  const double p = std::hypot(ecef.x(), ecef.y());
  const double theta = std::atan2(ecef.z() * a, p * b);

  const double sin_theta = std::sin(theta);
  const double cos_theta = std::cos(theta);

  Geodetic g;
  g.lat = std::atan2(ecef.z() + ep2 * b * sin_theta * sin_theta * sin_theta,
                     p - e2 * a * cos_theta * cos_theta * cos_theta);
  g.lon = std::atan2(ecef.y(), ecef.x());

  // Two fixed-point refinements on the height/latitude pair.  Bowring's initial
  // value is already accurate to well under a metre on Earth.
  for (int i = 0; i < 3; ++i) {
    const double sin_lat = std::sin(g.lat);
    const double n = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
    g.height = p / std::cos(g.lat) - n;
    const double lat_new =
        std::atan2(ecef.z(), p * (1.0 - e2 * n / (n + g.height)));
    if (std::abs(lat_new - g.lat) < 1e-15) {
      g.lat = lat_new;
      break;
    }
    g.lat = lat_new;
  }
  return g;
}

Mat3 enuToEcefRotation(const Geodetic& g) {
  const double sin_lat = std::sin(g.lat);
  const double cos_lat = std::cos(g.lat);
  const double sin_lon = std::sin(g.lon);
  const double cos_lon = std::cos(g.lon);

  Mat3 r;
  r(0, 0) = -sin_lon;
  r(0, 1) = -sin_lat * cos_lon;
  r(0, 2) = cos_lat * cos_lon;
  r(1, 0) = cos_lon;
  r(1, 1) = -sin_lat * sin_lon;
  r(1, 2) = cos_lat * sin_lon;
  r(2, 0) = 0.0;
  r(2, 1) = cos_lat;
  r(2, 2) = sin_lat;
  return r;
}

Mat3 ecefToEnuRotation(const Geodetic& g) {
  return enuToEcefRotation(g).transpose();
}

Mat3 ecefToNedRotation(const Geodetic& g) {
  const double sin_lat = std::sin(g.lat);
  const double cos_lat = std::cos(g.lat);
  const double sin_lon = std::sin(g.lon);
  const double cos_lon = std::cos(g.lon);

  Mat3 r;
  r(0, 0) = -sin_lat * cos_lon;
  r(0, 1) = -sin_lat * sin_lon;
  r(0, 2) = cos_lat;
  r(1, 0) = -sin_lon;
  r(1, 1) = cos_lon;
  r(1, 2) = 0.0;
  r(2, 0) = -cos_lat * cos_lon;
  r(2, 1) = -cos_lat * sin_lon;
  r(2, 2) = -sin_lat;
  return r;
}

Mat3 nedToEcefRotation(const Geodetic& g) {
  // Columns are the north, east and down unit vectors in ECEF.  Note the order:
  // an ENU matrix with a negated third column has the horizontal columns the
  // wrong way round and is not the same rotation.
  const Mat3 enu_to_ecef = enuToEcefRotation(g);
  Mat3 r;
  r.col(0) = enu_to_ecef.col(1);   // north
  r.col(1) = enu_to_ecef.col(0);   // east
  r.col(2) = -enu_to_ecef.col(2);  // down
  return r;
}

double primeVerticalRadius(double lat) {  const double sin_lat = std::sin(lat);
  const double e2 = Wgs84::kEccentricitySquared;
  return Wgs84::kSemiMajorAxis / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
}

double meridianRadius(double lat) {
  const double sin_lat = std::sin(lat);
  const double e2 = Wgs84::kEccentricitySquared;
  const double w = 1.0 - e2 * sin_lat * sin_lat;
  return Wgs84::kSemiMajorAxis * (1.0 - e2) / (w * std::sqrt(w));
}

double normalGravity(double lat, double height) {
  const double sin_lat = std::sin(lat);
  const double sin2 = sin_lat * sin_lat;
  const double sin4 = sin2 * sin2;
  const double g0 = 9.7803267714;
  return g0 * (1.0 + 5.27094e-3 * sin2 + 2.32718e-5 * sin4) - 3.086e-6 * height;
}

Mat3 rpyToRotation(const Vec3& rpy) {
  const double cr = std::cos(rpy.x());
  const double sr = std::sin(rpy.x());
  const double cp = std::cos(rpy.y());
  const double sp = std::sin(rpy.y());
  const double cy = std::cos(rpy.z());
  const double sy = std::sin(rpy.z());

  Mat3 r;
  r(0, 0) = cp * cy;
  r(0, 1) = sr * sp * cy - cr * sy;
  r(0, 2) = cr * sp * cy + sr * sy;
  r(1, 0) = cp * sy;
  r(1, 1) = sr * sp * sy + cr * cy;
  r(1, 2) = cr * sp * sy - sr * cy;
  r(2, 0) = -sp;
  r(2, 1) = sr * cp;
  r(2, 2) = cr * cp;
  return r;
}

Vec3 rotationToRpy(const Mat3& r) {
  Vec3 rpy;
  rpy.y() = std::asin(-r(2, 0));
  if (std::abs(r(2, 0)) < 1.0 - 1e-10) {
    rpy.x() = std::atan2(r(2, 1), r(2, 2));
    rpy.z() = std::atan2(r(1, 0), r(0, 0));
  } else {
    // Gimbal lock: fold the roll into yaw.
    rpy.x() = std::atan2(-r(1, 2), r(1, 1));
    rpy.z() = 0.0;
  }
  return rpy;
}

Vec3 ecefToEnu(const Vec3& ecef, const Geodetic& origin) {
  return ecefToEnuRotation(origin) * (ecef - geodeticToEcef(origin));
}

Vec3 enuToEcef(const Vec3& enu, const Geodetic& origin) {
  return geodeticToEcef(origin) + enuToEcefRotation(origin) * enu;
}

Vec3 ecefVectorToEnu(const Vec3& ecef_vector, const Geodetic& origin) {
  return ecefToEnuRotation(origin) * ecef_vector;
}

Vec3 ecefVectorToNed(const Vec3& ecef_vector, const Geodetic& origin) {
  const Vec3 enu = ecefVectorToEnu(ecef_vector, origin);
  return Vec3(enu.y(), enu.x(), -enu.z());
}

Vec3 nedVectorToEcef(const Vec3& ned, const Geodetic& origin) {
  const Vec3 enu(ned.y(), ned.x(), -ned.z());
  return enuToEcefRotation(origin) * enu;
}

Mat3 bodyToNedFromRpy(const Vec3& rpy_rad) {
  const double cr = std::cos(rpy_rad.x());
  const double sr = std::sin(rpy_rad.x());
  const double cp = std::cos(rpy_rad.y());
  const double sp = std::sin(rpy_rad.y());
  const double cy = std::cos(rpy_rad.z());
  const double sy = std::sin(rpy_rad.z());

  Mat3 rotation;
  rotation(0, 0) = cp * cy;
  rotation(0, 1) = sr * sp * cy - cr * sy;
  rotation(0, 2) = cr * sp * cy + sr * sy;
  rotation(1, 0) = cp * sy;
  rotation(1, 1) = sr * sp * sy + cr * cy;
  rotation(1, 2) = cr * sp * sy - sr * cy;
  rotation(2, 0) = -sp;
  rotation(2, 1) = sr * cp;
  rotation(2, 2) = cr * cp;
  return rotation;
}

Mat3 bodyToEcefFromRpy(const Vec3& rpy_rad, const Geodetic& geodetic) {
  return nedToEcefRotation(geodetic) * bodyToNedFromRpy(rpy_rad);
}

}  // namespace jo
