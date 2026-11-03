#include "common/time_system.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace jo {
namespace {

constexpr double kSecondsPerWeek = 604800.0;
constexpr double kGpsToUtcLeapSeconds = 18.0;
constexpr double kUnixEpochJulianDate = 2440587.5;
/// GPS time started 1980-01-06, which is Julian date 2444244.5.  UTC and GPS
/// were aligned at that instant, so no leap-second term appears here.
constexpr double kGpsEpochJulianDate = 2444244.5;

const char* kMonthName(int month) {
  static const char* kNames[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                 "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  if (month < 1 || month > 12) {
    return "???";
  }
  return kNames[month - 1];
}

}  // namespace

double secondsBetween(const GpsTime& lhs, const GpsTime& rhs) {
  return lhs.toSeconds() - rhs.toSeconds();
}

double gpsToUtcOffsetSeconds() { return kGpsToUtcLeapSeconds; }

bool civilToGpsTime(int year, int month, int day, int hour, int minute,
                    double second, GpsTime* out) {
  if (out == nullptr) {
    return false;
  }
  if (year < 1980 || year > 2200 || month < 1 || month > 12 || day < 1 ||
      day > 31 || hour < 0 || hour > 23 || minute < 0 || minute > 59 ||
      second < 0.0 || second > 61.0) {
    return false;
  }

  // Days since 1970-01-01, Howard Hinnant's days_from_civil.
  const long long y = (month <= 2) ? (year - 1) : year;
  const long long era = (y >= 0 ? y : y - 399) / 400;
  const unsigned long long yoe = static_cast<unsigned long long>(y - era * 400);
  const unsigned long long mp = static_cast<unsigned long long>((month + 9) % 12);
  const unsigned long long doy =
      (153 * mp + 2) / 5 + static_cast<unsigned long long>(day) - 1;
  const unsigned long long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const long long days = era * 146097 + static_cast<long long>(doe) - 719468;

  // 1980-01-06 is 3657 days after the Unix epoch.  That day is a Sunday, so the
  // remainder is already the correct day-of-week offset.
  const long long gps_days = days - 3657;
  const double seconds = static_cast<double>(gps_days) * 86400.0 +
                         hour * 3600.0 + minute * 60.0 + second;
  *out = GpsTime::fromSeconds(seconds);
  return true;
}

double civilToSecondsOfWeek(int year, int month, int day, int hour, int minute,
                            double second) {
  GpsTime time;
  if (!civilToGpsTime(year, month, day, hour, minute, second, &time)) {
    return 0.0;
  }
  return time.sow;
}

double julianDate(const GpsTime& time) {
  return kGpsEpochJulianDate + time.toSeconds() / 86400.0;
}

std::string formatGpsTime(const GpsTime& time) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%d:%012.3f", time.week, time.sow);
  return std::string(buffer);
}

std::string formatLocalTime(const GpsTime& time) {
  // Convert to a Unix timestamp, then shift into UTC+8.
  const double gps_seconds = time.toSeconds();
  const double unix_seconds =
      (kGpsEpochJulianDate - kUnixEpochJulianDate) * 86400.0 + gps_seconds -
      kGpsToUtcLeapSeconds;
  const double local_seconds = unix_seconds + 8.0 * 3600.0;

  // Civil date from days since 1970-01-01, via Howard Hinnant's algorithm.
  long long days = static_cast<long long>(std::floor(local_seconds / 86400.0));
  double rem = local_seconds - static_cast<double>(days) * 86400.0;
  if (rem < 0.0) {
    rem += 86400.0;
    --days;
  }

  long long z = days + 719468;
  const long long era = (z >= 0 ? z : z - 146096) / 146097;
  const unsigned long long doe = static_cast<unsigned long long>(z - era * 146097);
  const unsigned long long yoe =
      (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const long long y = static_cast<long long>(yoe) + era * 400;
  const unsigned long long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned long long mp = (5 * doy + 2) / 153;
  const unsigned long long d = doy - (153 * mp + 2) / 5 + 1;
  const unsigned long long m = mp < 10 ? mp + 3 : mp - 9;
  const long long year = y + (m <= 2 ? 1 : 0);

  const int hour = static_cast<int>(rem / 3600.0);
  rem -= hour * 3600.0;
  const int minute = static_cast<int>(rem / 60.0);
  rem -= minute * 60.0;

  char buffer[80];
  std::snprintf(buffer, sizeof(buffer), "%04lld-%02llu-%02llu %02d:%02d:%06.3f",
                year, m, d, hour, minute, rem);
  return std::string(buffer);
}

bool parseDatasetFileStamp(const std::string& name, GpsTime* out) {
  if (out == nullptr) {
    return false;
  }
  // Look for the first `YYYY-MM-DD-HH-MM-SS` inside the string; dataset files
  // are named "<stamp>_<index>_<camera>.<ext>".
  for (std::size_t i = 0; i + 19 <= name.size(); ++i) {
    int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;
    if (std::sscanf(name.c_str() + i, "%4d-%2d-%2d-%2d-%2d-%2d", &year, &month,
                    &day, &hour, &minute, &second) != 6) {
      continue;
    }
    if (year < 2000 || year > 2100 || month < 1 || month > 12 || day < 1 ||
        day > 31 || hour < 0 || hour > 23 || minute < 0 || minute > 59 ||
        second < 0 || second > 60) {
      continue;
    }
    if (name[i + 4] != '-' || name[i + 7] != '-' || name[i + 10] != '-' ||
        name[i + 13] != '-' || name[i + 16] != '-') {
      continue;
    }

    // Civil date to days since 1970-01-01.
    const long long y = (month <= 2) ? (year - 1) : year;
    const long long era = (y >= 0 ? y : y - 399) / 400;
    const unsigned long long yoe = static_cast<unsigned long long>(y - era * 400);
    const unsigned long long mp =
        (month + 9) % 12;  // March = 0
    const unsigned long long doy = (153 * mp + 2) / 5 + day - 1;
    const unsigned long long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    const long long days = era * 146097 + static_cast<long long>(doe) - 719468;

    const double local_seconds =
        static_cast<double>(days) * 86400.0 + hour * 3600.0 + minute * 60.0 +
        second;
    const double unix_seconds = local_seconds - 8.0 * 3600.0;
    const double gps_seconds =
        unix_seconds - (kGpsEpochJulianDate - kUnixEpochJulianDate) * 86400.0 +
        kGpsToUtcLeapSeconds;

    GpsTime t = GpsTime::fromSeconds(gps_seconds);
    // Reject absurd results (for example a version number that happens to look
    // like a date).
    if (t.week < 2000 || t.week > 2600) {
      continue;
    }
    *out = t;
    return true;
  }
  return false;
}

}  // namespace jo
