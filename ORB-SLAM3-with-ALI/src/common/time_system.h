#pragma once

#include <cstdint>
#include <string>

namespace jo {

/// GPS time expressed as a week number plus seconds of week.  This is the
/// internal time system (plan document section 2.2).  `sow` may exceed 604800
/// or be negative, in which case `normalized()` folds it back.
struct GpsTime {
  int week = 0;
  double sow = 0.0;

  /// The usual total ordering, useful for direct comparison.
  double toSeconds() const { return static_cast<double>(week) * 604800.0 + sow; }

  static GpsTime fromSeconds(double seconds) {
    GpsTime t;
    t.week = static_cast<int>(seconds / 604800.0);
    t.sow = seconds - static_cast<double>(t.week) * 604800.0;
    return t;
  }

  GpsTime normalized() const { return fromSeconds(toSeconds()); }
};

/// Difference in seconds, `lhs - rhs`.
double secondsBetween(const GpsTime& lhs, const GpsTime& rhs);

/// Civil calendar fields to GPS time.  The caller supplies fields already in the
/// GPS calendar, which is what RINEX 3 files record; no leap-second term is
/// applied.  `second` may be fractional.  Returns false for an implausible date,
/// so a version number that happens to look like a date cannot slip through.
bool civilToGpsTime(int year, int month, int day, int hour, int minute,
                    double second, GpsTime* out);

/// The same conversion, as seconds of week.  This is the project-wide GNSS time
/// representation: absolute GPS seconds would sit 1.4e9 s away from the inertial
/// epochs and silently break every cross-sensor comparison.
double civilToSecondsOfWeek(int year, int month, int day, int hour, int minute,
                            double second);

/// GPS to UTC.  The offset is 18 s for the whole span of this dataset; it is
/// kept as a constant with a named accessor so that it is easy to audit.
double gpsToUtcOffsetSeconds();

/// Julian date from GPS time, used when reading ephemeris and SP3 records.
double julianDate(const GpsTime& time);

/// Abbreviated representation such as "2323:374841.000".
std::string formatGpsTime(const GpsTime& time);

/// Human-readable local time (UTC+8) such as "2024-07-18 08:07:11.300".
/// Convenience only: the dataset names its files in this time base.
std::string formatLocalTime(const GpsTime& time);

/// Parse a `YYYY-MM-DD-HH-MM-SS` prefix as used by the dataset file names and
/// return it as GPS time.  Returns false when the prefix does not match.
bool parseDatasetFileStamp(const std::string& name, GpsTime* out);

}  // namespace jo
