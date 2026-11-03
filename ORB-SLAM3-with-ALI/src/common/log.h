#pragma once

#include <string>
#include <vector>

namespace jo {

enum class LogLevel : int {
  kTrace = 0,
  kDebug = 1,
  kInfo = 2,
  kWarn = 3,
  kError = 4,
  kOff = 5,
};

/// Process-wide logger.  Deliberately tiny: the plan document calls for levelled
/// logging with no silent configuration loss, which a hand-rolled logger makes
/// easy to audit.
class Log {
 public:
  static Log& instance();

  void setLevel(LogLevel level) { level_ = level; }
  LogLevel level() const { return level_; }

  /// Mirror everything to stderr as well as the log file.
  void setEchoToStderr(bool enabled) { echo_ = enabled; }

  /// Open a log file.  Returns false if the path is not writable.  A failure is
  /// reported on stderr but does not abort the run.
  bool openFile(const std::string& path);

  void write(LogLevel level, const std::string& message);

  /// Number of warnings and errors seen so far, for the final summary.
  int warningCount() const { return warnings_; }
  int errorCount() const { return errors_; }

 private:
  Log() = default;
  ~Log();
  Log(const Log&) = delete;
  Log& operator=(const Log&) = delete;

  LogLevel level_ = LogLevel::kInfo;
  bool echo_ = true;
  void* file_ = nullptr;  // std::FILE*
  int warnings_ = 0;
  int errors_ = 0;
};

void logTrace(const std::string& message);
void logDebug(const std::string& message);
void logInfo(const std::string& message);
void logWarn(const std::string& message);
void logError(const std::string& message);

/// printf-style helpers; the format string is applied with a 1024-byte buffer.
void logInfoF(const char* format, ...);
void logWarnF(const char* format, ...);
void logErrorF(const char* format, ...);

}  // namespace jo
