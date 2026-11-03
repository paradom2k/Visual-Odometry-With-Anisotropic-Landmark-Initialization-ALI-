#include "common/log.h"

#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <mutex>

namespace jo {
namespace {

std::mutex g_mutex;

const char* levelName(LogLevel level) {
  switch (level) {
    case LogLevel::kTrace:
      return "TRACE";
    case LogLevel::kDebug:
      return "DEBUG";
    case LogLevel::kInfo:
      return "INFO ";
    case LogLevel::kWarn:
      return "WARN ";
    case LogLevel::kError:
      return "ERROR";
    case LogLevel::kOff:
      return "OFF  ";
  }
  return "?????";
}

std::string timestamp() {
  const std::time_t now = std::time(nullptr);
  std::tm local{};
#if defined(_WIN32)
  localtime_s(&local, &now);
#else
  localtime_r(&now, &local);
#endif
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%H:%M:%S", &local);
  return std::string(buffer);
}

}  // namespace

Log& Log::instance() {
  static Log logger;
  return logger;
}

Log::~Log() {
  if (file_ != nullptr) {
    std::fclose(static_cast<std::FILE*>(file_));
    file_ = nullptr;
  }
}

bool Log::openFile(const std::string& path) {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (file_ != nullptr) {
    std::fclose(static_cast<std::FILE*>(file_));
    file_ = nullptr;
  }
  std::FILE* handle = std::fopen(path.c_str(), "w");
  if (handle == nullptr) {
    std::fprintf(stderr, "[log] cannot open '%s' for writing\n", path.c_str());
    return false;
  }
  file_ = handle;
  return true;
}

void Log::write(LogLevel level, const std::string& message) {
  if (level < level_) {
    return;
  }
  std::lock_guard<std::mutex> lock(g_mutex);
  if (level == LogLevel::kWarn) {
    ++warnings_;
  } else if (level == LogLevel::kError) {
    ++errors_;
  }

  const std::string line =
      timestamp() + " [" + levelName(level) + "] " + message;
  if (file_ != nullptr) {
    std::fprintf(static_cast<std::FILE*>(file_), "%s\n", line.c_str());
    std::fflush(static_cast<std::FILE*>(file_));
  }
  if (echo_) {
    std::fprintf(level >= LogLevel::kWarn ? stderr : stdout, "%s\n",
                 line.c_str());
  }
}

void logTrace(const std::string& message) {
  Log::instance().write(LogLevel::kTrace, message);
}
void logDebug(const std::string& message) {
  Log::instance().write(LogLevel::kDebug, message);
}
void logInfo(const std::string& message) {
  Log::instance().write(LogLevel::kInfo, message);
}
void logWarn(const std::string& message) {
  Log::instance().write(LogLevel::kWarn, message);
}
void logError(const std::string& message) {
  Log::instance().write(LogLevel::kError, message);
}

namespace {

std::string formatVariadic(const char* format, std::va_list args) {
  char buffer[1024];
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  return std::string(buffer);
}

}  // namespace

void logInfoF(const char* format, ...) {
  std::va_list args;
  va_start(args, format);
  const std::string message = formatVariadic(format, args);
  va_end(args);
  logInfo(message);
}

void logWarnF(const char* format, ...) {
  std::va_list args;
  va_start(args, format);
  const std::string message = formatVariadic(format, args);
  va_end(args);
  logWarn(message);
}

void logErrorF(const char* format, ...) {
  std::va_list args;
  va_start(args, format);
  const std::string message = formatVariadic(format, args);
  va_end(args);
  logError(message);
}

}  // namespace jo
