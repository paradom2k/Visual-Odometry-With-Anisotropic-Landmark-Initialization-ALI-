#include "ali_run.h"

#include "ali_files.h"
#include "ali_sensors.h"

#include "common/log.h"

#include "System.h"
#include "LocalMapping.h"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <exception>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <windows.h>
#include <dbghelp.h>
#pragma comment(lib, "dbghelp.lib")
#endif

namespace ali {

namespace {

#if defined(_WIN32)
// A one-off crash in the windowed UI used to leave nothing behind: the handler wrote only to stderr, and
// closing the window destroyed that.  These handlers now write a durable report, a minidump (all threads,
// not just the faulting one) and a copy of the link map from the same build, so the addresses remain
// resolvable even after a later rebuild.
std::string g_active_conf_path;
volatile LONG g_faulting = 0;

/// The repository root, derived from the executable's location.  Fixed buffers only: the heap may already
/// be damaged when this runs.
bool crashRootPath(char* out, std::size_t size) {
  char exe[MAX_PATH] = {0};
  if (GetModuleFileNameA(nullptr, exe, static_cast<DWORD>(MAX_PATH)) == 0) {
    return false;
  }
  char* slash = std::strrchr(exe, '\\');
  if (slash == nullptr) {
    slash = std::strrchr(exe, '/');
  }
  if (slash == nullptr) {
    return false;
  }
  *slash = '\0';  // the executable's directory
  char* leaf = std::strrchr(exe, '\\');
  if (leaf == nullptr) {
    leaf = std::strrchr(exe, '/');
  }
  if (leaf != nullptr) {
    const char* name = leaf + 1;
    if (std::strcmp(name, "bin") == 0 || std::strcmp(name, "build") == 0) {
      *leaf = '\0';
    }
  }
  if (std::strlen(exe) + 1 > size) {
    return false;
  }
  std::snprintf(out, size, "%s", exe);
  return true;
}

/// The last path separator, whichever kind the string mixes.
char* lastSeparator(char* path) {
  char* back = std::strrchr(path, '\\');
  char* forward = std::strrchr(path, '/');
  if (back == nullptr) {
    return forward;
  }
  if (forward == nullptr) {
    return back;
  }
  return back > forward ? back : forward;
}

/// Write one fault report to <repo>/reports/ali_crash.log (or %TEMP% if that cannot be opened), copy the
/// matching link map and executable beside it, and write a minidump.  Called from SEH, std::terminate and
/// abort.
void writeFaultReport(const char* kind, EXCEPTION_POINTERS* info, unsigned long code, const void* where,
                      bool want_dump) {
  if (InterlockedCompareExchange(&g_faulting, 1, 0) != 0) {
    std::_Exit(3);
  }
  HMODULE module = GetModuleHandleW(nullptr);
  void* frames[64];
  const USHORT count = CaptureStackBackTrace(0, 64, frames, nullptr);

  char root[MAX_PATH] = {0};
  const bool have_root = crashRootPath(root, sizeof(root));
  char log_path[MAX_PATH + 64] = {0};
  std::FILE* log = nullptr;
  if (have_root) {
    std::snprintf(log_path, sizeof(log_path), "%s/reports/ali_crash.log", root);
    log = std::fopen(log_path, "a");
  }
  if (log == nullptr) {
    char temp[MAX_PATH] = {0};
    if (GetTempPathA(static_cast<DWORD>(MAX_PATH), temp) == 0) {
      return;
    }
    std::snprintf(log_path, sizeof(log_path), "%sali_crash.log", temp);
    log = std::fopen(log_path, "a");
    if (log == nullptr) {
      return;
    }
  }

  SYSTEMTIME now;
  GetLocalTime(&now);
  auto emit = [&](const char* line) {
    std::fprintf(stderr, "%s", line);
    std::fputs(line, log);
  };
  char line[1024];
  std::snprintf(line, sizeof(line),
                "\n[FAULT] %04u-%02u-%02u %02u:%02u:%02u  %s  code %08lX at %p, module base %p, "
                "%u frames\n",
                now.wYear, now.wMonth, now.wDay, now.wHour, now.wMinute, now.wSecond, kind, code, where,
                static_cast<void*>(module), static_cast<unsigned>(count));
  emit(line);
  std::snprintf(line, sizeof(line), "[FAULT] config: %s\n",
                g_active_conf_path.empty() ? "(not set)" : g_active_conf_path.c_str());
  emit(line);
  std::snprintf(line, sizeof(line), "[FAULT] command line: %s\n", GetCommandLineA());
  emit(line);
  for (USHORT i = 0; i < count; ++i) {
    const std::uintptr_t rva = reinterpret_cast<std::uintptr_t>(frames[i]) -
                               reinterpret_cast<std::uintptr_t>(module);
    std::snprintf(line, sizeof(line), "[FAULT] frame %2u  rva 0x%llX\n", static_cast<unsigned>(i),
                  static_cast<unsigned long long>(rva));
    emit(line);
  }

  // Preserve the map and executable from this build next to the report.  A rebuild overwrites
  // build/<name>.map, which would make the RVAs above unresolvable; the copies keep the crash
  // self-contained, and resolve_map.py looks for the .exe beside the .map.
  char map_dst_for_log[MAX_PATH + 128] = {0};
  char exe[MAX_PATH] = {0};
  if (have_root && GetModuleFileNameA(nullptr, exe, static_cast<DWORD>(MAX_PATH)) != 0) {
    char* base = std::strrchr(exe, '\\');
    base = base == nullptr ? exe : base + 1;
    char stem[MAX_PATH] = {0};
    std::snprintf(stem, sizeof(stem), "%s", base);
    char* dot = std::strrchr(stem, '.');
    if (dot != nullptr) {
      *dot = '\0';
    }
    char map_src[MAX_PATH + 64] = {0};
    std::snprintf(map_src, sizeof(map_src), "%s/build/%s.map", root, stem);
    char* log_slash = lastSeparator(log_path);
    if (log_slash != nullptr) {
      const char separator = *log_slash;
      *log_slash = '\0';
      char exe_dst[MAX_PATH + 128] = {0};
      std::snprintf(map_dst_for_log, sizeof(map_dst_for_log), "%s/ali_crash_%lu.map", log_path,
                    static_cast<unsigned long>(GetCurrentProcessId()));
      std::snprintf(exe_dst, sizeof(exe_dst), "%s/ali_crash_%lu.exe", log_path,
                    static_cast<unsigned long>(GetCurrentProcessId()));
      if (CopyFileA(map_src, map_dst_for_log, FALSE)) {
        std::snprintf(line, sizeof(line), "[FAULT] map: %s\n", map_dst_for_log);
        emit(line);
      } else {
        std::snprintf(line, sizeof(line), "[FAULT] map copy failed: %s\n", map_src);
        emit(line);
        map_dst_for_log[0] = '\0';
      }
      if (CopyFileA(exe, exe_dst, FALSE)) {
        std::snprintf(line, sizeof(line), "[FAULT] exe: %s\n", exe_dst);
        emit(line);
      }
      *log_slash = separator;
    }
  }

  if (want_dump && log_path[0] != 0) {
    char dump_path[MAX_PATH + 128];
    std::snprintf(dump_path, sizeof(dump_path), "%s", log_path);
    char* slash = lastSeparator(dump_path);
    if (slash != nullptr) {
      *slash = '\0';
      std::snprintf(dump_path + std::strlen(dump_path), sizeof(dump_path) - std::strlen(dump_path),
                    "/ali_crash_%lu.dmp", static_cast<unsigned long>(GetCurrentProcessId()));
      HANDLE dump = CreateFileA(dump_path, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS,
                                FILE_ATTRIBUTE_NORMAL, nullptr);
      if (dump != INVALID_HANDLE_VALUE) {
        MINIDUMP_EXCEPTION_INFORMATION exception_info;
        exception_info.ThreadId = GetCurrentThreadId();
        exception_info.ExceptionPointers = info;
        exception_info.ClientPointers = FALSE;
        MiniDumpWriteDump(GetCurrentProcess(), GetCurrentProcessId(), dump, MiniDumpNormal,
                          info != nullptr ? &exception_info : nullptr, nullptr, nullptr);
        CloseHandle(dump);
        std::snprintf(line, sizeof(line), "[FAULT] minidump: %s\n", dump_path);
        emit(line);
      }
    }
  }
  if (map_dst_for_log[0] != 0) {
    std::snprintf(line, sizeof(line),
                  "[FAULT] resolve with: python tools/resolve_map.py %s %s\n", log_path,
                  map_dst_for_log);
    emit(line);
  } else {
    emit("[FAULT] map not copied; resolve against build/<name>.map from the same build\n");
  }
  std::fflush(stderr);
  std::fflush(log);
  std::fclose(log);
}

LONG WINAPI crashHandler(EXCEPTION_POINTERS* info) {
  const unsigned long code = info != nullptr && info->ExceptionRecord != nullptr
                                 ? info->ExceptionRecord->ExceptionCode
                                 : 0;
  const void* const where = info != nullptr && info->ExceptionRecord != nullptr
                                ? info->ExceptionRecord->ExceptionAddress
                                : nullptr;
  const char* kind = code == 0xE06D7363ul ? "uncaught C++ exception" : "SEH exception";
  writeFaultReport(kind, info, code, where, true);
  return EXCEPTION_EXECUTE_HANDLER;
}

void terminateHandler() {
  writeFaultReport("std::terminate", nullptr, 0xE0000000ul, nullptr, true);
  std::_Exit(3);
}

void abortHandler(int) {
  writeFaultReport("SIGABRT", nullptr, 0xE0000000ul, nullptr, true);
  std::_Exit(3);
}

void installCrashHandler() {
  SetUnhandledExceptionFilter(&crashHandler);
  std::set_terminate(&terminateHandler);
  std::signal(SIGABRT, &abortHandler);
}
#else
void installCrashHandler() {}
#endif

/// Decoded frames, produced ahead of the tracker by one thread.
///
/// Reading is 15.8 ms of a 44.0 ms frame on the full 4Seasons drive (bag decompression plus JPEG decode, and
/// the per-bag cache is lost every time the drive crosses into the next bag) and 25.2 ms of 51.4 ms in the
/// stereo-inertial case, so it is the one stage that can be overlapped with the estimation without changing
/// anything that is estimated.  The producer calls the SAME `readFrame` and the SAME right-frame pairing as
/// the loop it feeds, and it hands over the failures the same way, so the consumer sees the frames it would
/// have seen serially -- same order, same errors.  What changes is only WHEN the decode happens.
struct FramePair {
  std::size_t index = 0;   ///< the left-frame index this belongs to
  cv::Mat left;
  cv::Mat right;
  std::string left_error;  ///< empty when `left` was read
  std::string right_error; ///< empty when `right` was read or not asked for
};

class FramePrefetcher {
 public:
  FramePrefetcher(const Conf& conf, const std::vector<FrameRef>& left,
                  const std::vector<FrameRef>& right, bool stereo, std::size_t first_used,
                  double end_stamp, bool threaded)
      : conf_(conf), left_(left), right_(right), stereo_(stereo), first_used_(first_used),
        end_stamp_(end_stamp), threaded_(threaded), next_index_(first_used) {
    if (threaded_) {
      worker_ = std::thread([this]() { run(); });
    }
  }

  ~FramePrefetcher() { stop(); }

  /// Ends the producer and joins it.  Called before the run's result is filled in, so `decodeSeconds` reads a
  /// value no other thread is writing.
  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    not_full_.notify_all();
    not_empty_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  /// Total time the producer spent decoding.  This is the cost that used to be serial with the tracker; the
  /// difference between it and the run's wall time is what the overlap bought.
  double decodeSeconds() const { return decode_seconds_; }

  /// Total time the tracker spent waiting for the prefetch producer.  Zero on the serial path, whose
  /// decode cost is counted by decodeSeconds().
  double waitSeconds() const { return wait_seconds_; }

  /// Blocks until the next frame is ready.  False when the sequence is exhausted.
  bool next(FramePair* out) {
    if (!threaded_) {
      // The serial path: decode here, in the order the loop asks.  This is the behaviour the estimator had
      // before the prefetcher existed, kept because the trajectory depends on when the decode happens
      // relative to the mapping threads -- see the setting `prefetch_frames`.
      if (next_index_ >= left_.size() || left_[next_index_].stamp > end_stamp_) {
        return false;
      }
      // The decode is this thread's own work and is counted by decodeSeconds(); it is not time spent
      // waiting for a producer, so it must not be added to the wait total (that double-counted it in the
      // UI timing).
      decodeInto(next_index_, out);
      ++next_index_;
      return true;
    }
    const auto wait_started = std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [this]() { return !queue_.empty() || done_; });
    if (queue_.empty()) {
      return false;
    }
    *out = std::move(queue_.front());
    queue_.pop_front();
    lock.unlock();
    not_full_.notify_one();
    wait_seconds_ +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_started).count();
    return true;
  }

 private:
  /// The right frame paired with left index `i`, by the same rule the serial loop used: the same index when
  /// the counts agree, otherwise the nearest stamp.
  std::size_t rightFor(std::size_t i) const {
    if (right_.empty()) {
      return 0;
    }
    std::size_t j = std::min(i, right_.size() - 1);
    if (right_.size() != left_.size()) {
      std::size_t best = 0;
      double best_gap = 1.0e18;
      for (std::size_t k = 0; k < right_.size(); ++k) {
        const double gap = std::fabs(right_[k].stamp - left_[i].stamp);
        if (gap < best_gap) {
          best_gap = gap;
          best = k;
        }
      }
      j = best;
    }
    return j;
  }

  /// Reads one frame pair and accounts for the time, used by both the producer thread and the serial path.
  void decodeInto(std::size_t i, FramePair* item) {
    item->index = i;
    std::string error;
    const auto decode_started = std::chrono::steady_clock::now();
    if (!readFrame(conf_, left_[i], &item->left, &error)) {
      item->left_error = error;
    } else if (stereo_) {
      if (!readFrame(conf_, right_[rightFor(i)], &item->right, &error)) {
        item->right_error = error;
      }
    }
    decode_seconds_ +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - decode_started).count();
  }

  void run() {
#ifdef _WIN32
    // Decoding runs ahead of the tracker; below-normal priority keeps it from taking a core from ORB-SLAM3's
    // extraction/mapping threads when the queue is already full.
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif
    // The queue must cover a bag chunk load, not just a decode.  A 4Seasons chunk is ~144 MB and its
    // synchronous read on the external disk takes a few seconds; at ~24 ms/frame the old depth of 12
    // (0.3 s) drained long before the next chunk arrived, so the tracker stopped at every chunk boundary
    // (the `bag: image topic` lines in the log).  256 frames is ~6 s of this stream and ~33 MB of decoded
    // 800x400 mono Mats; the queue remains bounded for a stopped run.
    const std::size_t depth = 256;
    for (std::size_t i = first_used_; i < left_.size(); ++i) {
      if (left_[i].stamp > end_stamp_) {
        break;
      }
      FramePair item;
      decodeInto(i, &item);
      {
        std::unique_lock<std::mutex> lock(mutex_);
        not_full_.wait(lock, [this, depth]() { return queue_.size() < depth || stopping_; });
        if (stopping_) {
          break;
        }
        queue_.push_back(std::move(item));
      }
      not_empty_.notify_one();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      done_ = true;
    }
    not_empty_.notify_all();
  }

  const Conf& conf_;
  const std::vector<FrameRef>& left_;
  const std::vector<FrameRef>& right_;
  bool stereo_ = false;
  std::size_t first_used_ = 0;
  double end_stamp_ = 1.0e18;
  bool threaded_ = true;        ///< false = decode inline, which is the pre-prefetcher behaviour
  std::size_t next_index_ = 0;  ///< the serial path's cursor
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable not_empty_;
  std::condition_variable not_full_;
  std::deque<FramePair> queue_;
  bool done_ = false;
  bool stopping_ = false;
  double decode_seconds_ = 0.0;   ///< written by the producer, read after `stop()`
  double wait_seconds_ = 0.0;     ///< written by the consumer in next(), read after the loop
};

std::string joinPath(const std::string& a, const std::string& b) {
  if (a.empty()) {
    return b;
  }
  const char last = a.back();
  return (last == '/' || last == '\\') ? a + b : a + "/" + b;
}

/// The rotation angle of a pose, in degrees, printed by the per-frame diagnostic line.
///
/// The monocular stall has the camera's distance from the map origin pinned at ~10.5 units while Tcw's
/// translation swings through metres -- and a translation that swings while its magnitude does not is a
/// rotation, not a movement.  Printing the angle makes that a reading rather than an inference.
float rotationAngleDeg(const Sophus::SE3f& pose) {
  return static_cast<float>(Eigen::AngleAxisf(pose.rotationMatrix()).angle() * 180.0 /
                            3.14159265358979323846);
}

/// ORB-SLAM3's `Camera.type` string for this project's model name.  The two vocabularies differ and the
/// mapping is the whole point of having it in one place.
std::string orbCameraType(const std::string& model) {
  if (model == "kannala_brandt" || model == "kannalabrandt8" || model == "equidistant" ||
      model == "fisheye") {
    return "KannalaBrandt8";
  }
  return "PinHole";
}

/// A floating-point value as OpenCV's FileStorage will hand it back as a real.
///
/// `FileNode::operator float()` -- which ORB-SLAM3's `readParameter<float>` uses -- THROWS on a node it
/// read as an integer, and its own message is "must be a real number, aborting...".  A value of zero
/// formatted by an ostream is "0", which is an integer node, so a camera with no distortion cannot be
/// described at all unless the writer forces a decimal point.  Every real in this file goes through here.
std::string real(double value) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.12g", value);
  std::string text(buffer);
  if (text.find('.') == std::string::npos && text.find('e') == std::string::npos &&
      text.find("inf") == std::string::npos && text.find("nan") == std::string::npos) {
    text += ".0";
  }
  return text;
}

std::string matrixRow(const double* m, int row, int cols) {
  std::string out = "   [";
  for (int c = 0; c < cols; ++c) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), c == 0 ? "%.12g" : ", %.12g", m[row * cols + c]);
    out += buffer;
  }
  out += "]";
  return out;
}

void writeCamera(std::ofstream& out, const CameraConf& cam, const char* prefix) {
  const std::string type = orbCameraType(cam.model);
  out << prefix << ".fx: " << real(cam.fx) << "\n";
  out << prefix << ".fy: " << real(cam.fy) << "\n";
  out << prefix << ".cx: " << real(cam.cx) << "\n";
  out << prefix << ".cy: " << real(cam.cy) << "\n";
  if (type == "KannalaBrandt8") {
    // The equidistant model takes four radial terms and no tangential ones.
    for (int i = 0; i < 4; ++i) {
      const double k = i < static_cast<int>(cam.distortion.size()) ? cam.distortion[(std::size_t)i] : 0.0;
      out << prefix << ".k" << (i + 1) << ": " << real(k) << "\n";
    }
  } else {
    // PinHole: k1 k2 p1 p2 [k3].  ORB-SLAM3 reads `Camera1.k3` only when it is present, so it is written
    // only when the configuration supplied a fifth coefficient -- writing a zero would turn an undistorted
    // camera into a five-coefficient one for no reason.
    const double k1 = cam.distortion.size() > 0 ? cam.distortion[0] : 0.0;
    const double k2 = cam.distortion.size() > 1 ? cam.distortion[1] : 0.0;
    const double p1 = cam.distortion.size() > 2 ? cam.distortion[2] : 0.0;
    const double p2 = cam.distortion.size() > 3 ? cam.distortion[3] : 0.0;
    out << prefix << ".k1: " << real(k1) << "\n";
    out << prefix << ".k2: " << real(k2) << "\n";
    out << prefix << ".p1: " << real(p1) << "\n";
    out << prefix << ".p2: " << real(p2) << "\n";
    if (cam.distortion.size() > 4) {
      out << prefix << ".k3: " << real(cam.distortion[4]) << "\n";
    }
  }
}

}  // namespace

int sensorFor(Mode mode) {
  switch (mode) {
    case Mode::kMono: return ORB_SLAM3::System::MONOCULAR;
    case Mode::kMonoInertial: return ORB_SLAM3::System::IMU_MONOCULAR;
    case Mode::kStereo: return ORB_SLAM3::System::STEREO;
    case Mode::kStereoInertial: return ORB_SLAM3::System::IMU_STEREO;
    case Mode::kEpipolarStereo: return ORB_SLAM3::System::STEREO;
    case Mode::kEpipolarStereoInertial: return ORB_SLAM3::System::IMU_STEREO;
    case Mode::kEpipolarVio: return ORB_SLAM3::System::IMU_STEREO;
    case Mode::kGnss:
    case Mode::kGnssInertial:
    case Mode::kGnssInertialEpipolar:
    case Mode::kGnssInertialLidar:
    case Mode::kGnssInertialLidarEpipolar:
    case Mode::kLvi:
      // The GNSS and LVI routes never construct an ORB-SLAM3 System; the sensor enum is unused.
      return ORB_SLAM3::System::MONOCULAR;
  }
  return ORB_SLAM3::System::MONOCULAR;
}

std::string settingsPathFor(const Conf& conf, const std::string& repo_root) {
  if (!conf.settings_out.empty()) {
    return conf.settings_out;
  }
  // Beside the trajectory, under the configuration's own stem, so a run leaves two files that are
  // obviously a pair and a reader can see exactly which settings produced which trajectory.
  const std::string solution = conf.resolvedSolution(repo_root);
  const std::size_t dot = solution.find_last_of('.');
  const std::string stem = dot == std::string::npos ? solution : solution.substr(0, dot);
  return joinPath(repo_root.empty() ? std::string(".") : repo_root + "/sol", "settings.yaml");
}

bool writeSettings(const Conf& conf, const std::string& path, std::string* error) {
  std::ofstream out(path);
  if (!out) {
    if (error != nullptr) {
      *error = "cannot write the settings file " + path;
    }
    return false;
  }
  const bool stereo = (conf.mode == Mode::kStereo || conf.mode == Mode::kStereoInertial);
  const bool inertial = (conf.mode == Mode::kMonoInertial || conf.mode == Mode::kStereoInertial);

  out << "%YAML:1.0\n";
  out << "# Generated by ALI_GVIL from " << conf.path << ".  Do not hand-edit: every line here comes\n";
  out << "# from a key in that configuration file, and a change made here is lost on the next run.\n";
  out << "File.version: \"1.0\"\n\n";
  // ALI: 1 = anisotropic landmark initialization, 0 = stock ORB-SLAM3.
  out << "ALI: " << (conf.ali ? 1 : 0) << "\n\n";

  out << "Camera.type: \"" << orbCameraType(conf.camera.model) << "\"\n\n";
  writeCamera(out, conf.camera, "Camera1");
  if (stereo) {
    writeCamera(out, conf.camera_right, "Camera2");
  }

  out << "\nCamera.width: " << conf.camera.width << "\n";
  out << "Camera.height: " << conf.camera.height << "\n";
  out << "Camera.fps: " << (conf.camera.fps > 0.0 ? static_cast<int>(std::lround(conf.camera.fps)) : 30)
      << "\n";
  out << "Camera.RGB: 1\n\n";

  if (stereo) {
    // `Stereo.b` is the baseline in metres, and `Stereo.T_c1_c2` the right camera's pose in the left
    // camera's frame.  A rectified pair -- which is what the 4Seasons undistorted set is -- has identity
    // rotation and the baseline along -x, and the configuration can say which it means: naming `baseline`
    // builds the matrix, naming `t_bc_right`/`r_bc_right` supplies it.  Both are written, because
    // ORB-SLAM3 reads both.
    double t[3] = {-conf.baseline, 0.0, 0.0};
    double r[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    if (conf.camera_right.has_extrinsic) {
      // The right camera's pose is given in the BODY frame, and this key wants it in the LEFT camera's
      // frame: T_c1_c2 = T_c1_b * T_b_c2 = (T_b_c1)^-1 * T_b_c2.
      const double* tl = conf.camera.t_bc;
      const double* rl = conf.camera.r_bc;
      const double* tr = conf.camera_right.t_bc;
      const double* rr = conf.camera_right.r_bc;
      // R_c1_b = R_b_c1^T
      double r1t[9];
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          r1t[i * 3 + j] = rl[j * 3 + i];
        }
      }
      // R_c1_c2 = R_c1_b * R_b_c2
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          double sum = 0.0;
          for (int k = 0; k < 3; ++k) {
            sum += r1t[i * 3 + k] * rr[k * 3 + j];
          }
          r[i * 3 + j] = sum;
        }
      }
      // t_c1_c2 = R_c1_b * (t_b_c2 - t_b_c1)
      for (int i = 0; i < 3; ++i) {
        double sum = 0.0;
        for (int k = 0; k < 3; ++k) {
          sum += r1t[i * 3 + k] * (tr[k] - tl[k]);
        }
        t[i] = sum;
      }
    }
    out << "Stereo.b: " << real(conf.has_baseline ? conf.baseline : std::fabs(t[0])) << "\n";
    out << "Stereo.T_c1_c2: !!opencv-matrix\n";
    out << "   rows: 4\n   cols: 4\n   dt: f\n   data: [";
    for (int i = 0; i < 3; ++i) {
      char buffer[256];
      std::snprintf(buffer, sizeof(buffer), "%.12g, %.12g, %.12g, %.12g, ", r[i * 3 + 0], r[i * 3 + 1],
                    r[i * 3 + 2], t[i]);
      out << buffer;
    }
    out << "0.0, 0.0, 0.0, 1.0]\n\n";
    // ORB-SLAM3's stereo depth gate: features nearer than this are treated as reliable depth, the rest as
    // having unknown depth.  Forty baselines is its own default and a reasonable statement about a rig.
    const double baseline = conf.has_baseline ? conf.baseline : std::fabs(t[0]);
    out << "Stereo.ThDepth: " << real(std::max(30.0, 40.0 * baseline)) << "\n\n";
  }

  if (inertial) {
    // `IMU.T_b_c1` is the camera's pose in the BODY frame, 4x4 row major -- the same quantity the
    // configuration calls `r_bc`/`t_bc`.  It is not the body's pose in the camera frame, and getting the
    // direction backwards is the classic way to make a VI run converge to a plausible wrong answer, so the
    // run prints the composed camera-to-body matrix in its log where it can be checked against the dataset.
    out << "IMU.T_b_c1: !!opencv-matrix\n";
    out << "   rows: 4\n   cols: 4\n   dt: f\n   data: [";
    for (int i = 0; i < 3; ++i) {
      char buffer[256];
      std::snprintf(buffer, sizeof(buffer), "%.12g, %.12g, %.12g, %.12g, ", conf.camera.r_bc[i * 3 + 0],
                    conf.camera.r_bc[i * 3 + 1], conf.camera.r_bc[i * 3 + 2], conf.camera.t_bc[i]);
      out << buffer;
    }
    out << "0.0, 0.0, 0.0, 1.0]\n\n";
    out << "IMU.NoiseGyro: " << real(conf.imu.noise_gyro) << "\n";
    out << "IMU.NoiseAcc: " << real(conf.imu.noise_acc) << "\n";
    out << "IMU.GyroWalk: " << real(conf.imu.gyro_walk) << "\n";
    out << "IMU.AccWalk: " << real(conf.imu.acc_walk) << "\n";
    out << "IMU.Frequency: " << real(conf.imu.frequency > 0.0 ? conf.imu.frequency : 200.0) << "\n";
    // The port's own addition (see PATCHES.md): ORB-SLAM3 1.0's settings loader never reads this key, and
    // without it the stereo-inertial initialisation gate cannot be opened by configuration at all -- which
    // for a car, whose average acceleration rarely changes by 0.5 m/s^2 between frames, means a run that
    // never initialises.
    out << "IMU.fastInit: " << (conf.imu.fast_init ? 1 : 0) << "\n";
    out << "IMU.resetMapIfNotInitialized: " << (conf.imu.reset_map_if_imu_not_init ? 1 : 0) << "\n";
    if (conf.imu.gravity > 0.0) {
      out << "IMU.gravity: " << real(conf.imu.gravity) << "\n";
    }
    out << "IMU.InsertKFsWhenLost: " << (conf.imu.insert_kfs_when_lost ? 1 : 0) << "\n";
    // Whether the tracker's per-frame pose solve uses the inertial variant.  Written for every mode; 1 is
    // upstream.  See `imu_inertial_pose_tracking` in ali_conf.h.
    out << "IMU.InertialPoseTracking: " << (conf.imu.inertial_pose_tracking ? 1 : 0) << "\n";
    out << "IMU.EstimateScale: " << (conf.imu.estimate_scale ? 1 : 0) << "\n";
    // 0 = upstream's 10.  A longer window gives a poorly excited start (a car driving straight) enough
    // data for the inertial initialisation to find a physical bias instead of a degenerate one.
    out << "IMU.InitMinKeyFrames: " << conf.imu.init_min_keyframes << "\n";
    // false keeps the map purely visual (the IMU is read and preintegrated but never initialises).
    out << "IMU.UseInertialFactors: " << (conf.imu.use_inertial_factors ? 1 : 0) << "\n\n";
  }

  // Minimum frames between keyframes; 0 is upstream (any frame may be one).
  out << "KeyframeMinFrames: " << conf.keyframe_min_frames << "\n";

  // The monocular initialiser's minimum disparity, for every mode.
  //
  // It lived inside the inertial branch, which put it exactly where it cannot help: the gate is in
  // `Tracking::MonocularInitialization`, so the mode that needs it is MONOCULAR, which never enters
  // that branch.  Measured on this drive, the consequence is a map triangulated from a pair with so
  // little baseline that its points are effectively at infinity, and a tracker that then explains
  // the whole journey as CAMERA ROTATION: the pose's translation stopped changing at t = 130 s and
  // the run finished 313 s later at the same point, with 9487 frames reported as tracked and an
  // image mean that kept moving (46.7 to 85.4 and back) -- the input was fine and the estimate was not.
  out << "IMU.minInitialParallax: " << real(conf.imu.min_initial_parallax) << "\n";
  // 0 = mean, 1 = median.  A number rather than a word because the file is read by the same
  // `readParameter<int>` the rest of the port uses, and because an absent key must leave the mean --
  // which is what the configurations already in the tree were measured with.
  // Read by Tracking's fFastInit block alongside fastInit and the parallax gate.
  out << "IMU.relocalizeBeforeImuInit: "
      << (conf.imu.relocalize_before_imu_init ? 1 : 0) << "\n";
  out << "IMU.minInitialParallaxStatistic: "
      << (conf.imu.parallax_statistic == ImuConf::ParallaxStatistic::kMedian ? 1 : 0) << "\n";
  // The gate that decides whether a monocular-inertial frame still counts as tracking before the inertial
  // unit is up.  Written for every mode for the same reason as the parallax key above; 50 is upstream's
  // value and leaves the configurations already in the tree at the behaviour their results were recorded
  // with.
  out << "IMU.minMatchesInliersPreInit: " << conf.imu.min_inliers_pre_init << "\n";
  // Whether a stationary stretch may reset the active map.  Written for every mode; 1 is upstream.
  out << "IMU.resetIfNotEnoughMotion: " << (conf.imu.reset_if_not_enough_motion ? 1 : 0) << "\n";
  // How much accumulated motion the inertial alignment waits for before it solves for the map scale.
  // 5 and 15 seconds are upstream.
  out << "IMU.firstAlignmentSeconds: " << real(conf.first_alignment_seconds) << "\n";
  out << "IMU.secondAlignmentSeconds: " << real(conf.second_alignment_seconds) << "\n";
    // Monocular maps only: it re-estimates the scale and the gyro bias every ~8 s rather than
    // leaving them where the first initialisation put them.  Written for every mode because the
    // key is inert where it does not apply, and a configuration that names it is stating what it
    // wants rather than what it happens to get.
    out << "slam.scale_refinement: " << (conf.imu.scale_refinement ? 1 : 0) << "\n";

  out << "ORBextractor.nFeatures: " << conf.features << "\n";
  out << "ORBextractor.scaleFactor: " << real(conf.scale_factor) << "\n";
  out << "ORBextractor.nLevels: " << conf.levels << "\n";
  out << "ORBextractor.iniThFAST: " << conf.ini_th_fast << "\n";
  out << "ORBextractor.minThFAST: " << conf.min_th_fast << "\n\n";

  // The viewer block: ORB-SLAM3's Settings asserts on every one of these keys even when the viewer is off,
  // so they are required rather than optional.  This project draws its own window.
  out << "Viewer.KeyFrameSize: " << real(0.05) << "\n";
  out << "Viewer.KeyFrameLineWidth: " << real(1.0) << "\n";
  out << "Viewer.GraphLineWidth: " << real(0.9) << "\n";
  out << "Viewer.PointSize: " << real(2.0) << "\n";
  out << "Viewer.CameraSize: " << real(0.08) << "\n";
  out << "Viewer.CameraLineWidth: " << real(3.0) << "\n";
  out << "Viewer.ViewpointX: " << real(0.0) << "\n";
  out << "Viewer.ViewpointY: " << real(-0.7) << "\n";
  out << "Viewer.ViewpointZ: " << real(-1.8) << "\n";
  out << "Viewer.ViewpointF: " << real(500.0) << "\n";
  out << "Viewer.imageViewScale: " << real(1.0) << "\n";
  return true;
}

RunResult runConf(const Conf& conf, const std::string& repo_root,
                  void (*progress)(const ProgressInfo&, void*), void* progress_user,
                  const std::atomic<bool>* cancel) {
  installCrashHandler();
  g_active_conf_path = conf.path;
  // Test hook for the crash reporter.  It only fires with the environment variable set, and exists so the
  // log/minidump/map path can be verified instead of assumed:
  //   ALI_CRASH_TEST=1            access violation on the worker thread
  //   ALI_CRASH_TEST=terminate    std::terminate handler path
  if (const char* crash_test = std::getenv("ALI_CRASH_TEST")) {
    if (std::strcmp(crash_test, "terminate") == 0) {
      std::terminate();
    }
    volatile int* invalid = nullptr;
    *invalid = 1;
  }
  RunResult result;
  // This build implements ORB-SLAM3's four visual modes and nothing else.  A configuration that names any
  // other mode is refused here rather than silently run through a branch it did not ask for.
  if (conf.mode != Mode::kMono && conf.mode != Mode::kMonoInertial && conf.mode != Mode::kStereo &&
      conf.mode != Mode::kStereoInertial) {
    result.error = std::string("mode '") + modeName(conf.mode) +
                   "' is not built in this project (mono, mono-inertial, stereo, stereo-inertial only)";
    return result;
  }
  const auto started = std::chrono::steady_clock::now();

  // ---- the settings the run actually uses ------------------------------------------------
  result.settings_path = settingsPathFor(conf, repo_root);
  std::string error;
  if (!writeSettings(conf, result.settings_path, &error)) {
    result.error = error;
    return result;
  }

  // ---- the frames ------------------------------------------------------------------------
  std::vector<FrameRef> left;
  if (!collectFrames(conf, &left, &error)) {
    result.error = "frames: " + error;
    return result;
  }
  const bool stereo = (conf.mode == Mode::kStereo || conf.mode == Mode::kStereoInertial);
  std::vector<FrameRef> right;
  if (stereo) {
    if (!collectFramesRight(conf, &right, &error)) {
      result.error = "right frames: " + error;
      return result;
    }
    // Pair by stamp when the two cameras are separate files.  A pair that is not within half a frame period
    // is not a stereo pair, and feeding one would triangulate across an unknown time offset -- so the
    // pairing is exact for a bag (where the two topics share the capture clock) and nearest-within-a-
    // tolerance otherwise, with the count of what was dropped reported.
    const std::string right_source =
        !conf.images_right_dir.empty() ? conf.images_right_dir
        : !conf.images_right.empty()    ? conf.images_right
        : !conf.images_manifest_right.empty() ? conf.images_manifest_right
                                              : std::string("(derived from the left: cam0 -> cam1)");
    jo::logInfoF("ali: resolved left %zu from '%s', right %zu from '%s'", left.size(),
                 conf.images_dir.c_str(), right.size(), right_source.c_str());
    if (left.size() != right.size()) {
      jo::logWarnF("ali: left has %zu frame(s) and right has %zu; pairing by nearest stamp",
                   left.size(), right.size());
    }
  }

  // ---- the inertial data -----------------------------------------------------------------
  const bool inertial = (conf.mode == Mode::kMonoInertial || conf.mode == Mode::kStereoInertial);
  std::vector<ImuSample> imu;
  if (inertial) {
    if (!collectImu(conf, &imu, &error)) {
      result.error = "imu: " + error;
      return result;
    }
    jo::logInfoF("ali: %zu IMU sample(s), %.3f .. %.3f", imu.size(), imu.front().stamp,
                 imu.back().stamp);
    if (imu.back().stamp < left.back().stamp) {
      jo::logWarnF("ali: the inertial data stop at %.3f but the images run to %.3f; the tail of the run "
                   "has no inertial support and will drift",
                   imu.back().stamp, left.back().stamp);
    }
  }

  // ---- start ORB-SLAM3 -------------------------------------------------------------------
  const int sensor = sensorFor(conf.mode);
  jo::logInfoF("ali: starting ORB-SLAM3 in mode %s (sensor %d), %zu frame(s)", modeName(conf.mode), sensor,
               left.size());
  // ORB-SLAM3 keeps the frame / keyframe / map / map-point id counters in statics.  A second System in
  // the same process -- which is what the UI does when the user runs one configuration and then another --
  // would otherwise start with the first run's ids while its own Tracking starts its frame bookkeeping at
  // zero, and the two disagree.  Measured: after a 4Seasons stereo-inertial run, GREAT campus01
  // stereo-inertial initialised a map and then matched 8 of 334 points on the next frame, failing every
  // frame from there.  Each run owns its numbering.
  ORB_SLAM3::Frame::nNextId = 0;
  ORB_SLAM3::KeyFrame::nNextId = 0;
  ORB_SLAM3::Map::nNextId = 0;
  ORB_SLAM3::MapPoint::nNextId = 0;
  ORB_SLAM3::System slam(conf.vocabulary, result.settings_path,
                         static_cast<ORB_SLAM3::System::eSensor>(sensor), conf.viewer);

  // ---- the loop --------------------------------------------------------------------------
  // The trajectory, collected as the tracker produces it.
  //
  // WHY THIS IS WRITTEN HERE AND NOT BY `System::SaveTrajectoryTUM`.  That function reconstructs each
  // frame's pose by walking from the frame's REFERENCE KEYFRAME up the spanning tree
  // (`while(pKF->isBad()) { Trw = Trw * pKF->mTcp; pKF = pKF->GetParent(); }`), and on a map that has been
  // reset -- which is what a monocular run does whenever it loses tracking -- those pointers can be gone.
  // Measured: `ali_ui.exe --run-test confs_mono.conf` reached the end of the run, printed "Saving camera
  // trajectory", and died with an access violation; the same program on the stereo configuration, whose map
  // is never reset, wrote its trajectory and exited cleanly.  Upstream hides this by refusing to write a
  // monocular trajectory at all, so the path was never exercised.
  //
  // The pose `Track*` returns is the same `Twc` the writer emits -- `SaveTrajectoryTUM` only applies the
  // "first keyframe at the origin" anchoring first, and for a monocular map the first keyframe IS the
  // origin by construction.  Writing what the tracker hands back is therefore the same trajectory, one
  // call site, no spanning tree, and no dependence on a keyframe that may have been culled.
  std::vector<double> pose_stamp;
  // Reused across frames so the progress path does not allocate a vector per frame.
  std::vector<float> keypoints_scratch;
  std::vector<float> keypoints_right_scratch;
  std::vector<Sophus::SE3f> pose_twc;
  std::size_t imu_cursor = 0;
  double previous_stamp = left.empty() ? 0.0 : left.front().stamp;
  // The window is counted from the first frame the run will actually USE, not from the sequence's first.
  // It used to be `left.front().stamp + duration`, which made `--start-frame N --duration D` run nothing at
  // all: with a start frame past the end of the window the loop breaks on its first iteration, and the run
  // reports "stopping at frame N, the requested duration is done" having processed no frame.
  const std::size_t first_used =
      std::min(static_cast<std::size_t>(std::max(0, conf.start_frame)), left.empty() ? 0 : left.size() - 1);
  const double end_stamp =
      conf.duration > 0.0 ? left[first_used].stamp + conf.duration : 1.0e18;
  // The frames are decoded one thread ahead of the tracker; see FramePrefetcher.  Everything below is the
  // loop this used to be, with the two `readFrame` calls moved into that thread -- same frames, same order,
  // same failure handling.
  FramePrefetcher prefetcher(conf, left, right, stereo, first_used, end_stamp, conf.prefetch_frames);
  for (;;) {
    FramePair frame;
    if (!prefetcher.next(&frame)) {
      break;
    }
    const std::size_t i = frame.index;
    // Polled at the frame boundary rather than inside the tracker: one frame is the granularity a caller
    // can act on anyway, and the run still shuts the estimator down and writes the poses it has.
    if (cancel != nullptr && cancel->load(std::memory_order_relaxed)) {
      jo::logInfoF("ali: stopping at frame %zu, cancellation was requested", i);
      result.cancelled = true;
      break;
    }
    if (!frame.left_error.empty()) {
      jo::logWarnF("ali: frame %zu: %s", i, frame.left_error.c_str());
      continue;
    }
    cv::Mat& image_left = frame.left;
    cv::Mat& image_right = frame.right;
    if (stereo && !frame.right_error.empty()) {
      jo::logWarnF("ali: frame %zu: right: %s", i, frame.right_error.c_str());
      continue;
    }

    // Every inertial sample up to this frame belongs to the interval that ends here.  This is ORB-SLAM3's
    // own contract in every one of its VI examples: the measurements are passed WITH the image that closes
    // the interval, not before it.
    std::vector<ORB_SLAM3::IMU::Point> measurements;
    if (inertial) {
      const double until = left[i].stamp + conf.imu.time_shift;
      while (imu_cursor < imu.size() && imu[imu_cursor].stamp <= until) {
        const ImuSample& sample = imu[imu_cursor];
        measurements.emplace_back(sample.accel.x, sample.accel.y, sample.accel.z, sample.gyro.x,
                                  sample.gyro.y, sample.gyro.z, sample.stamp);
        ++imu_cursor;
      }
    }

    // The pose the tracker believes now, which is what the live view draws.
    const auto track_started = std::chrono::steady_clock::now();
    Sophus::SE3f tracked;
    if (stereo) {
      tracked = slam.TrackStereo(image_left, image_right, left[i].stamp, measurements);
    } else {
      tracked = slam.TrackMonocular(image_left, left[i].stamp, measurements);
    }
    result.seconds_track += std::chrono::duration<double>(std::chrono::steady_clock::now() - track_started).count();
    ++result.frames;
    previous_stamp = left[i].stamp;
    // A per-frame line every 200 frames: the frame's own brightness, the tracking state, and the pose the
    // tracker returned.  "Is the estimator moving" cannot be answered from the run's totals -- a run can
    // report 9487 tracked frames and a frozen pose, which is what this one did -- so the sequence of states
    // is what says where it stopped.  The image mean is the control: if it changes and the pose does not,
    // the estimator is the problem and not the input.
    if (conf.debug_every > 0 && i % static_cast<std::size_t>(conf.debug_every) == 0) {
      const cv::Scalar mean = cv::mean(image_left);
      const Eigen::Vector3f t_now = tracked.inverse().translation();
      // THE MATCHED-MAP-POINT COUNT AND THE DEPTHS ARE WHAT MAKE THIS LINE DIAGNOSTIC.  A frozen pose with
      // state 2 does not by itself say whether the tracker is still doing anything: `Track*` returns the
      // frame's pose whatever happened, and the state only turns LOST once a failure has persisted, so a
      // pose can sit unchanged for minutes with the state reporting OK.  The two readings separate the
      // possibilities -- a match count near zero means the tracker is idle and the pose is a stale
      // motion-model guess, while a healthy count at runaway depth means the map itself has degenerated and
      // the frame's motion is being explained by rotation alone.
      const std::vector<ORB_SLAM3::MapPoint*> matched = slam.GetTrackedMapPoints();
      int n_matched = 0;
      std::vector<float> depths;
      depths.reserve(matched.size());
      for (std::size_t k = 0; k < matched.size(); ++k) {
        if (matched[k] == nullptr || matched[k]->isBad()) {
          continue;
        }
        ++n_matched;
        const Eigen::Vector3f p_cam = tracked * matched[k]->GetWorldPos();
        if (p_cam.allFinite() && p_cam.z() > 0.0f) {
          depths.push_back(p_cam.z());
        }
      }
      float depth_median = 0.0f;
      if (!depths.empty()) {
        std::sort(depths.begin(), depths.end());
        depth_median = depths[depths.size() / 2];
      }
      // The matched depths as a coarse histogram.  A median alone cannot tell a distribution that moved
      // from one that split: the monocular runs alternate between a median of 0.3 and one of 45 on
      // consecutive frames, which is what a bimodal set looks like once the middle is being reported.
      int near_bin = 0, mid_bin = 0, far_bin = 0, very_far_bin = 0;
      for (std::size_t k = 0; k < depths.size(); ++k) {
        if (depths[k] < 1.0f) ++near_bin;
        else if (depths[k] < 5.0f) ++mid_bin;
        else if (depths[k] < 20.0f) ++far_bin;
        else ++very_far_bin;
      }
      int map_keyframes = 0;
      int map_points = 0;
      bool mapper_accepts = false;
      slam.GetMapSize(&map_keyframes, &map_points, &mapper_accepts);
      double map_spread = 0.0;
      int spread_points = 0;
      int queue = 0;
      slam.GetMapSpread(&map_spread, &spread_points, &queue);
      int local_keyframes = 0;
      int local_points = 0;
      slam.GetLocalMapSize(&local_keyframes, &local_points);
      // How much of the pose is rotation.  The monocular stall has the camera's distance from the map
      // origin pinned while Tcw's translation swings, which is a rotation; printing the angle makes that
      // visible instead of inferable, and its rate is what "rotating in place" means quantitatively.
      const float rot_deg = static_cast<float>(std::fabs(rotationAngleDeg(tracked.inverse())));
      jo::logInfoF("ali: frame %5zu t %.3f  mean %6.2f  state %d  Twc (%.3f, %.3f, %.3f)  "
                   "rot %7.2f deg  inliers %4d  matched %4d  keys %4zu  depth med %7.2f  bins %d/%d/%d/%d  "
                   "map %5d KF %6d MP  local %4d KF %5d MP  spread %8.3f  queue %2d  %s",
                   i, left[i].stamp, mean[0], slam.GetTrackingState(), t_now.x(), t_now.y(), t_now.z(),
                   rot_deg, slam.GetMatchesInliers(), n_matched, slam.GetTrackedKeyPointsUn().size(),
                   depth_median, near_bin, mid_bin, far_bin, very_far_bin, map_keyframes, map_points,
                   local_keyframes, local_points, map_spread, queue,
                   mapper_accepts ? "accepting" : "holding");
      std::fflush(stdout);
    }
    // Tracking::OK -- Tracking.h's eTrackingState.  Declared here because both the writer below and the
    // progress view use it, and they must use the SAME rule or the live picture and the file disagree.
    const int kTrackingOk = 2;
    {
      // ONLY WHILE THE TRACKER IS TRACKING.  `TrackMonocular` returns the current frame's pose whatever the
      // tracking state is, and once the tracker goes LOST that pose stops changing: it is the last one it
      // believed, or the identity, and it is not a measurement.  Recording those frames drew a trajectory
      // that freezes half-way through the drive and then runs flat to the end -- measured on the mono full
      // drive, the pose sat at (-8.66, 0.09, 0.70) from t = 130 s to t = 313 s while the run reported 9487
      // tracked frames.  ORB-SLAM3's own writer skips exactly these frames (`if(*lbL) continue;`, reading
      // the per-frame lost flag from the tracker), which is why replacing it changed the answer; the state
      // is the same information through a public accessor.
      if (slam.GetTrackingState() == kTrackingOk) {
        const Sophus::SE3f twc = tracked.inverse();
        const Eigen::Vector3f t = twc.translation();
        if (t.allFinite() && std::fabs(t.x()) < 1.0e6f && std::fabs(t.y()) < 1.0e6f &&
            std::fabs(t.z()) < 1.0e6f) {
          pose_stamp.push_back(left[i].stamp);
          pose_twc.push_back(twc);
        }
      }
    }
    if (progress != nullptr) {
      ProgressInfo info;
      info.frame = i;
      info.frames = left.size();
      info.stamp = left[i].stamp;
      // `Track*` returns Tcw; the trajectory is written as Twc, and the live view draws the same thing so
      // that the picture during a run and the file after it are the same shape.
      const Sophus::SE3f twc = tracked.inverse();
      const Eigen::Vector3f t = twc.translation();
      // THE SAME RULE THE WRITER USES, and it did not.  This used to be "the pose is finite", which is true
      // of every frame -- `Track*` returns a pose whatever happened -- so the live view also plotted the
      // RECENTLY_LOST frames, whose pose is the last value the tracker believed or the identity.  That is the
      // small local jitter visible while a run is in progress: the finished picture is redrawn from the file
      // when the run ends, and the file never had those points.
      info.tracked = (slam.GetTrackingState() == kTrackingOk) && t.allFinite() && std::fabs(t.x()) < 1.0e6f;
      info.x = t.x();
      info.y = t.y();
      info.z = t.z();
      if (image_left.isContinuous() && image_left.type() == CV_8UC1) {
        info.gray = image_left.data;
        info.width = image_left.cols;
        info.height = image_left.rows;
        // The tracker's keypoints, so a view does not have to re-detect them.  Only fetched when the caller
        // asked for the image, which is the same condition the UI's feature view runs under.
        const std::vector<cv::KeyPoint> tracked_keys = slam.GetTrackedKeyPointsUn();
        keypoints_scratch.clear();
        keypoints_scratch.reserve(tracked_keys.size() * 2);
        for (const cv::KeyPoint& kp : tracked_keys) {
          keypoints_scratch.push_back(kp.pt.x);
          keypoints_scratch.push_back(kp.pt.y);
        }
        info.keypoints_xy = keypoints_scratch.empty() ? nullptr : keypoints_scratch.data();
        info.keypoint_count = tracked_keys.size();
      }
      // The right camera of a stereo run was never handed to the view: only the epipolar branch filled these
      // fields, so after an epipolar/mono run the right window kept the previous dataset's image while the
      // header showed the current frame.  Publish the current right image and its ORB keypoints here.
      if (stereo && !image_right.empty() && image_right.isContinuous() &&
          image_right.type() == CV_8UC1) {
        info.gray_right = image_right.data;
        info.width_right = image_right.cols;
        info.height_right = image_right.rows;
        info.gray_right_mat = image_right;
        const std::vector<cv::KeyPoint> tracked_right = slam.GetTrackedKeyPointsRight();
        keypoints_right_scratch.clear();
        keypoints_right_scratch.reserve(tracked_right.size() * 2);
        for (const cv::KeyPoint& kp : tracked_right) {
          keypoints_right_scratch.push_back(kp.pt.x);
          keypoints_right_scratch.push_back(kp.pt.y);
        }
        info.keypoints_right_xy =
            keypoints_right_scratch.empty() ? nullptr : keypoints_right_scratch.data();
        info.keypoint_right_count = tracked_right.size();
      }
      {
        // Timed separately because the UI's callback re-detects ORB features for its view, which is real
        // work on every frame and would otherwise be charged to the tracker.
        const auto callback_started = std::chrono::steady_clock::now();
        progress(info, progress_user);
        result.seconds_progress +=
            std::chrono::duration<double>(std::chrono::steady_clock::now() - callback_started).count();
      }
    }
  }
  // Join the producer before reading its total, and before the estimate is shut down, so the number reported
  // is the whole decode cost including the frames produced ahead of a stop.
  prefetcher.stop();
  result.seconds_decode = prefetcher.decodeSeconds();
  result.seconds_io_wait = prefetcher.waitSeconds();
  result.prefetched = conf.prefetch_frames;
  (void)previous_stamp;

  // ---- the trajectory --------------------------------------------------------------------
  const std::string solution = conf.resolvedSolution(repo_root);
  {
    std::ofstream out(solution);
    if (!out) {
      result.error = "cannot write " + solution;
      slam.Shutdown();
      return result;
    }
    // ORB-SLAM3's own format and precision: `fixed` with six decimals, which is what its writer emits and
    // what the scorers in `tools/` read.  The timestamps are Unix-epoch seconds, so the six decimals are
    // the sub-second part and a stream default would destroy them.
    out << std::fixed << std::setprecision(6);
    out << "# timestamp x y z qx qy qz qw\n";
    for (std::size_t k = 0; k < pose_stamp.size(); ++k) {
      const Eigen::Vector3f t = pose_twc[k].translation();
      const Eigen::Quaternionf q = pose_twc[k].unit_quaternion();
      out << pose_stamp[k] << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " "
          << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    result.poses = pose_stamp.size();
  }
  result.solution_path = solution;
  {
    // Count what was written, so that "the run produced no pose" is answered by the file rather than by a
    // return code: a run that tracks frames and writes no pose is a run that never initialised.
    std::ifstream in(solution);
    std::string line;
    std::size_t lines = 0;
    while (std::getline(in, line)) {
      if (!line.empty() && line[0] != '#') {
        ++lines;
      }
    }
    result.poses = lines;
  }

  slam.Shutdown();

  // ALI's own counter: how many landmarks were initialised through the anisotropic path.  A run that asked
  // for ALI and reports zero has not exercised it, which is a fact worth seeing rather than inferring.
  if (conf.ali) {
    jo::logInfoF("ali: %lu map point(s) triangulated anisotropically", ORB_SLAM3::LocalMapping::ALITriangulations());
  }

  const auto finished = std::chrono::steady_clock::now();
  result.wall_seconds = std::chrono::duration<double>(finished - started).count();
  // A cancelled run is not a failed one -- it did what it was asked -- and it can legitimately hold no
  // pose, because the stop can land before the monocular initialisation has fired.
  result.ok = result.poses > 0 || result.cancelled;
  if (!result.ok) {
    result.error = "the run produced no pose -- the tracker did not initialise";
  }
  return result;
}

}  // namespace ali
