// ali_vins -- the application.
//
// Two front ends over one implementation:
//
//   ali_vins_app.exe                     the window: pick a configuration from config/, read what it points at,
//                                            press Run, watch the estimator's output, see the score
//   ali_vins_app.exe --list              print the configurations that were found, with their paths checked
//   ali_vins_app.exe --run <conf> [--no-score]
//                                            run one configuration without a window
//
// The headless modes are not a separate afterthought: they drive the same RunConf, the same argument building and
// the same scoring call as the buttons do, so the parts of the application that can be wrong -- path resolution,
// argument construction, output naming, scoring -- are the parts that can be tested without a display.

#ifdef _WIN32
// windows.h defines min() and max() as macros, which then break every std::min/std::max in this file.  NOMINMAX is
// the documented way to ask for the functions instead of the macros; without it the plot's scaling code does not
// compile at all.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commctrl.h>
#include <commdlg.h>
#include <shellapi.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

// --------------------------------------------------------------------------------------------- small utilities

std::string Trim(const std::string& s) {
  std::size_t begin = 0;
  std::size_t end = s.size();
  while (begin < end && std::isspace(static_cast<unsigned char>(s[begin]))) ++begin;
  while (end > begin && std::isspace(static_cast<unsigned char>(s[end - 1]))) --end;
  return s.substr(begin, end - begin);
}

std::string ToLower(std::string s) {
  for (std::size_t i = 0; i < s.size(); ++i) s[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(s[i])));
  return s;
}

bool IsAbsolutePath(const std::string& p) {
  if (p.empty()) return false;
  if (p[0] == '/' || p[0] == '\\') return true;
  return p.size() > 1 && p[1] == ':';
}

std::string JoinPath(const std::string& a, const std::string& b) {
  if (a.empty()) return b;
  if (b.empty()) return a;
  if (IsAbsolutePath(b)) return b;
  std::string out = a;
  if (out.back() != '/' && out.back() != '\\') out += '\\';
  return out + b;
}

/// Windows accepts either separator, but a configuration file written with '/' and a command line built with '\'
/// look like two different paths when they are printed side by side, which is needless confusion in the window.
std::string NormaliseSeparators(std::string path) {
#ifdef _WIN32
  std::replace(path.begin(), path.end(), '/', '\\');
#endif
  return path;
}

bool DirectoryExists(const std::string& path) {
  if (path.empty()) return false;
#ifdef _WIN32
  const DWORD attributes = GetFileAttributesA(path.c_str());
  return attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
#else
  return FileExists(path);
#endif
}

std::string DirName(const std::string& path) {
  const std::size_t slash = path.find_last_of("/\\");
  return slash == std::string::npos ? std::string() : path.substr(0, slash);
}

std::string BaseName(const std::string& path) {
  const std::size_t slash = path.find_last_of("/\\");
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

std::string StripExtension(const std::string& name) {
  const std::size_t dot = name.find_last_of('.');
  return dot == std::string::npos ? name : name.substr(0, dot);
}

bool FileExists(const std::string& path) {
  std::ifstream probe(path.c_str(), std::ios::binary);
  return probe.good();
}

/// Current size of a file in bytes, or 0 when it does not exist or cannot be read.  Used to decide whether the
/// trajectory being written has grown since the last redraw, which is a far cheaper question than parsing it.
unsigned long long FileSizeOrZero(const std::string& path) {
  std::ifstream stream(path.c_str(), std::ios::binary | std::ios::ate);
  if (!stream) return 0;
  const std::streamoff size = stream.tellg();
  return size > 0 ? static_cast<unsigned long long>(size) : 0;
}

std::string ReadWholeFile(const std::string& path) {
  std::ifstream stream(path.c_str(), std::ios::binary);
  if (!stream) return std::string();
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

/// "20261013_101530" -- enough to order runs, and it never contains a separator.
std::string Timestamp() {
  std::time_t now = std::time(nullptr);
  std::tm local = *std::localtime(&now);
  char text[32];
  std::strftime(text, sizeof(text), "%Y%m%d_%H%M%S", &local);
  return text;
}

/// Where this executable lives.  Everything else is found relative to it, so the application works from any
/// working directory -- including a double-click from Explorer, which does not set one.
std::string ExecutableDirectory() {
#ifdef _WIN32
  char buffer[MAX_PATH];
  const DWORD length = GetModuleFileNameA(nullptr, buffer, MAX_PATH);
  if (length == 0 || length >= MAX_PATH) return std::string();
  return DirName(buffer);
#else
  return std::string();
#endif
}

std::string ProjectRoot() { return DirName(ExecutableDirectory()); }

// --------------------------------------------------------------------------------------------- configuration

struct RunConf {
  std::string path;  // the .conf file itself
  std::string name;
  std::string settings;
  // ALI-VINS uses one estimator: KLT tracking and MSCKF + SLAM design adopted from OpenVINS, with GFTT detection and ALI.
  // The key is still parsed so configurations written for the original schema keep working.
  std::string engine = "openvins";
  std::string dataset_root;
  std::string manifest;
  std::string imu;
  std::string reference;
  std::string output;
  // Optional: the ROS bag the dataset came from, and where its extraction lives.  When the manifest is missing and
  // a bag is named, the application extracts the bag first -- so a configuration can be written against the bag
  // itself rather than against files somebody has to remember to produce.
  std::string bag;
  std::string work_dir;
  int frames = 0;
  int start = 0;
  int window = 8;
  int max_features = -1;  // -1 = keep the YAML's num_pts
  double time_scale = 1.0;

  // Resolved against the project root (settings, output, dataset_root, bag, work_dir).
  std::string settings_path;
  std::string output_dir;
  std::string dataset_root_path;
  std::string bag_path;
  std::string work_dir_path;
  // Resolved against dataset_root (manifest, imu, reference) -- they are dataset files.
  std::string manifest_path;
  std::string imu_path;
  std::string reference_path;
};

bool ParseConf(const std::string& path, RunConf& conf, std::string& error) {
  std::ifstream stream(path.c_str());
  if (!stream) {
    error = "cannot open " + path;
    return false;
  }
  conf.path = path;
  std::map<std::string, std::string> values;
  std::string line;
  while (std::getline(stream, line)) {
    const std::size_t comment = line.find('#');
    if (comment != std::string::npos) line = line.substr(0, comment);
    line = Trim(line);
    if (line.empty()) continue;
    const std::size_t equals = line.find('=');
    if (equals == std::string::npos) continue;
    values[ToLower(Trim(line.substr(0, equals)))] = Trim(line.substr(equals + 1));
  }

  const std::string fallback_name = StripExtension(BaseName(path));
  conf.name = values.count("name") ? values["name"] : fallback_name;
  conf.settings = values["settings"];
  conf.dataset_root = values["dataset_root"];
  conf.manifest = values["manifest"];
  conf.imu = values["imu"];
  conf.reference = values["reference"];
  conf.output = values.count("output") ? values["output"] : "sol";
  conf.engine = values.count("engine") ? ToLower(values["engine"]) : "legacy";
  conf.bag = values["bag"];
  conf.work_dir = values["work_dir"];
  if (values.count("frames")) conf.frames = std::atoi(values["frames"].c_str());
  if (values.count("start")) conf.start = std::atoi(values["start"].c_str());
  if (values.count("window")) conf.window = std::atoi(values["window"].c_str());
  if (values.count("max_features")) conf.max_features = std::atoi(values["max_features"].c_str());
  if (values.count("time_scale")) conf.time_scale = std::atof(values["time_scale"].c_str());

  if (conf.settings.empty() || conf.manifest.empty()) {
    error = path + ": needs at least 'settings' and 'manifest'";
    return false;
  }

  const std::string root = ProjectRoot();
  conf.settings_path = NormaliseSeparators(JoinPath(root, conf.settings));
  conf.output_dir = NormaliseSeparators(JoinPath(root, conf.output));
  conf.dataset_root_path = NormaliseSeparators(JoinPath(root, conf.dataset_root));
  conf.manifest_path = NormaliseSeparators(JoinPath(conf.dataset_root_path, conf.manifest));
  conf.imu_path =
      conf.imu.empty() ? std::string() : NormaliseSeparators(JoinPath(conf.dataset_root_path, conf.imu));
  conf.reference_path = conf.reference.empty()
                            ? std::string()
                            : NormaliseSeparators(JoinPath(conf.dataset_root_path, conf.reference));
  conf.bag_path = conf.bag.empty() ? std::string() : NormaliseSeparators(JoinPath(root, conf.bag));
  conf.work_dir_path = conf.work_dir.empty() ? std::string() : NormaliseSeparators(JoinPath(root, conf.work_dir));
  return true;
}

/// Every *.conf in config/, in name order.  Sorted because a list that reorders itself between launches is
/// needlessly hard to use.
std::vector<RunConf> DiscoverConfs(std::string& warning) {
  std::vector<RunConf> confs;
  const std::string directory = JoinPath(ProjectRoot(), "config");
#ifdef _WIN32
  WIN32_FIND_DATAA found;
  const std::string pattern = directory + "\\*.conf";
  HANDLE search = FindFirstFileA(pattern.c_str(), &found);
  if (search == INVALID_HANDLE_VALUE) {
    warning = "no config/ directory next to the executable's parent (" + directory + ")";
    return confs;
  }
  do {
    if (found.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;
    RunConf conf;
    std::string error;
    if (ParseConf(JoinPath(directory, found.cFileName), conf, error)) {
      confs.push_back(conf);
    } else {
      warning += error + "\n";
    }
  } while (FindNextFileA(search, &found));
  FindClose(search);
#else
  warning = "configuration discovery is Windows-only in this build";
#endif
  std::sort(confs.begin(), confs.end(),
            [](const RunConf& a, const RunConf& b) { return a.name < b.name; });
  return confs;
}

/// The detail shown when a configuration is selected (and printed by --list).  Paths are annotated with whether
/// they exist, because the most common way to get a confusing failure is a configuration pointing somewhere that
/// is not there, and saying so up front is cheaper than reading it out of the estimator's log.
std::string Describe(const RunConf& conf) {
  std::ostringstream out;
  out << "name        : " << conf.name << "\n";
  out << "conf file   : " << conf.path << "\n";
  out << "engine      : " << conf.engine
      << (conf.engine == "openvins" ? "  (GFTT + KLT, MSCKF + SLAM, ALI)" : "  (legacy engine)") << "\n";
  out << "settings    : " << conf.settings_path << (FileExists(conf.settings_path) ? "  [ok]" : "  [MISSING]")
      << "\n";
  out << "dataset root: " << conf.dataset_root_path
      << (conf.dataset_root_path.empty()
              ? "  [not set; dataset paths must then be absolute]"
              : (DirectoryExists(conf.dataset_root_path) ? "  [ok]" : "  [MISSING]"))
      << "\n";
  out << "manifest    : " << conf.manifest_path << (FileExists(conf.manifest_path) ? "  [ok]" : "  [MISSING]")
      << "\n";
  if (!conf.bag_path.empty()) {
    out << "bag         : " << conf.bag_path << (FileExists(conf.bag_path) ? "  [ok]" : "  [MISSING]") << "\n";
    out << "work dir    : "
        << (conf.work_dir_path.empty() ? std::string("(unset; required to extract a bag)") : conf.work_dir_path)
        << (conf.work_dir_path.empty()
                ? ""
                : (DirectoryExists(conf.work_dir_path) ? "  [ok]" : "  [will be created]"))
        << "\n";
    if (!FileExists(conf.manifest_path) && FileExists(conf.bag_path) && !conf.work_dir_path.empty()) {
      out << "              (the manifest is not built yet; Run will extract the bag into the work dir first)\n";
    }
  }
  if (conf.imu_path.empty()) {
    out << "imu         : (none; the estimator will run without inertial data)\n";
  } else {
    out << "imu         : " << conf.imu_path << (FileExists(conf.imu_path) ? "  [ok]" : "  [MISSING]") << "\n";
  }
  if (conf.reference_path.empty()) {
    out << "reference   : (none; the run will not be scored)\n";
  } else {
    out << "reference   : " << conf.reference_path
        << (FileExists(conf.reference_path) ? "  [ok]" : "  [MISSING, scoring will be skipped]") << "\n";
  }
  out << "frames      : " << (conf.frames > 0 ? std::to_string(conf.frames) : std::string("all")) << "\n";
  out << "max features: " << (conf.max_features > 0 ? std::to_string(conf.max_features) : std::string("(from YAML)"))
      << "\n";
  out << "start       : " << conf.start << "\n";
  out << "window      : " << conf.window << " keyframe(s)\n";
  out << "time scale  : " << conf.time_scale << " (trajectory timestamps -> seconds for the scorer)\n";
  out << "output      : " << conf.output_dir << "\n";
  return out.str();
}

// --------------------------------------------------------------------------------------------- trajectories

struct Trajectory {
  std::vector<double> time, x, y, z;
  std::size_t size() const { return time.size(); }
  bool empty() const { return time.empty(); }
};

/// The estimator's own output, and route_e_r2's: whitespace-separated, "<timestamp> x y z [more]".  Extra columns
/// are ignored rather than rejected, because route_e_r2 appends a quaternion and a viewer wants to be able to open
/// either branch's file.
bool LoadTrajectory(const std::string& path, Trajectory& out, std::string& error) {
  std::ifstream stream(path.c_str());
  if (!stream) {
    error = "cannot open " + path;
    return false;
  }
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream parser(line);
    double t = 0.0, x = 0.0, y = 0.0, z = 0.0;
    if (!(parser >> t >> x >> y >> z)) continue;
    out.time.push_back(t);
    out.x.push_back(x);
    out.y.push_back(y);
    out.z.push_back(z);
  }
  if (out.empty()) {
    error = "no poses in " + path;
    return false;
  }
  return true;
}

/// Ground truth, as this workspace stores it: a header line beginning '#', then
/// "t_seconds,x,y,z,qx,qy,qz,qw" with commas.
bool LoadReference(const std::string& path, Trajectory& out, std::string& error) {
  std::ifstream stream(path.c_str());
  if (!stream) {
    error = "cannot open " + path;
    return false;
  }
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::vector<double> values;
    std::string field;
    std::stringstream parser(line);
    while (std::getline(parser, field, ',')) values.push_back(std::atof(field.c_str()));
    if (values.size() < 4) continue;
    out.time.push_back(values[0]);
    out.x.push_back(values[1]);
    out.y.push_back(values[2]);
    out.z.push_back(values[3]);
  }
  if (out.empty()) {
    error = "no poses in " + path;
    return false;
  }
  return true;
}

struct Alignment {
  bool ok = false;
  std::string reason;
  double yaw_degrees = 0.0;
  double offset_x = 0.0, offset_y = 0.0, offset_z = 0.0;
  std::size_t matched = 0;
  double rms_3d = 0.0;
  double span_seconds = 0.0;
  std::vector<double> aligned_x, aligned_y, aligned_z;  // the estimate, after alignment
};

/// Aligns an estimate onto the reference the way the scorer does: a rotation about the vertical axis and a
/// translation, with the scale left at 1.
///
/// Scale is deliberately *not* fitted.  Fitting it would make a trajectory that is uniformly too small look
/// perfect, and this branch's known weakness is exactly that its scale is compressed -- so a display that hid it
/// would hide the one thing worth seeing.
///
/// Correspondences are the nearest reference sample in time, within `max_gap` seconds, which is also what the
/// scorer pairs on.  With the pairs fixed, the yaw has a closed form: the angle that maximises the sum of the
/// cross/dot products of the centred point pairs.
Alignment AlignEstimateToReference(const Trajectory& estimate, const Trajectory& reference, double max_gap) {
  Alignment result;
  if (estimate.empty() || reference.empty()) {
    result.reason = "one of the trajectories is empty";
    return result;
  }

  std::vector<double> ex, ey, ez, rx, ry, rz;
  double first_time = 0.0, last_time = 0.0;
  bool have_time = false;
  for (std::size_t i = 0; i < estimate.size(); ++i) {
    // Nearest reference sample in time.  The reference is ordered, so a linear scan with a moving start would do,
    // but these are hundreds of samples and a scan from the beginning is clearer and fast enough.
    std::size_t best = 0;
    double best_gap = 0.0;
    bool found = false;
    for (std::size_t j = 0; j < reference.size(); ++j) {
      const double gap = std::fabs(reference.time[j] - estimate.time[i]);
      if (!found || gap < best_gap) {
        best = j;
        best_gap = gap;
        found = true;
      }
    }
    if (!found || best_gap > max_gap) continue;
    ex.push_back(estimate.x[i]);
    ey.push_back(estimate.y[i]);
    ez.push_back(estimate.z[i]);
    rx.push_back(reference.x[best]);
    ry.push_back(reference.y[best]);
    rz.push_back(reference.z[best]);
    if (!have_time) {
      first_time = estimate.time[i];
      have_time = true;
    }
    last_time = estimate.time[i];
  }

  result.matched = ex.size();
  result.span_seconds = have_time ? last_time - first_time : 0.0;
  if (ex.size() < 3) {
    result.reason = "fewer than 3 poses could be matched to the reference in time";
    return result;
  }

  double ex_mean = 0.0, ey_mean = 0.0, ez_mean = 0.0;
  double rx_mean = 0.0, ry_mean = 0.0, rz_mean = 0.0;
  for (std::size_t i = 0; i < ex.size(); ++i) {
    ex_mean += ex[i]; ey_mean += ey[i]; ez_mean += ez[i];
    rx_mean += rx[i]; ry_mean += ry[i]; rz_mean += rz[i];
  }
  const double n = static_cast<double>(ex.size());
  ex_mean /= n; ey_mean /= n; ez_mean /= n;
  rx_mean /= n; ry_mean /= n; rz_mean /= n;

  double sum_cross = 0.0, sum_dot = 0.0;
  for (std::size_t i = 0; i < ex.size(); ++i) {
    const double px = ex[i] - ex_mean, py = ey[i] - ey_mean;
    const double qx = rx[i] - rx_mean, qy = ry[i] - ry_mean;
    sum_cross += px * qy - py * qx;
    sum_dot += px * qx + py * qy;
  }
  const double yaw = std::atan2(sum_cross, sum_dot);
  const double c = std::cos(yaw), s = std::sin(yaw);

  result.yaw_degrees = yaw * 180.0 / 3.14159265358979323846;
  result.offset_x = rx_mean - (c * ex_mean - s * ey_mean);
  result.offset_y = ry_mean - (s * ex_mean + c * ey_mean);
  result.offset_z = rz_mean - ez_mean;

  result.aligned_x.resize(estimate.size());
  result.aligned_y.resize(estimate.size());
  result.aligned_z.resize(estimate.size());
  for (std::size_t i = 0; i < estimate.size(); ++i) {
    result.aligned_x[i] = c * estimate.x[i] - s * estimate.y[i] + result.offset_x;
    result.aligned_y[i] = s * estimate.x[i] + c * estimate.y[i] + result.offset_y;
    result.aligned_z[i] = estimate.z[i] + result.offset_z;
  }

  // RMS over the matched pairs only, which is what the number describes.
  double sum = 0.0;
  for (std::size_t i = 0; i < ex.size(); ++i) {
    const double dx = (c * ex[i] - s * ey[i] + result.offset_x) - rx[i];
    const double dy = (s * ex[i] + c * ey[i] + result.offset_y) - ry[i];
    const double dz = (ez[i] + result.offset_z) - rz[i];
    sum += dx * dx + dy * dy + dz * dz;
  }
  result.rms_3d = std::sqrt(sum / n);
  result.ok = true;
  return result;
}

std::string DescribeAlignment(const Alignment& alignment, std::size_t estimate_poses,
                              std::size_t reference_poses) {
  std::ostringstream out;
  out << "estimate poses : " << estimate_poses << "\n";
  out << "reference poses: " << reference_poses << "\n";
  if (!alignment.ok) {
    out << "alignment      : not possible - " << alignment.reason << "\n";
    return out.str();
  }
  out << "matched pairs  : " << alignment.matched << " over " << alignment.span_seconds << " s\n";
  out << "yaw            : " << alignment.yaw_degrees << " deg\n";
  out << "translation    : (" << alignment.offset_x << ", " << alignment.offset_y << ", " << alignment.offset_z
      << ")\n";
  out << "RMS 3D         : " << alignment.rms_3d << " m\n";
  return out.str();
}

// --------------------------------------------------------------------------------------------- running

/// Set from the command line (--plot / --ref) so the window can open already showing a result.  That is convenient
// after the fact, and it is also what makes the drawing itself checkable: a plot that only appears after a
// multi-minute run cannot be inspected by anything except a person waiting for it.
std::string g_startup_estimate;
std::string g_startup_reference;

/// Set from the command line (--run <conf> --window) so the window opens and starts that run itself.  Mainly this is
/// so the live plotting can be exercised and observed without anyone clicking: a feature whose only test is "a
/// person watched it once" is not tested.
std::string g_startup_run_conf;
/// --frames, applied to the windowed run as well as the headless one, so a check of a few seconds does not have to
/// edit the configuration or wait out a full drive.
int g_startup_frame_override = -1;
/// Set with --window.  In that mode there is a console, and the live status lines go to it as well as to the status
/// bar, which is what makes a headless check of the live plot possible.
bool g_echo_status_to_stdout = false;

/// The runner executable sits next to this one, in bin/.  Keeping the estimator in its own process is deliberate:
/// this branch has crashed during development, and a crash in a child process is reported in the log and the
/// window survives, which is not true of a crash in a worker thread.
///
/// ALI-VINS uses KLT tracking and MSCKF + SLAM design adopted from OpenVINS, with GFTT detection and ALI.  The engine key is
/// still read so existing configurations keep their schema, but it selects the same runner.
std::string RunnerExecutable(const std::string& engine) {
  (void)engine;
  return JoinPath(ExecutableDirectory(), "ali_vins_run.exe");
}

struct RunResult {
  bool started = false;
  unsigned long exit_code = 0;
  std::string trajectory_path;
  std::string log_path;
  std::string score_path;
  std::string reference_path;  // carried so the plot finds the ground truth even if the selection moved on
  std::string score;      // the scorer's output, or the reason it was skipped
  bool scored = false;
  std::string error;
};

/// Builds the runner's command line from a configuration.  Separated from the spawn so that --list and the
/// window cannot disagree about what the arguments are.
///
/// The runner reads one YAML (which names its two calibration files) and takes the frame cap from the
/// configuration's max_features key.
std::string BuildRunnerCommandLine(const RunConf& conf, const std::string& trajectory_out) {
  std::ostringstream command;
  command << '"' << RunnerExecutable(conf.engine) << '"';
  command << " --manifest \"" << conf.manifest_path << '"';
  command << " --config \"" << conf.settings_path << '"';
  if (!conf.imu_path.empty()) command << " --imu \"" << conf.imu_path << '"';
  command << " --out \"" << trajectory_out << '"';
  if (!conf.dataset_root_path.empty()) command << " --base-dir \"" << conf.dataset_root_path << '"';
  if (conf.frames > 0) command << " --frames " << conf.frames;
  if (conf.start > 0) command << " --start " << conf.start;
  if (conf.max_features > 0) command << " --max-features " << conf.max_features;
  return command.str();
}

#ifdef _WIN32
/// The estimator currently running, so that Stop can terminate it.  A file-scope handle rather than a field on the
/// window state because the same runner is used headlessly, where there is no window state to put it in.
volatile HANDLE g_current_child_process = nullptr;

/// Runs a process, merging its stdout and stderr into one stream and handing each chunk to `sink` as it arrives.
/// Returns false only when the process could not be started; a non-zero exit code is a normal result.
bool RunProcess(const std::string& command_line, const std::string& working_directory,
                const std::function<void(const std::string&)>& sink, unsigned long& exit_code,
                std::string& error) {
  SECURITY_ATTRIBUTES attributes;
  attributes.nLength = sizeof(attributes);
  attributes.lpSecurityDescriptor = nullptr;
  attributes.bInheritHandle = TRUE;

  HANDLE read_end = nullptr;
  HANDLE write_end = nullptr;
  if (!CreatePipe(&read_end, &write_end, &attributes, 0)) {
    error = "CreatePipe failed";
    return false;
  }
  // The reader must not be inherited, or the child would hold it open and the reader would never see end-of-file.
  SetHandleInformation(read_end, HANDLE_FLAG_INHERIT, 0);

  STARTUPINFOA startup;
  std::memset(&startup, 0, sizeof(startup));
  startup.cb = sizeof(startup);
  startup.dwFlags = STARTF_USESTDHANDLES | STARTF_USESHOWWINDOW;
  startup.wShowWindow = SW_HIDE;
  startup.hStdOutput = write_end;
  startup.hStdError = write_end;
  startup.hStdInput = nullptr;

  PROCESS_INFORMATION process;
  std::memset(&process, 0, sizeof(process));

  std::vector<char> mutable_command(command_line.begin(), command_line.end());
  mutable_command.push_back('\0');

  // Build the child's environment explicitly rather than inheriting it, so that a run started from the window gets
  // the same thread settings as the runs this project's numbers were measured with.  Passing a non-null
  // lpEnvironment replaces the whole block, hence copying the parent's first.
  //
  // This matters for the OpenVINS kernel specifically: its dynamic initialiser solves with Ceres, and with four
  // threads that solve was measured to land on either side of a stability boundary from run to run (31.21 m and
  // 3099.18 m from one configuration and one input).  Pinned to one thread it converged in three runs out of three.
  // The workspace's own full-drive script sets OMP_NUM_THREADS=1 for route_e_r2 for the same reason.
  std::vector<char> environment;
  {
    std::string block;
    LPCH current = GetEnvironmentStringsA();
    for (LPCH entry = current; entry != nullptr && *entry != '\0'; entry += std::strlen(entry) + 1) {
      const std::string text(entry);
      if (text.rfind("OMP_NUM_THREADS=", 0) == 0) continue;
      if (text.rfind("OPENCV_FOR_THREADS_NUM=", 0) == 0) continue;
      block += text;
      block += '\0';
    }
    if (current) FreeEnvironmentStringsA(current);
    block += "OMP_NUM_THREADS=1";
    block += '\0';
    block += "OPENCV_FOR_THREADS_NUM=1";
    block += '\0';
    block += '\0';
    environment.assign(block.begin(), block.end());
  }

  const BOOL created = CreateProcessA(nullptr, mutable_command.data(), nullptr, nullptr, TRUE,
                                      CREATE_NO_WINDOW, environment.data(),
                                      working_directory.empty() ? nullptr : working_directory.c_str(), &startup,
                                      &process);
  CloseHandle(write_end);  // the child holds its own copy; without this the read below never ends
  if (!created) {
    CloseHandle(read_end);
    error = "cannot start " + command_line + " (is ali_vins_run.exe in bin/?)";
    return false;
  }
  g_current_child_process = process.hProcess;

  char buffer[4096];
  DWORD read = 0;
  while (ReadFile(read_end, buffer, sizeof(buffer) - 1, &read, nullptr) && read > 0) {
    buffer[read] = '\0';
    sink(std::string(buffer, read));
  }
  CloseHandle(read_end);

  WaitForSingleObject(process.hProcess, INFINITE);
  DWORD code = 0;
  GetExitCodeProcess(process.hProcess, &code);
  exit_code = code;
  g_current_child_process = nullptr;
  CloseHandle(process.hThread);
  CloseHandle(process.hProcess);
  return true;
}

/// Runs the scorer if there is a reference to score against.  Written as a function with an explicit "why not"
/// string because "no score appeared" is otherwise indistinguishable from "scoring is broken".
bool ScoreTrajectory(const RunConf& conf, const std::string& trajectory_path, std::string& score_output,
                     std::string& reason) {
  if (conf.reference_path.empty()) {
    reason = "no reference in the configuration: scoring skipped";
    return false;
  }
  if (!FileExists(conf.reference_path)) {
    reason = "reference not found: " + conf.reference_path;
    return false;
  }
  const std::string scorer = JoinPath(ProjectRoot(), "tools\\gauge_free_score.py");
  if (!FileExists(scorer)) {
    reason = "scorer not found: " + scorer;
    return false;
  }

  std::ostringstream command;
  command << "python \"" << scorer << "\" --est \"" << trajectory_path << "\" --ref \""
          << conf.reference_path << "\" --time-scale " << conf.time_scale;

  std::string collected;
  unsigned long code = 0;
  std::string error;
  // python first, then the py launcher: on a machine where only one of them exists the other is simply absent,
  // and reporting the attempt is more useful than guessing up front.
  const char* interpreters[2] = {"python", "py"};
  for (int attempt = 0; attempt < 2; ++attempt) {
    std::string line = command.str();
    if (attempt == 1) line = "py " + line.substr(7);  // replace the leading "python " with "py "
    collected.clear();
    if (RunProcess(line, ProjectRoot(), [&collected](const std::string& chunk) { collected += chunk; }, code,
                   error)) {
      score_output = collected;
      if (code != 0) reason = "the scorer exited with code " + std::to_string(code) + ":\n" + collected;
      return code == 0;
    }
  }
  reason = "python not found: " + error;
  return false;
}

/// Appends to a file, used for the run's log so that a run can be read back after the window is closed.
void AppendToFile(const std::string& path, const std::string& text) {
  std::ofstream stream(path.c_str(), std::ios::app | std::ios::binary);
  if (stream) stream << text;
}

/// Turns the extractor's "frame,stamp" table into the image manifest the runner reads: "<stamp> <frame path>" per
/// line.  The paths are written absolute on purpose -- the runner only applies --base-dir to relative entries, and
/// an absolute path is correct whether or not the frames happen to live under the dataset root.
bool WriteManifestFromTimes(const std::string& times_csv, const std::string& frames_dir,
                            const std::string& manifest_out, std::size_t& count, std::string& error) {
  std::ifstream stream(times_csv.c_str());
  if (!stream) {
    error = "cannot open " + times_csv;
    return false;
  }
  std::ofstream output(manifest_out.c_str());
  if (!output) {
    error = "cannot write " + manifest_out;
    return false;
  }
  std::string line;
  count = 0;
  bool first_row = true;
  while (std::getline(stream, line)) {
    if (line.empty() || line[0] == '#') continue;
    const std::size_t comma = line.find(',');
    if (comma == std::string::npos) continue;
    const std::string index_text = Trim(line.substr(0, comma));
    if (first_row) {
      first_row = false;
      // The file starts with a "frame,stamp" header; skip it if the first field is not a number.
      if (!index_text.empty() && !std::isdigit(static_cast<unsigned char>(index_text[0]))) continue;
    }
    const double stamp = std::atof(line.substr(comma + 1).c_str());
    const int index = std::atoi(index_text.c_str());
    char name[64];
    std::snprintf(name, sizeof(name), "frame_%06d.png", index);
    output << stamp << ' ' << JoinPath(frames_dir, name) << '\n';
    ++count;
  }
  if (count == 0) {
    error = "no frames in " + times_csv;
    return false;
  }
  return true;
}

/// Makes sure the dataset a configuration names exists before the estimator is started.
///
/// A configuration may point straight at a ROS bag.  The C++ side does not parse the bag itself: the framing is
/// implemented in tools/bag_extract.py, which is pure Python and was derived from the actual bytes of this
/// workspace's bags.  Extracting a multi-gigabyte bag into thousands of PNGs takes minutes, so it happens once and
/// is then reused -- the check is for the extractor's own outputs, not for a marker file, so an interrupted
/// extraction simply resumes from the beginning rather than being mistaken for a finished one.
bool EnsureDataset(RunConf& conf, const std::function<void(const std::string&)>& log, std::string& error) {
  if (FileExists(conf.manifest_path)) {
    log("[app] dataset: manifest present, reusing it\n");
    return true;
  }
  if (conf.bag_path.empty()) {
    error = "no manifest at " + conf.manifest_path + ", and this configuration names no bag to extract";
    return false;
  }
  if (!FileExists(conf.bag_path)) {
    error = "no manifest, and the bag is missing too: " + conf.bag_path;
    return false;
  }
  if (conf.work_dir_path.empty()) {
    error = "a bag is named but work_dir is not set, so there is nowhere to extract it";
    return false;
  }
  CreateDirectoryA(conf.work_dir_path.c_str(), nullptr);

  const std::string times_csv = JoinPath(conf.work_dir_path, "times.csv");
  if (!FileExists(times_csv)) {
    const std::string extractor = JoinPath(ProjectRoot(), "tools\\bag_extract.py");
    if (!FileExists(extractor)) {
      error = "extractor not found: " + extractor;
      return false;
    }
    std::ostringstream command;
    command << "python \"" << extractor << "\" --bag \"" << conf.bag_path << "\" --kind fourseasons --out \""
            << conf.work_dir_path << "\"";
    std::ostringstream note;
    note << "[app] extracting " << BaseName(conf.bag_path) << " into " << conf.work_dir_path
         << " (minutes, for a bag this size)\n";
    log(note.str());

    unsigned long code = 0;
    std::string run_error;
    if (!RunProcess(command.str(), ProjectRoot(), log, code, run_error) || code != 0) {
      error = "bag extraction failed: " + (run_error.empty() ? "exit " + std::to_string(code) : run_error);
      return false;
    }
  } else {
    log("[app] dataset: times.csv present, reusing the existing extraction\n");
  }

  // The configuration's imu path may be written against the dataset root while the extractor wrote it into the work
  // directory.  Prefer whichever actually exists, and say so when they differ, rather than silently running without
  // inertial data.
  const std::string extracted_imu = JoinPath(conf.work_dir_path, "imu.csv");
  if (!FileExists(conf.imu_path) && FileExists(extracted_imu)) {
    std::ostringstream note;
    note << "[app] imu: '" << conf.imu_path << "' is missing; using the extracted " << extracted_imu << "\n";
    log(note.str());
    conf.imu_path = extracted_imu;
  }

  std::size_t count = 0;
  if (!WriteManifestFromTimes(times_csv, JoinPath(conf.work_dir_path, "frames"), conf.manifest_path, count, error)) {
    return false;
  }
  std::ostringstream note;
  note << "[app] wrote a manifest of " << count << " frame(s) to " << conf.manifest_path << "\n";
  log(note.str());
  return true;
}

/// Everything one run does, from a configuration to a scored solution.  Both front ends call this.
///
/// `on_prepared` is called once the output paths are known and before the estimator starts, which is what lets the
/// window begin following the trajectory file while it is still being written.  It is optional so that the headless
/// `--run` path does not have to care.
RunResult ExecuteRun(RunConf& conf, const std::function<void(const std::string&)>& log, bool score,
                     int frame_override = -1, const std::function<void(const RunResult&)>& on_prepared = {}) {
  RunResult result;

  const std::string stem = StripExtension(BaseName(conf.path)) + "_" + Timestamp();
  result.trajectory_path = JoinPath(conf.output_dir, stem + ".txt");
  result.log_path = JoinPath(conf.output_dir, stem + ".log");
  result.score_path = JoinPath(conf.output_dir, stem + ".score.txt");
  result.reference_path = conf.reference_path;

  if (on_prepared) on_prepared(result);

  CreateDirectoryA(conf.output_dir.c_str(), nullptr);

  // The override is applied before anything describes or uses the configuration, so the header, the command line and
  // the log all agree about what was actually run.
  if (frame_override >= 0) conf.frames = frame_override;

  // Every chunk is echoed to the caller and appended to the log file in one place, so the two cannot diverge.
  // The stream is opened once for the whole run: opening the file per chunk is measurably slow on Windows
  // (Defender rescans the growing log on every open), and it made a full drive look CPU-starved.
  std::ofstream log_stream(result.log_path.c_str(), std::ios::out | std::ios::binary);
  const std::function<void(const std::string&)> tee = [&](const std::string& chunk) {
    log(chunk);
    if (log_stream) {
      log_stream.write(chunk.data(), static_cast<std::streamsize>(chunk.size()));
      log_stream.flush();
    }
  };

  std::ostringstream header;
  header << "=== ali_vins run ===\n";
  header << Describe(conf);
  header << "trajectory  : " << result.trajectory_path << "\n";
  header << "log         : " << result.log_path << "\n";
  header << "command     : " << BuildRunnerCommandLine(conf, result.trajectory_path) << "\n\n";
  tee(header.str());

  // The dataset has to exist before the estimator is started; if the configuration points at a bag, this is where
  // the bag is turned into frames.
  std::string dataset_error;
  if (!EnsureDataset(conf, tee, dataset_error)) {
    std::ostringstream message;
    message << "\n[app] " << dataset_error << "\n";
    tee(message.str());
    return result;
  }

  const std::string command = BuildRunnerCommandLine(conf, result.trajectory_path);
  if (!RunProcess(command, ProjectRoot(), tee, result.exit_code, result.error)) {
    std::ostringstream message;
    message << "\n[app] " << result.error << "\n";
    tee(message.str());
    return result;
  }
  result.started = true;

  std::ostringstream finished;
  finished << "\n[app] estimator exited with code " << result.exit_code << "\n";
  tee(finished.str());

  // A run that produced no trajectory is not scored, and "no score" is not a good enough report: the
  // scorer would be handed a path that does not exist and answer with a Python traceback, which reads
  // like a bug in the scorer rather than a failed estimator.  This is the path a deliberately
  // unrunnable configuration takes (config/kitti_00.conf and config/gici_1_1.conf point `settings` at a
  // file that does not exist yet), so the message has to be the one that explains the situation.
  if (result.exit_code != 0) {
    std::ostringstream note;
    note << "\n[app] the estimator did not finish, so there is nothing to score.\n"
            "      Check the detail above: a settings file marked [MISSING], a run with no inertial\n"
            "      data, or a frame in the manifest that cannot be read.\n";
    tee(note.str());
    return result;
  }

  if (score) {
    std::string score_output;
    std::string reason;
    std::ostringstream note;
    if (ScoreTrajectory(conf, result.trajectory_path, score_output, reason)) {
      result.scored = true;
      result.score = score_output;
      note << "\n=== score ===\n" << score_output << "\n";
    } else {
      result.score = reason;
      note << "\n=== score ===\n" << reason << "\n";
    }
    tee(note.str());
    std::ofstream score_file(result.score_path.c_str());
    if (score_file) score_file << result.score;
  }
  return result;
}
#endif  // _WIN32

// --------------------------------------------------------------------------------------------- headless modes

int RunList() {
  std::string warning;
  const std::vector<RunConf> confs = DiscoverConfs(warning);
  std::printf("[app] project root : %s\n", ProjectRoot().c_str());
  std::printf("[app] runner       : %s%s\n", RunnerExecutable(confs.empty() ? "openvins" : confs[0].engine).c_str(),
              FileExists(RunnerExecutable(confs.empty() ? "openvins" : confs[0].engine)) ? "" : "  [MISSING]");
  std::printf("[app] configurations found: %d\n\n", static_cast<int>(confs.size()));
  if (!warning.empty()) std::printf("[app] note: %s\n\n", warning.c_str());
  for (std::size_t i = 0; i < confs.size(); ++i) {
    std::printf("--- %d ---\n%s\n", static_cast<int>(i), Describe(confs[i]).c_str());
    std::printf("command     : %s\n\n", BuildRunnerCommandLine(confs[i], "<trajectory>").c_str());
  }
  return confs.empty() ? 1 : 0;
}

int RunHeadless(const std::string& conf_argument, bool score, int frame_override) {
  std::string conf_path = conf_argument;
  if (!FileExists(conf_path)) {
    // Accept either a path or just the configuration's name, so "app --run 4seasons_fs_full" works.
    const std::string candidate = JoinPath(JoinPath(ProjectRoot(), "config"), conf_argument);
    if (FileExists(candidate)) {
      conf_path = candidate;
    } else if (FileExists(candidate + ".conf")) {
      conf_path = candidate + ".conf";
    }
  }
  RunConf conf;
  std::string error;
  if (!ParseConf(conf_path, conf, error)) {
    std::fprintf(stderr, "[app] %s\n", error.c_str());
    return 2;
  }

  const RunResult result = ExecuteRun(
      conf, [](const std::string& chunk) { std::fwrite(chunk.data(), 1, chunk.size(), stdout); std::fflush(stdout); },
      score, frame_override);

  if (!result.started) return 3;
  // Only claim a solution when there is one.  A failed estimator leaves no trajectory file, and
  // printing "solution written" over a path that does not exist sent a reader looking for it.
  if (result.exit_code == 0) {
    std::printf("\n[app] solution written:\n  %s\n  %s\n", result.trajectory_path.c_str(),
                result.log_path.c_str());
    if (result.scored) std::printf("  %s\n", result.score_path.c_str());
  } else {
    std::printf("\n[app] no solution: the estimator exited with code %d\n  log: %s\n",
                result.exit_code, result.log_path.c_str());
  }
  return static_cast<int>(result.exit_code);
}

/// The alignment the plot performs, without a display.  Same code path, printable result -- which is the only way
/// to check the geometry of an overlay that is otherwise only visible to a person looking at the window.
int RunCompare(const std::string& estimate_path, const std::string& reference_path) {
  if (reference_path.empty()) {
    std::fprintf(stderr, "[app] --compare needs --ref <file>\n");
    return 2;
  }
  Trajectory estimate;
  Trajectory reference;
  std::string error;
  if (!LoadTrajectory(estimate_path, estimate, error)) {
    std::fprintf(stderr, "[app] %s\n", error.c_str());
    return 2;
  }
  if (!LoadReference(reference_path, reference, error)) {
    std::fprintf(stderr, "[app] %s\n", error.c_str());
    return 2;
  }
  const Alignment alignment = AlignEstimateToReference(estimate, reference, 0.2);
  std::printf("[app] estimate : %s\n", estimate_path.c_str());
  std::printf("[app] reference: %s\n", reference_path.c_str());
  std::printf("%s", DescribeAlignment(alignment, estimate.size(), reference.size()).c_str());
  return alignment.ok ? 0 : 1;
}

#ifdef _WIN32

// --------------------------------------------------------------------------------------------- the window

const int kConfList = 1001;
const int kRunButton = 1002;
const int kStopButton = 1003;
const int kOpenButton = 1004;
const int kReloadButton = 1005;
const int kDetailsView = 1006;
const int kLogView = 1007;
const int kStatusText = 1008;
const int kPlotView = 1009;
const int kLoadButton = 1010;
const int kShowPlotButton = 1011;

// --------------------------------------------------------------------------------------------- the trajectory plot

/// What the plot draws.  It is a plain top-down view on purpose: this is a ground vehicle, and a plan view of the
/// estimate against the reference is the picture that shows whether the trajectory has the right shape and the
/// right size.  Equal scaling on both axes, so a trajectory that is too small looks too small.
struct PlotState {
  Trajectory reference;       // the part of the reference the estimate actually covers (what was scored)
  Trajectory reference_full;  // the whole reference file, for the "full" toggle
  Trajectory estimate;
  Alignment alignment;
  std::string caption;
  bool has_reference = false;
  bool has_estimate = false;
  bool hide_reference = false;   // double-click toggles it, for reading the estimate on its own
  bool reference_full_span = false;  // false = only the estimate's time window, which is the comparable part
};

PlotState g_plot;

/// Whether a run is in progress.  A plain flag rather than AppState::running because the plot's paint handler is
/// defined above AppState, and its empty-state message has to distinguish "still initialising" from "run
/// something" -- getting that wrong is exactly what made a healthy run look like a failure.
bool g_run_in_progress = false;

// Defined with the rest of the window logic, below; the plot's own double-click handler calls it.
void CycleReferenceMode();

/// The reference file usually covers far more time than the estimate: the full-drive ground truth is 95 s while a
/// 900-frame run is 30 s.  Drawing all of it makes the estimate a speck at the edge and the picture useless for
/// judging shape or size, so by default the reference is cut to the window the estimate actually spans -- which is
/// also the part the scorer compares against.
Trajectory RestrictToTimeRange(const Trajectory& in, double first, double last) {
  Trajectory out;
  for (std::size_t i = 0; i < in.size(); ++i) {
    if (in.time[i] < first || in.time[i] > last) continue;
    out.time.push_back(in.time[i]);
    out.x.push_back(in.x[i]);
    out.y.push_back(in.y[i]);
    out.z.push_back(in.z[i]);
  }
  return out;
}

/// The reference the plot should draw right now.
const Trajectory* ActiveReference() {
  if (!g_plot.has_reference || g_plot.hide_reference) return nullptr;
  return g_plot.reference_full_span ? &g_plot.reference_full : &g_plot.reference;
}

/// Downloads the estimate and the reference into the plot from files, and computes the display alignment.
bool PlotLoad(const std::string& estimate_path, const std::string& reference_path, std::string& message) {
  Trajectory estimate;
  std::string error;
  if (!LoadTrajectory(estimate_path, estimate, error)) {
    message = error;
    return false;
  }
  g_plot.estimate = estimate;
  g_plot.has_estimate = true;

  std::ostringstream caption;
  caption << BaseName(estimate_path) << "   " << estimate.size() << " poses";

  g_plot.reference = Trajectory();
  g_plot.reference_full = Trajectory();
  g_plot.has_reference = false;
  g_plot.alignment = Alignment();
  if (!reference_path.empty() && FileExists(reference_path)) {
    Trajectory reference;
    if (LoadReference(reference_path, reference, error)) {
      g_plot.reference_full = reference;
      g_plot.has_reference = true;
      // 0.2 s is the scorer's --max-gap default, so the pairs shown are the pairs that were scored.
      g_plot.alignment = AlignEstimateToReference(estimate, reference, 0.2);
      g_plot.reference = RestrictToTimeRange(reference, estimate.time.front(), estimate.time.back());
      caption << "   vs " << BaseName(reference_path);
      if (g_plot.alignment.ok) {
        caption << "   yaw " << g_plot.alignment.yaw_degrees << " deg"
                << "   matched " << g_plot.alignment.matched
                << " over " << g_plot.alignment.span_seconds << " s"
                << "   RMS 3D " << g_plot.alignment.rms_3d << " m";
      } else {
        caption << "   (not aligned: " << g_plot.alignment.reason << ")";
      }
    } else {
      caption << "   (reference unreadable: " << error << ")";
    }
  }
  caption << "\n";
  caption << "grey = reference, colour = estimate (blue start -> red end), scale 1 (no scale fitting), equal axes";
  if (g_plot.has_reference) {
    caption << "\nreference shown: " << (g_plot.reference_full_span ? "the whole file" : "only the estimate's time window")
            << " (" << g_plot.reference.size() << " of " << g_plot.reference_full.size() << " poses)";
  }

  g_plot.caption = caption.str();
  message = DescribeAlignment(g_plot.alignment, g_plot.estimate.size(),
                              g_plot.has_reference ? g_plot.reference.size() : 0);
  return true;
}

LRESULT CALLBACK PlotProc(HWND window, UINT message, WPARAM wparam, LPARAM lparam) {
  if (message == WM_ERASEBKGND) return 1;  // the paint double-buffers, so erasing first only flickers
  if (message != WM_PAINT) {
    if (message == WM_LBUTTONDBLCLK) {
      CycleReferenceMode();
      return 0;
    }
    return DefWindowProcA(window, message, wparam, lparam);
  }

  PAINTSTRUCT paint;
  HDC target = BeginPaint(window, &paint);
  RECT client;
  GetClientRect(window, &client);
  const int width = client.right - client.left;
  const int height = client.bottom - client.top;

  // Double buffer: without it the trajectory flickers on every resize.
  HDC canvas = CreateCompatibleDC(target);
  HBITMAP buffer = CreateCompatibleBitmap(target, width > 0 ? width : 1, height > 0 ? height : 1);
  HBITMAP previous = reinterpret_cast<HBITMAP>(SelectObject(canvas, buffer));

  HBRUSH background = CreateSolidBrush(RGB(252, 252, 252));
  FillRect(canvas, &client, background);
  DeleteObject(background);

  HFONT font = CreateFontA(-12, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
                           CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, "Consolas");
  HGDIOBJ old_font = SelectObject(canvas, font);
  SetBkMode(canvas, TRANSPARENT);

  if (!g_plot.has_estimate) {
    SetTextColor(canvas, RGB(120, 120, 120));
    // Which of the three situations this is matters, and they look identical from here: the estimator running and
    // still initialising is normal and needs a few seconds of data, whereas "run something" is an instruction.
    // Before this said only "No trajectory yet." in all three cases, which reads like a failure during a perfectly
    // healthy run -- and it was read that way.
    const char* hint = g_run_in_progress ? "Run in progress - no poses yet." : "No trajectory yet.";
    TextOutA(canvas, 12, 12, hint, static_cast<int>(std::strlen(hint)));
    const char* hint2 = "Run a configuration, or press \"Load .txt\" to open a trajectory file.";
    if (g_run_in_progress) {
      hint2 = "The trajectory is drawn here as it is produced, once a second. Nothing appears until the estimator "
              "has initialised, which needs a few seconds of data.";
    }
    TextOutA(canvas, 12, 30, hint2, static_cast<int>(std::strlen(hint2)));
    SelectObject(canvas, old_font);
    DeleteObject(font);
    BitBlt(target, 0, 0, width, height, canvas, 0, 0, SRCCOPY);
    SelectObject(canvas, previous);
    DeleteObject(buffer);
    DeleteDC(canvas);
    EndPaint(window, &paint);
    return 0;
  }

  // ---- world bounds over everything that will be drawn ------------------------------------------------
  double min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0;
  bool have_bounds = false;
  const std::vector<double>* xs[2] = {nullptr, nullptr};
  const std::vector<double>* ys[2] = {nullptr, nullptr};
  const Trajectory* active_reference = ActiveReference();
  if (active_reference && active_reference->size() >= 2) {
    xs[0] = &active_reference->x;
    ys[0] = &active_reference->y;
  }
  if (g_plot.alignment.ok) {
    xs[1] = &g_plot.alignment.aligned_x;
    ys[1] = &g_plot.alignment.aligned_y;
  } else {
    xs[1] = &g_plot.estimate.x;
    ys[1] = &g_plot.estimate.y;
  }
  for (int set = 0; set < 2; ++set) {
    if (!xs[set]) continue;
    for (std::size_t i = 0; i < xs[set]->size(); ++i) {
      const double x = (*xs[set])[i];
      const double y = (*ys[set])[i];
      if (!have_bounds) {
        min_x = max_x = x;
        min_y = max_y = y;
        have_bounds = true;
      } else {
        min_x = std::min(min_x, x); max_x = std::max(max_x, x);
        min_y = std::min(min_y, y); max_y = std::max(max_y, y);
      }
    }
  }
  if (!have_bounds) {
    SelectObject(canvas, old_font);
    DeleteObject(font);
    BitBlt(target, 0, 0, width, height, canvas, 0, 0, SRCCOPY);
    SelectObject(canvas, previous);
    DeleteObject(buffer);
    DeleteDC(canvas);
    EndPaint(window, &paint);
    return 0;
  }

  const double plot_left = 12.0, plot_top = 56.0, plot_right = width - 12.0, plot_bottom = height - 24.0;
  const double plot_width = std::max(1.0, plot_right - plot_left);
  const double plot_height = std::max(1.0, plot_bottom - plot_top);

  double span_x = std::max(1e-6, max_x - min_x);
  double span_y = std::max(1e-6, max_y - min_y);
  // One scale for both axes: a trajectory that is compressed in one direction must look compressed.
  const double scale = std::min(plot_width / span_x, plot_height / span_y);
  const double used_width = span_x * scale;
  const double used_height = span_y * scale;
  const double origin_x = plot_left + (plot_width - used_width) * 0.5;
  const double origin_y = plot_top + (plot_height - used_height) * 0.5;

  struct Projector {
    double origin_x, origin_y, scale, min_x, max_y;
    void operator()(double x, double y, int& sx, int& sy) const {
      sx = static_cast<int>(origin_x + (x - min_x) * scale + 0.5);
      sy = static_cast<int>(origin_y + (max_y - y) * scale + 0.5);  // y up, because north-up is what a map means
    }
  } project = {origin_x, origin_y, scale, min_x, max_y};

  // ---- grid ------------------------------------------------------------------------------------------
  {
    HPEN grid_pen = CreatePen(PS_SOLID, 1, RGB(232, 232, 232));
    HGDIOBJ old_pen = SelectObject(canvas, grid_pen);
    // A step that lands near 60 px on screen, rounded to a 1/2/5 x 10^n value so the labels are readable.
    const double raw_step = 60.0 / scale;
    const double magnitude = std::pow(10.0, std::floor(std::log10(raw_step)));
    double step = magnitude;
    if (raw_step / magnitude >= 5.0) step = 5.0 * magnitude;
    else if (raw_step / magnitude >= 2.0) step = 2.0 * magnitude;

    for (double x = std::ceil(min_x / step) * step; x <= max_x; x += step) {
      int sx = 0, sy = 0;
      int dummy = 0;
      project(x, min_y, sx, dummy);
      project(x, max_y, dummy, sy);
      MoveToEx(canvas, sx, static_cast<int>(plot_top), nullptr);
      LineTo(canvas, sx, static_cast<int>(plot_bottom));
    }
    for (double y = std::ceil(min_y / step) * step; y <= max_y; y += step) {
      int sx = 0, sy = 0;
      int dummy = 0;
      project(min_x, y, sx, dummy);
      project(max_x, y, dummy, sy);
      MoveToEx(canvas, static_cast<int>(plot_left), sy, nullptr);
      LineTo(canvas, static_cast<int>(plot_right), sy);
    }
    SelectObject(canvas, old_pen);
    DeleteObject(grid_pen);

    // Axis labels: metres, and the scale is stated so the reader can measure anything on the picture.
    SetTextColor(canvas, RGB(140, 140, 140));
    char label[128];
    std::snprintf(label, sizeof(label), "%.1f m per grid step  |  x %.1f..%.1f m, y %.1f..%.1f m", step, min_x,
                  max_x, min_y, max_y);
    TextOutA(canvas, static_cast<int>(plot_left), static_cast<int>(plot_bottom) + 4, label,
             static_cast<int>(std::strlen(label)));
  }

  // ---- the reference ---------------------------------------------------------------------------------
  if (active_reference && active_reference->size() >= 2) {
    HPEN reference_pen = CreatePen(PS_SOLID, 2, RGB(150, 150, 150));
    HGDIOBJ old_pen = SelectObject(canvas, reference_pen);
    std::vector<POINT> points(active_reference->size());
    for (std::size_t i = 0; i < active_reference->size(); ++i) {
      int sx = 0, sy = 0;
      project(active_reference->x[i], active_reference->y[i], sx, sy);
      points[i].x = sx;  // POINT members are LONG, the projector works in int
      points[i].y = sy;
    }
    Polyline(canvas, points.data(), static_cast<int>(points.size()));
    SelectObject(canvas, old_pen);
    DeleteObject(reference_pen);
  }

  // ---- the estimate, coloured by time so the direction of travel is visible ---------------------------
  {
    const std::vector<double>& px = g_plot.alignment.ok ? g_plot.alignment.aligned_x : g_plot.estimate.x;
    const std::vector<double>& py = g_plot.alignment.ok ? g_plot.alignment.aligned_y : g_plot.estimate.y;
    const std::size_t count = std::min(px.size(), py.size());

    const int kSteps = 32;
    HPEN pens[kSteps];
    for (int k = 0; k < kSteps; ++k) {
      const double u = static_cast<double>(k) / (kSteps - 1);
      const int red = static_cast<int>(40 + u * 200);
      const int green = static_cast<int>(90 - u * 60);
      const int blue = static_cast<int>(220 - u * 200);
      pens[k] = CreatePen(PS_SOLID, 2, RGB(red, green, blue));
    }
    HGDIOBJ old_pen = SelectObject(canvas, pens[0]);
    for (std::size_t i = 0; i + 1 < count; ++i) {
      const int bucket = count > 1 ? static_cast<int>(i * (kSteps - 1) / (count - 1)) : 0;
      SelectObject(canvas, pens[bucket]);
      int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
      project(px[i], py[i], x0, y0);
      project(px[i + 1], py[i + 1], x1, y1);
      MoveToEx(canvas, x0, y0, nullptr);
      LineTo(canvas, x1, y1);
    }
    SelectObject(canvas, old_pen);
    for (int k = 0; k < kSteps; ++k) DeleteObject(pens[k]);

    // Start and end, so the picture has a direction even in one colour.
    if (count > 0) {
      int sx = 0, sy = 0;
      HBRUSH start_brush = CreateSolidBrush(RGB(0, 150, 0));
      project(px.front(), py.front(), sx, sy);
      RECT dot = {sx - 4, sy - 4, sx + 4, sy + 4};
      FillRect(canvas, &dot, start_brush);
      DeleteObject(start_brush);

      HBRUSH end_brush = CreateSolidBrush(RGB(200, 0, 0));
      project(px.back(), py.back(), sx, sy);
      RECT end_dot = {sx - 4, sy - 4, sx + 4, sy + 4};
      FillRect(canvas, &end_dot, end_brush);
      DeleteObject(end_brush);
    }
  }

  // ---- caption ---------------------------------------------------------------------------------------
  SetTextColor(canvas, RGB(40, 40, 40));
  int line_y = 8;
  std::istringstream caption_stream(g_plot.caption);
  std::string caption_line;
  while (std::getline(caption_stream, caption_line)) {
    TextOutA(canvas, 12, line_y, caption_line.c_str(), static_cast<int>(caption_line.size()));
    line_y += 16;
  }

  SelectObject(canvas, old_font);
  DeleteObject(font);
  BitBlt(target, 0, 0, width, height, canvas, 0, 0, SRCCOPY);
  SelectObject(canvas, previous);
  DeleteObject(buffer);
  DeleteDC(canvas);
  EndPaint(window, &paint);
  return 0;
}

bool RegisterPlotClass(HINSTANCE instance) {
  WNDCLASSA plot_class;
  std::memset(&plot_class, 0, sizeof(plot_class));
  plot_class.lpfnWndProc = PlotProc;
  plot_class.hInstance = instance;
  plot_class.lpszClassName = "AliVinsPlotWindow";
  plot_class.hCursor = LoadCursor(nullptr, IDC_ARROW);
  plot_class.hbrBackground = nullptr;  // painted in WM_PAINT
  return RegisterClassA(&plot_class) != 0;
}

// A custom message whose lParam is a heap std::string*: the reader thread cannot touch controls directly without
// risking a deadlock with the UI thread, so it hands the text over and the window owns it from then on.
const UINT kAppendLog = WM_APP + 1;
const UINT kRunFinished = WM_APP + 2;
/// Sent by the worker as soon as the run's output path is known, so the window can start following the trajectory
/// file while the estimator is still writing it.
const UINT kRunStarted = WM_APP + 3;

/// How often the live plot looks at the trajectory file.  The file is flushed pose by pose, so this is purely a
/// redraw cadence.  One second, not faster: each redraw re-runs the alignment, which is O(poses x reference samples)
/// -- about 26 million comparisons at the end of a full drive -- and that work happens on the same machine as an
/// estimator that is sensitive to losing the CPU.  A redraw is skipped entirely when the file has not grown.
const UINT kLivePlotTimerMs = 1000;

struct AppState {
  HWND window = nullptr;
  HWND conf_list = nullptr;
  HWND details = nullptr;
  HWND log_view = nullptr;
  HWND status = nullptr;
  HWND run_button = nullptr;
  HWND stop_button = nullptr;
  HWND reference_button = nullptr;
  HWND plot = nullptr;
  std::vector<RunConf> confs;
  std::thread worker;
  bool running = false;
  HINSTANCE instance = nullptr;
  // The configuration the plot's reference came from, so a run can reload the plot against the right ground truth.
  std::string plotted_reference;
  RunResult last_result;
  // Live plotting: the trajectory the current run is writing, its size at the last redraw, and the ground truth to
  // draw it against.  Empty when no run is in progress.
  std::string live_trajectory_path;
  std::string live_reference_path;
  unsigned long long live_trajectory_bytes = 0;
  int live_pose_count = 0;
};

AppState* g_app = nullptr;

void SetStatus(const std::string& text) {
  if (g_app && g_app->status) SetWindowTextA(g_app->status, text.c_str());
  // --window has a console, and echoing here is what lets the live plot be verified from outside.
  if (g_echo_status_to_stdout) {
    std::printf("[app] %s\n", text.c_str());
    std::fflush(stdout);
  }
}

/// The log pane keeps at most this many characters.  An edit control's default capacity is 30000 characters, and it
/// does not complain when it is full -- it simply stops accepting text.  A full-drive run writes well over a
/// megabyte of estimator output, so the pane froze at exactly 30000 while the run carried on and sol/*.log kept
/// growing, which looks precisely like a hung run.  Measured with EM_REPLACESEL: 40000 characters appended to a
/// default control leave 30000 in it; the same append after EM_SETLIMITTEXT leaves all 40000.
const int kLogCharacterLimit = 4000000;
/// When the pane passes this, the oldest text is dropped.  Appending to an edit control costs time proportional to
/// the text already in it, so an unbounded pane would get slower for the whole run even with the limit raised.
const int kLogTrimAbove = 800000;
/// How much recent output to keep when trimming.
const std::size_t kLogKeepAfterTrim = 300000;

void AppendLogDirect(const std::string& text) {
  if (!g_app || !g_app->log_view) return;
  const int length = GetWindowTextLengthA(g_app->log_view);

  // Was the view already showing the end?  If the reader has scrolled up to look at something, a new line must not
  // yank them back down -- but when it is already at the end, it has to follow the new output or the pane looks
  // frozen even though it is filling.
  DWORD selection_start = 0, selection_end = 0;
  SendMessageA(g_app->log_view, EM_GETSEL, reinterpret_cast<WPARAM>(&selection_start),
               reinterpret_cast<LPARAM>(&selection_end));
  const bool was_at_end = static_cast<int>(selection_end) >= length;

  SendMessageA(g_app->log_view, EM_SETSEL, static_cast<WPARAM>(length), static_cast<LPARAM>(length));
  SendMessageA(g_app->log_view, EM_REPLACESEL, FALSE, reinterpret_cast<LPARAM>(text.c_str()));

  const int after = GetWindowTextLengthA(g_app->log_view);
  if (after > kLogTrimAbove) {
    std::string all(static_cast<std::size_t>(after), '\0');
    GetWindowTextA(g_app->log_view, &all[0], after + 1);
    std::size_t cut = all.size() > kLogKeepAfterTrim ? all.size() - kLogKeepAfterTrim : 0;
    const std::size_t newline = all.find('\n', cut);
    if (newline != std::string::npos) cut = newline + 1;
    all.erase(0, cut);
    all = "[... earlier output dropped from this pane; the complete log is written to sol/ ...]\r\n" + all;
    SetWindowTextA(g_app->log_view, all.c_str());
    const int end = GetWindowTextLengthA(g_app->log_view);
    SendMessageA(g_app->log_view, EM_SETSEL, static_cast<WPARAM>(end), static_cast<LPARAM>(end));
    SendMessageA(g_app->log_view, EM_SCROLLCARET, 0, 0);
    return;
  }
  if (was_at_end) SendMessageA(g_app->log_view, EM_SCROLLCARET, 0, 0);
}

void ReloadConfs() {
  std::string warning;
  g_app->confs = DiscoverConfs(warning);
  SendMessageA(g_app->conf_list, LB_RESETCONTENT, 0, 0);
  for (std::size_t i = 0; i < g_app->confs.size(); ++i) {
    SendMessageA(g_app->conf_list, LB_ADDSTRING, 0,
                 reinterpret_cast<LPARAM>(g_app->confs[i].name.c_str()));
  }
  if (!g_app->confs.empty()) {
    SendMessageA(g_app->conf_list, LB_SETCURSEL, 0, 0);
    SetWindowTextA(g_app->details, Describe(g_app->confs[0]).c_str());
  } else {
    SetWindowTextA(g_app->details,
                   "No configurations found.\n\nExpected a .conf file in the config/ directory next to bin/.");
  }
  if (!warning.empty()) AppendLogDirect("[app] " + warning + "\n");
  SetStatus(std::to_string(g_app->confs.size()) + " configuration(s); " + ProjectRoot());
}

RunConf* SelectedConf() {
  const int selection = static_cast<int>(SendMessageA(g_app->conf_list, LB_GETCURSEL, 0, 0));
  if (selection < 0 || selection >= static_cast<int>(g_app->confs.size())) return nullptr;
  return &g_app->confs[selection];
}

void StartRun() {
  if (g_app->running) return;
  RunConf* conf = SelectedConf();
  if (!conf) {
    MessageBoxA(g_app->window, "Select a configuration first.", "ali_vins", MB_OK | MB_ICONINFORMATION);
    return;
  }

  g_app->running = true;
  g_run_in_progress = true;
  EnableWindow(g_app->run_button, FALSE);
  EnableWindow(g_app->stop_button, TRUE);
  AppendLogDirect("\n");
  SetStatus("running " + conf->name + " ...");

  // The configuration is copied into the thread: the list can be reloaded while a run is in progress, which would
  // otherwise leave the worker holding a pointer into a vector that has been replaced.
  //
  // Deliberately not `const RunConf copy`: a by-value capture takes the type with its constness, so a const local
  // stays const inside the closure and cannot be passed to ExecuteRun, which fills in resolved paths.
  RunConf copy = *conf;
  if (g_app->worker.joinable()) g_app->worker.join();
  g_app->worker = std::thread([copy]() mutable {
    const RunResult result = ExecuteRun(
        copy,
        [](const std::string& chunk) {
          std::string* owned = new std::string(chunk);
          PostMessageA(g_app->window, kAppendLog, 0, reinterpret_cast<LPARAM>(owned));
        },
        true, g_startup_frame_override,
        // The estimator has not started yet at this point, but the path it will write is already decided -- the
        // driver opens the file before its first frame.  Telling the window now is what turns the plot live.
        [](const RunResult& prepared) {
          RunResult* owned = new RunResult(prepared);
          PostMessageA(g_app->window, kRunStarted, 0, reinterpret_cast<LPARAM>(owned));
        });
    RunResult* owned = new RunResult(result);
    PostMessageA(g_app->window, kRunFinished, 0, reinterpret_cast<LPARAM>(owned));
  });
}

void StopRun() {
  if (!g_app->running) return;
  const HANDLE child = g_current_child_process;
  if (child) {
    TerminateProcess(child, 1);
    AppendLogDirect("\n[app] stop requested: the estimator was terminated.\n");
  } else {
    AppendLogDirect("\n[app] stop requested, but the estimator has not started yet.\n");
  }
}

/// Opens a trajectory file and shows it.  The reference comes from the selected configuration, so a trajectory
/// loaded by hand is still put next to the same ground truth a run of that configuration would be scored against.
void LoadTrajectoryFromDialog() {
  char file_name[MAX_PATH];
  file_name[0] = '\0';

  RunConf* conf = SelectedConf();
  const std::string initial_directory = conf ? conf->output_dir : JoinPath(ProjectRoot(), "sol");

  OPENFILENAMEA dialog;
  std::memset(&dialog, 0, sizeof(dialog));
  dialog.lStructSize = sizeof(dialog);
  dialog.hwndOwner = g_app->window;
  dialog.lpstrFilter = "Trajectory (*.txt)\0*.txt\0All files (*.*)\0*.*\0";
  dialog.lpstrFile = file_name;
  dialog.nMaxFile = MAX_PATH;
  dialog.lpstrTitle = "Open a trajectory";
  dialog.lpstrInitialDir = initial_directory.c_str();
  dialog.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;
  if (!GetOpenFileNameA(&dialog)) return;

  const std::string reference = conf ? conf->reference_path : std::string();
  std::string message;
  if (PlotLoad(file_name, reference, message)) {
    g_app->plotted_reference = reference;
    AppendLogDirect("\n=== trajectory loaded ===\n" + message);
    InvalidateRect(g_app->plot, nullptr, TRUE);
    SetStatus("plotting " + BaseName(file_name));
  } else {
    MessageBoxA(g_app->window, message.c_str(), "ali_vins", MB_OK | MB_ICONWARNING);
  }
}

/// Cycles the reference through the three states that are useful: the estimate's time window (the comparable part),
/// the whole file (to see where the estimate sits in the drive), and hidden (to read the estimate alone).
void CycleReferenceMode() {
  if (!g_plot.has_reference) {
    SetStatus("there is no reference to show for this trajectory");
    return;
  }
  if (g_plot.hide_reference) {
    g_plot.hide_reference = false;
    g_plot.reference_full_span = false;
  } else if (!g_plot.reference_full_span) {
    g_plot.reference_full_span = true;
  } else {
    g_plot.reference_full_span = false;
    g_plot.hide_reference = true;
  }

  const char* label = g_plot.hide_reference ? "Ref: off"
                      : (g_plot.reference_full_span ? "Ref: full" : "Ref: window");
  if (g_app->reference_button) SetWindowTextA(g_app->reference_button, label);
  if (g_app->plot) InvalidateRect(g_app->plot, nullptr, TRUE);

  std::ostringstream status;
  status << "reference: "
         << (g_plot.hide_reference ? "hidden" : (g_plot.reference_full_span ? "the whole file" : "the estimate's time window"));
  if (g_plot.has_reference) {
    status << " (" << (g_plot.reference_full_span ? g_plot.reference_full.size() : g_plot.reference.size())
           << " poses)";
  }
  SetStatus(status.str());
}

LRESULT CALLBACK WindowProc(HWND window, UINT message, WPARAM wparam, LPARAM lparam) {  switch (message) {
    case WM_CREATE: {
      g_app->window = window;
      const int margin = 10;

      CreateWindowA("STATIC", "Configuration", WS_CHILD | WS_VISIBLE, margin, margin, 240, 18, window, nullptr,
                    g_app->instance, nullptr);
      g_app->conf_list = CreateWindowA("LISTBOX", nullptr,
                                       WS_CHILD | WS_VISIBLE | WS_BORDER | WS_VSCROLL | LBS_NOTIFY, margin,
                                       margin + 20, 240, 150, window,
                                       reinterpret_cast<HMENU>(kConfList), g_app->instance, nullptr);

      g_app->run_button = CreateWindowA("BUTTON", "Run", WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON, margin,
                                        margin + 180, 54, 28, window, reinterpret_cast<HMENU>(kRunButton),
                                        g_app->instance, nullptr);
      g_app->stop_button = CreateWindowA("BUTTON", "Stop", WS_CHILD | WS_VISIBLE, margin + 60, margin + 180, 54,
                                         28, window, reinterpret_cast<HMENU>(kStopButton), g_app->instance,
                                         nullptr);
      CreateWindowA("BUTTON", "Open sol", WS_CHILD | WS_VISIBLE, margin + 120, margin + 180, 54, 28, window,
                    reinterpret_cast<HMENU>(kOpenButton), g_app->instance, nullptr);
      CreateWindowA("BUTTON", "Reload", WS_CHILD | WS_VISIBLE, margin + 180, margin + 180, 54, 28, window,
                    reinterpret_cast<HMENU>(kReloadButton), g_app->instance, nullptr);
      CreateWindowA("BUTTON", "Load .txt", WS_CHILD | WS_VISIBLE, margin, margin + 212, 120, 26, window,
                    reinterpret_cast<HMENU>(kLoadButton), g_app->instance, nullptr);
      g_app->reference_button =
          CreateWindowA("BUTTON", "Ref: window", WS_CHILD | WS_VISIBLE, margin + 126, margin + 212, 108, 26,
                        window, reinterpret_cast<HMENU>(kShowPlotButton), g_app->instance, nullptr);

      const int right_x = margin + 260 + 12;
      CreateWindowA("STATIC", "Trajectory", WS_CHILD | WS_VISIBLE, right_x, margin, 400, 18, window, nullptr,
                    g_app->instance, nullptr);
      g_app->plot = CreateWindowA("AliVinsPlotWindow", nullptr,
                                  WS_CHILD | WS_VISIBLE | WS_BORDER | WS_TABSTOP, right_x, margin + 20, 600,
                                  300, window, reinterpret_cast<HMENU>(kPlotView), g_app->instance, nullptr);

      CreateWindowA("STATIC", "Estimator output", WS_CHILD | WS_VISIBLE, right_x, margin + 330, 400, 18, window,
                    nullptr, g_app->instance, nullptr);
      g_app->log_view = CreateWindowA("EDIT", nullptr,
                                      WS_CHILD | WS_VISIBLE | WS_BORDER | WS_VSCROLL | ES_MULTILINE | ES_READONLY |
                                          ES_AUTOVSCROLL,
                                      right_x, margin + 350, 600, 240, window, reinterpret_cast<HMENU>(kLogView),
                                      g_app->instance, nullptr);
      // Raise the edit control's default 30000-character capacity: see AppendLogDirect for why this is not optional.
      SendMessageA(g_app->log_view, EM_SETLIMITTEXT, static_cast<WPARAM>(kLogCharacterLimit), 0);

      CreateWindowA("STATIC", "Run configuration", WS_CHILD | WS_VISIBLE, margin, margin + 246, 400, 18, window,
                    nullptr, g_app->instance, nullptr);
      g_app->details = CreateWindowA("EDIT", nullptr,
                                     WS_CHILD | WS_VISIBLE | WS_BORDER | WS_VSCROLL | ES_MULTILINE | ES_READONLY,
                                     margin, margin + 266, 260, 320, window,
                                     reinterpret_cast<HMENU>(kDetailsView), g_app->instance, nullptr);

      g_app->status = CreateWindowA("STATIC", "", WS_CHILD | WS_VISIBLE | SS_LEFT, margin, margin + 508, 700, 18,
                                    window, reinterpret_cast<HMENU>(kStatusText), g_app->instance, nullptr);

      EnableWindow(g_app->stop_button, FALSE);
      ReloadConfs();
      return 0;
    }

    case WM_SIZE: {
      // Left column: the list and the details, fixed width.  Right side: the trajectory on top, the log below it,
      // both taking whatever is left -- so the window can be resized to read a long trajectory or a long log
      // without either one being clipped.
      const int width = LOWORD(lparam);
      const int height = HIWORD(lparam);
      const int margin = 10;
      const int left_width = 260;
      const int right_x = margin + left_width + 12;
      const int right_width = std::max(120, width - right_x - margin);
      const int plot_top = margin + 20;
      const int plot_height = std::max(120, (height - 2 * margin - 60) * 45 / 100);
      const int log_top = plot_top + plot_height + 30;

      if (g_app->conf_list) MoveWindow(g_app->conf_list, margin, margin + 20, left_width, 150, TRUE);
      if (g_app->details) {
        MoveWindow(g_app->details, margin, margin + 266, left_width, std::max(80, height - margin - 266 - 30), TRUE);
      }
      if (g_app->plot) MoveWindow(g_app->plot, right_x, plot_top, right_width, plot_height, TRUE);
      if (g_app->log_view) {
        MoveWindow(g_app->log_view, right_x, log_top, right_width, std::max(60, height - log_top - 34), TRUE);
      }
      if (g_app->status) MoveWindow(g_app->status, margin, height - 24, width - 2 * margin, 18, TRUE);
      return 0;
    }

    case WM_COMMAND: {
      const int id = LOWORD(wparam);
      const int notification = HIWORD(wparam);
      if (id == kRunButton) {
        StartRun();
      } else if (id == kStopButton) {
        StopRun();
      } else if (id == kReloadButton) {
        ReloadConfs();
      } else if (id == kLoadButton) {
        LoadTrajectoryFromDialog();
      } else if (id == kShowPlotButton) {
        CycleReferenceMode();
      } else if (id == kOpenButton) {
        RunConf* conf = SelectedConf();
        const std::string directory = conf ? conf->output_dir : JoinPath(ProjectRoot(), "sol");
        CreateDirectoryA(directory.c_str(), nullptr);
        ShellExecuteA(nullptr, "open", directory.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
      } else if (id == kConfList && notification == LBN_SELCHANGE) {
        RunConf* conf = SelectedConf();
        if (conf) SetWindowTextA(g_app->details, Describe(*conf).c_str());
      }
      return 0;
    }

    case kAppendLog: {
      std::string* text = reinterpret_cast<std::string*>(lparam);
      if (text) {
        AppendLogDirect(*text);
        delete text;
      }
      return 0;
    }

    // The run's output path is known and the estimator is about to write to it, so start following it.  The plot is
    // cleared first: showing the previous run's trajectory under the new run's log would be worse than showing
    // nothing, and "no poses yet" is the truth for the first few seconds anyway.
    case kRunStarted: {
      RunResult* prepared = reinterpret_cast<RunResult*>(lparam);
      if (prepared) {
        g_app->live_trajectory_path = prepared->trajectory_path;
        g_app->live_reference_path = prepared->reference_path;
        g_app->live_trajectory_bytes = 0;
        g_app->live_pose_count = 0;
        g_plot = PlotState();
        SetTimer(g_app->window, kLivePlotTimerMs, kLivePlotTimerMs, nullptr);
        delete prepared;
      }
      return 0;
    }

    // Live redraw.  Nothing is parsed unless the trajectory file has actually grown, so a run that is still
    // initialising -- when the file exists but is empty -- costs one file-size query per tick and nothing else.
    case WM_TIMER: {
      if (wparam != kLivePlotTimerMs || !g_app->running) return 0;
      const unsigned long long bytes = FileSizeOrZero(g_app->live_trajectory_path);
      if (bytes == 0 || bytes == g_app->live_trajectory_bytes) return 0;
      g_app->live_trajectory_bytes = bytes;

      std::string message;
      if (PlotLoad(g_app->live_trajectory_path, g_app->live_reference_path, message)) {
        g_app->plotted_reference = g_app->live_reference_path;
        g_app->live_pose_count = static_cast<int>(g_plot.estimate.size());
        InvalidateRect(g_app->plot, nullptr, TRUE);
        // The caption already carries the pose count and the RMS, so the status line only has to say the run is
        // still going -- it must not claim to be finished.
        SetStatus("running ... " + std::to_string(g_app->live_pose_count) + " pose(s) plotted so far");
      }
      return 0;
    }


    case kRunFinished: {
      RunResult* result = reinterpret_cast<RunResult*>(lparam);
      if (result) {
        g_app->last_result = *result;
        g_app->running = false;
        g_run_in_progress = false;
        // The run is over, so stop polling and forget the live path: the final plot below comes from the finished
        // file, which is the same file, now complete.
        KillTimer(g_app->window, kLivePlotTimerMs);
        g_app->live_trajectory_path.clear();
        g_app->live_reference_path.clear();
        EnableWindow(g_app->run_button, TRUE);
        EnableWindow(g_app->stop_button, FALSE);
        if (!result->started) {
          SetStatus("could not start the estimator - see the log");
        } else if (result->scored) {
          // The score line is what the run was for, so it goes in the title bar as well as the log.
          const std::size_t at = result->score.find("RMS 3D");
          const std::string summary =
              at == std::string::npos ? "run finished" : Trim(result->score.substr(at, 40));
          SetStatus("run finished (exit " + std::to_string(result->exit_code) + ") - " + summary);
        } else {
          SetStatus("run finished (exit " + std::to_string(result->exit_code) + ")");
        }

        // The trajectory this run produced goes straight onto the plot, against the ground truth the
        // configuration names -- so "did it work" is answerable from the window without opening anything.
        if (result->started && FileExists(result->trajectory_path)) {
          std::string message;
          if (PlotLoad(result->trajectory_path, result->reference_path, message)) {
            g_app->plotted_reference = result->reference_path;
            InvalidateRect(g_app->plot, nullptr, TRUE);
            AppendLogDirect("\n" + message);
          } else {
            AppendLogDirect("\n[app] could not plot the trajectory: " + message + "\n");
          }
        }
        delete result;
      }
      return 0;
    }

    case WM_DESTROY:
      PostQuitMessage(0);
      return 0;

    default:
      break;
  }
  return DefWindowProcA(window, message, wparam, lparam);
}

int RunWindowed(HINSTANCE instance) {
  // This is a console-subsystem binary so that --list and --run print into a terminal like any other tool.  When the
  // window is what was actually asked for, the console that a double-click from Explorer creates is hidden, so the
  // window is all the user sees.
  if (HWND console = GetConsoleWindow()) ShowWindow(console, SW_HIDE);

  AppState state;
  g_app = &state;
  state.instance = instance;

  INITCOMMONCONTROLSEX controls;
  controls.dwSize = sizeof(controls);
  controls.dwICC = ICC_STANDARD_CLASSES;
  InitCommonControlsEx(&controls);

  WNDCLASSA window_class;
  std::memset(&window_class, 0, sizeof(window_class));
  window_class.lpfnWndProc = WindowProc;
  window_class.hInstance = instance;
  window_class.lpszClassName = "AliVinsAppWindow";
  window_class.hCursor = LoadCursor(nullptr, IDC_ARROW);
  window_class.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_BTNFACE + 1);
  if (!RegisterClassA(&window_class)) {
    std::fprintf(stderr, "[app] RegisterClass failed\n");
    return 1;
  }
  if (!RegisterPlotClass(instance)) {
    std::fprintf(stderr, "[app] could not register the plot window class\n");
    return 1;
  }

  HWND window = CreateWindowA("AliVinsAppWindow", "ali-vins - ALI + SLAM VIO",
                              WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, 1240, 820, nullptr, nullptr,
                              instance, nullptr);
  if (!window) {
    std::fprintf(stderr, "[app] CreateWindow failed\n");
    return 1;
  }
  ShowWindow(window, SW_SHOW);

  // Open showing an existing result, when one was named on the command line.
  if (!g_startup_estimate.empty()) {
    std::string message;
    if (PlotLoad(g_startup_estimate, g_startup_reference, message)) {
      AppendLogDirect("=== trajectory loaded ===\n" + message + "\n");
      InvalidateRect(state.plot, nullptr, TRUE);
      SetStatus("plotting " + BaseName(g_startup_estimate));
    } else {
      AppendLogDirect("[app] could not load " + g_startup_estimate + ": " + message + "\n");
    }
  }

  // --run <conf> --window: select that configuration in the list and start it, so the window shows the run it was
  // asked for instead of waiting for a click.  The name is matched the way --run matches it, so a configuration's
  // file name and its displayed name both work, and so does a path.
  if (!g_startup_run_conf.empty()) {
    int index = -1;
    for (std::size_t i = 0; i < state.confs.size(); ++i) {
      const std::string stem = StripExtension(BaseName(state.confs[i].path));
      if (state.confs[i].name == g_startup_run_conf || stem == g_startup_run_conf ||
          state.confs[i].path == g_startup_run_conf) {
        index = static_cast<int>(i);
        break;
      }
    }
    if (index < 0) {
      const std::string complaint = "[app] no configuration named " + g_startup_run_conf + "\n";
      AppendLogDirect(complaint);
      std::fprintf(stderr, "%s", complaint.c_str());
      return 2;
    }
    SendMessageA(state.conf_list, LB_SETCURSEL, static_cast<WPARAM>(index), 0);
    SetWindowTextA(state.details, Describe(state.confs[static_cast<std::size_t>(index)]).c_str());
    StartRun();
  }

  MSG message;
  while (GetMessageA(&message, nullptr, 0, 0) > 0) {
    TranslateMessage(&message);
    DispatchMessageA(&message);
  }
  if (state.worker.joinable()) state.worker.join();
  g_app = nullptr;
  return 0;
}

#endif  // _WIN32

}  // namespace

int main(int argc, char** argv) {
  std::string run_conf;
  std::string compare_estimate;
  std::string compare_reference;
  bool list = false;
  bool score = true;
  bool run_in_window = false;
  int frame_override = -1;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const bool has_next = (i + 1) < argc;
    if (arg == "--list") list = true;
    else if (arg == "--run" && has_next) run_conf = argv[++i];
    // With --run: start that run in the window and watch the plot fill in, instead of running headless.
    else if (arg == "--window") run_in_window = true;
    else if (arg == "--no-score") score = false;
    // Overrides the configuration's frame count.  Present because the full-drive configuration legitimately says
    // "all 9487 frames" and a check of a few seconds should not have to edit it.
    else if (arg == "--frames" && has_next) frame_override = std::atoi(argv[++i]);
    // The alignment the plot uses, without a display: same code, printable result.
    else if (arg == "--compare" && has_next) compare_estimate = argv[++i];
    else if (arg == "--ref" && has_next) compare_reference = argv[++i];
    // Open the window with this trajectory already plotted.
    else if (arg == "--plot" && has_next) g_startup_estimate = argv[++i];
    else if (arg == "--help" || arg == "-h") {
      std::printf(
          "ali-vins -- GFTT + KLT frontend, MSCKF + SLAM, anisotropic landmark initialization\n\n"
          "  (no arguments)                  open the window: choose a configuration from config/ and run it\n"
          "  --plot <traj> [--ref <ref>]     open the window already showing this trajectory\n"
          "  --list                          list the configurations found in config/, with their paths checked\n"
          "  --run <conf>                    run one configuration without a window\n"
          "  --window                        with --run, show that run in the window, plotting as it goes\n"
          "  --frames N                      with --run, override the configuration's frame count\n"
          "  --no-score                      with --run, do not score the result\n"
          "  --compare <traj> --ref <ref>    align a trajectory to the reference and report the alignment\n\n"
          "A configuration may be given as a path or by name; the name is looked up in config/.\n"
          "Solutions are written to the configuration's output directory (sol/ by default).\n"
          "--compare uses the same alignment as the window's plot: yaw + translation, scale fixed at 1.\n"
          "In the window, double-click the plot or press \"Ref: ...\" to switch the reference between the\n"
          "estimate's time window, the whole file, and hidden.\n");
      return 0;
    } else {
      std::fprintf(stderr, "unknown argument %s (try --help)\n", arg.c_str());
      return 2;
    }
  }

  if (list) return RunList();
  if (!compare_estimate.empty()) return RunCompare(compare_estimate, compare_reference);
  // --run is headless by default, because that is what a script wants.  --window makes it start the same run inside
  // the window instead: the only way to watch the live plot without clicking, and the only way to exercise that
  // path without a person at the keyboard.
  if (!run_conf.empty() && !run_in_window) return RunHeadless(run_conf, score, frame_override);
  // The window opens showing this trajectory when one was named with --plot.
  g_startup_reference = compare_reference;
  g_startup_run_conf = run_conf;
  g_startup_frame_override = frame_override;
  g_echo_status_to_stdout = run_in_window;
#ifdef _WIN32
  return RunWindowed(GetModuleHandleA(nullptr));
#else
  std::fprintf(stderr, "no window in this build; use --list or --run <conf>\n");
  return 2;
#endif
}
