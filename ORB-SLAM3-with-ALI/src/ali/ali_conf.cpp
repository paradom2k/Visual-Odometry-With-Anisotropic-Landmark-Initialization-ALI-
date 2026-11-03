#include "ali_conf.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace ali {
namespace {

std::string trim(const std::string& text) {
  const char* spaces = " \t\r\n";
  const std::size_t first = text.find_first_not_of(spaces);
  if (first == std::string::npos) {
    return std::string();
  }
  return text.substr(first, text.find_last_not_of(spaces) - first + 1);
}

std::string lower(std::string text) {
  for (char& c : text) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return text;
}

std::string normaliseKey(const std::string& key) {
  std::string out;
  out.reserve(key.size());
  for (char c : key) {
    out.push_back((c == '-' || c == ' ') ? '_'
                                         : static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  return out;
}

/// Strip a `#` comment.  Only a `#` at the start of the line or after whitespace counts, so a path or a
/// value containing one is not truncated.
std::string stripComment(const std::string& line) {
  for (std::size_t i = 0; i < line.size(); ++i) {
    if (line[i] == '#' && (i == 0 || line[i - 1] == ' ' || line[i - 1] == '\t')) {
      return line.substr(0, i);
    }
  }
  return line;
}

bool parseDouble(const std::string& value, double* out) {
  if (value.empty()) {
    return false;
  }
  char* end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str()) {
    return false;
  }
  while (end != nullptr && *end != '\0') {
    if (*end != ' ' && *end != '\t') {
      return false;
    }
    ++end;
  }
  *out = parsed;
  return true;
}

bool parseBool(const std::string& value, bool* out) {
  const std::string v = lower(trim(value));
  if (v == "on" || v == "true" || v == "yes" || v == "1") {
    *out = true;
    return true;
  }
  if (v == "off" || v == "false" || v == "no" || v == "0") {
    *out = false;
    return true;
  }
  return false;
}

/// A whitespace- or comma-separated list of numbers.
bool parseList(const std::string& value, std::vector<double>* out) {
  std::string text = value;
  for (char& c : text) {
    if (c == ',') {
      c = ' ';
    }
  }
  std::istringstream stream(text);
  double v = 0.0;
  out->clear();
  while (stream >> v) {
    out->push_back(v);
  }
  return !out->empty();
}

/// The GNSS-factor keys, parsed outside `loadConf`'s else-if ladder for the same MSVC C1061 reason.
bool parseGnssKey(const std::string& key, const std::string& value, Conf* out) {
  if (key == "gnss_dd") {
    bool b = false;
    if (parseBool(value, &b)) out->gnss_dd = b; else out->warnings.push_back(key + " = " + value + " is not on/off");
    return true;
  }
  if (key == "gnss_code" || key == "gnss_phase") {
    bool b = false;
    if (parseBool(value, &b)) {
      if (key == "gnss_code") out->gnss_code = b; else out->gnss_phase = b;
    } else {
      out->warnings.push_back(key + " = " + value + " is not on/off");
    }
    return true;
  }
  if (key == "gnss_rover" || key == "gnss_base" || key == "gnss_nav" || key == "gnss_truth") {
    if (key == "gnss_rover") out->gnss_rover = value;
    else if (key == "gnss_base") out->gnss_base = value;
    else if (key == "gnss_nav") out->gnss_nav = value;
    else out->gnss_truth = value;
    return true;
  }
  if (key == "gnss_base_ecef" || key == "gnss_lever") {
    std::vector<double> v;
    if (parseList(value, &v) && v.size() == 3) {
      if (key == "gnss_base_ecef") {
        for (int i = 0; i < 3; ++i) out->gnss_base_ecef[i] = v[static_cast<std::size_t>(i)];
      } else {
        for (int i = 0; i < 3; ++i) out->gnss_lever[i] = v[static_cast<std::size_t>(i)];
      }
    } else {
      out->warnings.push_back(key + " wants three numbers");
    }
    return true;
  }
  if (key == "gnss_code_sigma" || key == "gnss_phase_sigma") {
    double v = 0.0;
    if (parseDouble(value, &v) && v > 0.0) {
      if (key == "gnss_code_sigma") out->gnss_code_sigma = v; else out->gnss_phase_sigma = v;
    } else {
      out->warnings.push_back(key + " = " + value + " wants a positive number");
    }
    return true;
  }
  if (key == "gnss_weight") {
    double v = 0.0;
    if (parseDouble(value, &v) && v >= 0.1 && v <= 100.0) {
      out->gnss_weight = v;
    } else {
      out->warnings.push_back(key + " = " + value + " expects a value in [0.1, 100]");
    }
    return true;
  }
  if (key == "gnss_position_sigma") {
    double v = 0.0;
    if (parseDouble(value, &v) && v >= 0.0 && v <= 10.0) {
      out->gnss_position_sigma = v;
    } else {
      out->warnings.push_back(key + " = " + value + " expects a value in [0, 10] m");
    }
    return true;
  }
  if (key == "gnss_yaw_sigma_deg") {
    double v = 0.0;
    if (parseDouble(value, &v) && v >= 0.0 && v <= 30.0) {
      out->gnss_yaw_sigma_deg = v;
    } else {
      out->warnings.push_back(key + " = " + value + " expects a value in [0, 30] deg");
    }
    return true;
  }
  if (key == "gnss_code_huber") {
    double v = 0.0;
    if (parseDouble(value, &v) && v >= 1.0 && v <= 100.0) {
      out->gnss_code_huber = v;
    } else {
      out->warnings.push_back(key + " = " + value + " expects a value in [1, 100]");
    }
    return true;
  }
  if (key == "gnss_phase_slip" || key == "gnss_code_outlier" ||
      key == "gnss_ambiguity_prior") {
    double v = 0.0;
    if (parseDouble(value, &v) && v > 0.0) {
      if (key == "gnss_phase_slip") out->gnss_phase_slip = v;
      else if (key == "gnss_code_outlier") out->gnss_code_outlier = v;
      else out->gnss_ambiguity_prior = v;
    } else {
      out->warnings.push_back(key + " = " + value + " wants a positive number");
    }
    return true;
  }
  if (key == "gnss_float_ambiguity") {
    bool b = false;
    if (parseBool(value, &b)) out->gnss_float_ambiguity = b;
    else out->warnings.push_back(key + " = " + value + " is not on/off");
    return true;
  }
  if (key == "gnss_start_time" || key == "gnss_end_time") {
    double v = 0.0;
    if (parseDouble(value, &v) && v >= 0.0) {
      if (key == "gnss_start_time") out->gnss_start_time = v; else out->gnss_end_time = v;
    } else {
      out->warnings.push_back(key + " = " + value + " wants a non-negative GPS second");
    }
    return true;
  }
  if (key == "gnss_window") {
    double v = 0.0;
    if (parseDouble(value, &v) && v >= 2.0) {
      out->gnss_window = static_cast<int>(v);
    } else {
      out->warnings.push_back(key + " = " + value + " wants an integer >= 2");
    }
    return true;
  }
  if (key == "gnss_fix_biases") {
    bool b = false;
    if (parseBool(value, &b)) out->gnss_fix_biases = b;
    else out->warnings.push_back(key + " = " + value + " is not on/off");
    return true;
  }
  return false;
}

/// LVI visual-factor keys, likewise outside the main ladder.
bool parseLviKey(const std::string& key, const std::string& value, Conf* out) {
  if (key != "lvi_visual_trans" && key != "lvi_visual_rot_deg" && key != "lvi_visual_huber") {
    return false;
  }
  double v = 0.0;
  if (!parseDouble(value, &v) || v <= 0.0) {
    out->warnings.push_back(key + " = " + value + " wants a positive number");
    return true;
  }
  if (key == "lvi_visual_trans") {
    out->lvi_visual_trans_sigma = v;
  } else if (key == "lvi_visual_rot_deg") {
    out->lvi_visual_rot_deg = v;
  } else {
    out->lvi_visual_huber = v;
  }
  return true;
}

/// The LiDAR-window keys, parsed outside `loadConf`'s else-if ladder.  That ladder is already close to
/// MSVC's C1061 block-nesting limit; adding these branches inline pushed it over ("block nesting too
/// deep" at the debug_every branch).  Returning true means the key belongs to this group, whether or not
/// its value parsed; the warning is recorded here.
bool parseLidarWindowKey(const std::string& key, const std::string& value, Conf* out) {
  if (key == "lidar_window") {
    double v = 0;
    if (parseDouble(value, &v) && v >= 0) {
      out->lidar_window = static_cast<int>(v);
    } else {
      out->warnings.push_back(key + " = " + value + " wants a non-negative integer");
    }
    return true;
  }
  if (key == "lidar_window_output") {
    const std::string v = lower(trim(value));
    if (v == "window" || v == "icp") {
      out->lidar_window_output = v;
    } else {
      out->warnings.push_back(key + " = " + value + " is not `window` or `icp`");
    }
    return true;
  }
  if (key == "lvi_visual_keyframe") {
    double v = 0.0;
    if (parseDouble(value, &v) && v >= 1.0 && v <= 20.0) {
      out->lvi_visual_keyframe = static_cast<int>(v);
    } else {
      out->warnings.push_back(key + " = " + value + " wants an integer in [1, 20]");
    }
    return true;
  }
  if (key == "lidar_window_solve_interval") {
    double v = 0.0;
    if (parseDouble(value, &v) && v >= 1.0 && v <= 50.0) {
      out->lidar_window_solve_interval = static_cast<int>(v);
    } else {
      out->warnings.push_back(key + " = " + value + " wants an integer in [1, 50]");
    }
    return true;
  }
  if (key == "lidar_window_marginalize" || key == "lidar_window_anchor" ||
      key == "lidar_window_anchor_all" || key == "lidar_window_fix_biases" ||
      key == "lidar_window_persistent") {
    bool b = false;
    if (parseBool(value, &b)) {
      if (key == "lidar_window_marginalize") {
        out->lidar_window_marginalize = b;
      } else if (key == "lidar_window_anchor") {
        out->lidar_window_anchor = b;
      } else if (key == "lidar_window_anchor_all") {
        out->lidar_window_anchor_all = b;
      } else if (key == "lidar_window_persistent") {
        out->lidar_window_persistent = b;
      } else {
        out->lidar_window_fix_biases = b;
      }
    } else {
      out->warnings.push_back(key + " = " + value + " is not on/off");
    }
    return true;
  }
  if (key == "lidar_window_prior_trans" || key == "lidar_window_prior_rot_deg" ||
      key == "lidar_window_relative_trans" || key == "lidar_window_relative_rot_deg" ||
      key == "lidar_window_point_sigma") {
    double v = 0;
    if (parseDouble(value, &v) && v > 0) {
      if (key == "lidar_window_prior_trans") {
        out->lidar_window_prior_trans = v;
      } else if (key == "lidar_window_prior_rot_deg") {
        out->lidar_window_prior_rot_deg = v;
      } else if (key == "lidar_window_relative_trans") {
        out->lidar_window_relative_trans = v;
      } else if (key == "lidar_window_point_sigma") {
        out->lidar_window_point_sigma = v;
      } else {
        out->lidar_window_relative_rot_deg = v;
      }
    } else {
      out->warnings.push_back(key + " = " + value + " wants a positive number");
    }
    return true;
  }
  if (key == "display_map") {
    out->display_map = value;
    return true;
  }
  if (key == "cloud_dir") {
    out->cloud_dir = value;
    return true;
  }
  if (key == "cloud_voxel") {
    double v = 0.0;
    if (parseDouble(value, &v) && v > 0.0) {
      out->cloud_voxel = static_cast<float>(v);
    } else {
      out->warnings.push_back(key + " = " + value + " wants a positive metre value");
    }
    return true;
  }
  if (key == "cloud_max_points") {
    double v = 0.0;
    if (parseDouble(value, &v) && v >= 1000.0) {
      out->cloud_max_points = static_cast<int>(v);
    } else {
      out->warnings.push_back(key + " = " + value + " wants an integer >= 1000");
    }
    return true;
  }
  if (key == "lidar_window_points") {
    double v = 0;
    if (parseDouble(value, &v) && v >= 0) {
      out->lidar_window_points = static_cast<int>(v);
    } else {
      out->warnings.push_back(key + " = " + value + " wants a non-negative integer");
    }
    return true;
  }
  return false;
}

}  // namespace

const char* modeName(Mode mode) {
  switch (mode) {
    case Mode::kMono: return "mono";
    case Mode::kMonoInertial: return "mono-inertial";
    case Mode::kStereo: return "stereo";
    case Mode::kStereoInertial: return "stereo-inertial";
    case Mode::kLidar: return "lidar";
    case Mode::kLidarInertial: return "lidar-inertial";
    case Mode::kEpipolarStereo: return "epipolar";
    case Mode::kEpipolarStereoInertial: return "epipolar-inertial";
    case Mode::kEpipolarVio: return "epipolar-vio";
    case Mode::kGnss: return "gnss";
    case Mode::kGnssInertial: return "gnss-inertial";
    case Mode::kGnssInertialEpipolar: return "gnss-inertial-epipolar";
    case Mode::kGnssInertialLidar: return "gnss-inertial-lidar";
    case Mode::kGnssInertialLidarEpipolar: return "gnss-inertial-lidar-epipolar";
    case Mode::kLvi: return "lvi";
  }
  return "?";
}

std::string Conf::resolvedSolution(const std::string& repo_root) const {
  if (!solution.empty()) {
    return solution;
  }
  // The acceptance criterion: the trajectory is stored beside the configuration that produced it and under
  // the same stem, so `conf/fs_stereo.conf` writes `sol/fs_stereo.txt`.
  std::string stem = path;
  const std::size_t slash = stem.find_last_of("/\\");
  if (slash != std::string::npos) {
    stem = stem.substr(slash + 1);
  }
  const std::size_t dot = stem.find_last_of('.');
  if (dot != std::string::npos) {
    stem = stem.substr(0, dot);
  }
  const std::string dir = repo_root.empty() ? std::string(".") : repo_root + "/sol";
  return dir + "/" + stem + ".txt";
}

std::string repoRootFrom(const std::string& executable_path) {
  // The build puts the binaries at <root>/build/src/ali_run.exe and the staging copies them to
  // <root>/bin/ali_run.exe.  Walk up until a directory that holds `conf` and `src` is found; stop after
  // four levels so a stray executable outside the tree does not walk to the drive root.
  std::string dir = executable_path;
  for (int i = 0; i < 4; ++i) {
    const std::size_t slash = dir.find_last_of("/\\");
    if (slash == std::string::npos) {
      break;
    }
    dir = dir.substr(0, slash);
    std::ifstream conf_test(dir + "/conf/ali_probe");
    std::ifstream src_test(dir + "/src/ali/ali_conf.h");
    if (src_test.good()) {
      return dir;
    }
  }
  return std::string();
}

bool loadConf(const std::string& path, Conf* out, std::string* error) {
  std::ifstream in(path);
  if (!in) {
    if (error != nullptr) {
      *error = "cannot open " + path;
    }
    return false;
  }
  // The UI reuses one Conf object across configuration selections.  Without this reset every field the
  // previous file did not mention survives: after a GNSS run, an epipolar config inherited gnss_dd,
  // gnss_start_time and the GNSS paths, so the epipolar runner tried to prepare GREAT rover epochs
  // against a 4Seasons image window ("no rover epoch in the requested window") and paid the RINEX/DD
  // preparation cost.  Each file is a complete configuration, so start from the defaults.
  *out = Conf();
  out->path = path;
  out->solution.clear();

  std::size_t line_no = 0;
  std::string line;
  while (std::getline(in, line)) {
    ++line_no;
    const std::string text = trim(stripComment(line));
    if (text.empty()) {
      continue;
    }
    const std::size_t eq = text.find('=');
    if (eq == std::string::npos) {
      out->warnings.push_back("line " + std::to_string(line_no) + ": not `key = value`: " + text);
      continue;
    }
    const std::string key = normaliseKey(trim(text.substr(0, eq)));
    const std::string value = trim(text.substr(eq + 1));
    if (key == "epipolar_pointline_mask") {
      bool b = false;
      if (parseBool(value, &b)) out->epipolar_pointline_mask = b;
      else out->warnings.push_back(key + " = " + value + " is not on/off");
      continue;
    }
    if (key == "display_map") {
      out->display_map = value;
      continue;
    }
    bool recognised = true;

    // Dense-cloud output keys are handled by the same helper; they apply to every mode (ignored where
    // there is no LiDAR) so the key must not fall through to the mode-specific ladder.
    if (key.rfind("cloud_", 0) == 0) {
      if (!parseLidarWindowKey(key, value, out)) {
        out->unknown_keys.push_back(key);
        out->warnings.push_back("line " + std::to_string(line_no) + ": unrecognised key `" + key + "`");
      }
      continue;
    }

    // Plan 1 local-BA switch.  It is not a `lidar_window*` key and must not fall through to the
    // camera-mode ladder.
    if (key == "gnss_lidar_4s_frames") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 10.0 && v <= 200.0) {
        out->gnss_lidar_4s_frames = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " = " + value + " wants an integer in [10, 200]");
      }
      continue;
    }
    if (key == "gnss_lidar_4s_mapper_points") {
      bool b = false;
      if (parseBool(value, &b)) out->gnss_lidar_4s_mapper_points = b;
      else out->warnings.push_back(key + " = " + value + " is not on/off");
      continue;
    }
    if (key == "gnss_lidar_4s_visual_matches") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 20.0 && v <= 1600.0) {
        out->gnss_lidar_4s_visual_matches = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " = " + value + " wants an integer in [20, 1600]");
      }
      continue;
    }
    if (key == "gnss_lidar_4s_visual_keyframe") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 1.0 && v <= 20.0) {
        out->gnss_lidar_4s_visual_keyframe = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " = " + value + " wants an integer in [1, 20]");
      }
      continue;
    }
    if (key == "gnss_lidar_4s_solver_iterations") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 1.0 && v <= 20.0) {
        out->gnss_lidar_4s_solver_iterations = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " = " + value + " wants an integer in [1, 20]");
      }
      continue;
    }
    if (key == "gnss_lidar_4s_scan_keyframe") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 1.0 && v <= 20.0) {
        out->gnss_lidar_4s_scan_keyframe = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " = " + value + " wants an integer in [1, 20]");
      }
      continue;
    }
    if (key == "gnss_lidar_4s_gauss_newton") {
      bool b = false;
      if (parseBool(value, &b)) out->gnss_lidar_4s_gauss_newton = b;
      else out->warnings.push_back(key + " = " + value + " is not on/off");
      continue;
    }
    if (key == "gnss_visual_4s_visual_keyframe") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 1.0 && v <= 20.0) {
        out->gnss_visual_4s_visual_keyframe = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " = " + value + " wants an integer in [1, 20]");
      }
      continue;
    }
    if (key == "gnss_visual_4s_gauss_newton") {
      bool b = false;
      if (parseBool(value, &b)) out->gnss_visual_4s_gauss_newton = b;
      else out->warnings.push_back(key + " = " + value + " is not on/off");
      continue;
    }
    if (key == "gnss_visual_4s_solver_iterations") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 1.0 && v <= 20.0) {
        out->gnss_visual_4s_solver_iterations = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " = " + value + " wants an integer in [1, 20]");
      }
      continue;
    }
    if (key == "gnss_visual_4s_window") {
      bool b = false;
      if (parseBool(value, &b)) out->gnss_visual_4s_window = b;
      else out->warnings.push_back(key + " = " + value + " is not on/off");
      continue;
    }
    if (key == "gnss_visual_4s_frames") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 10.0 && v <= 200.0) {
        out->gnss_visual_4s_frames = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " = " + value + " wants an integer in [10, 200]");
      }
      continue;
    }
    if (key == "gnss_visual_4s_matches") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 20.0 && v <= 1600.0) {
        out->gnss_visual_4s_matches = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " = " + value + " wants an integer in [20, 1600]");
      }
      continue;
    }
    if (key == "gnss_gi_marginalize") {
      bool b = false;
      if (parseBool(value, &b)) out->gnss_gi_marginalize = b;
      else out->warnings.push_back(key + " = " + value + " is not on/off");
      continue;
    }
    if (key == "gnss_gi_frames") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 2.0 && v <= 32.0) {
        out->gnss_gi_frames = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " = " + value + " wants an integer in [2, 32]");
      }
      continue;
    }
    if (key == "imu_gyro_scale" || key == "imu_accel_scale") {
      double v = 0.0;
      if (parseDouble(value, &v) && v > 0.5 && v < 1.5) {
        if (key == "imu_gyro_scale") out->imu_gyro_scale = v;
        else out->imu_accel_scale = v;
      } else {
        out->warnings.push_back(key + " = " + value + " wants a value in [0.5, 1.5]");
      }
      continue;
    }
    if (key == "imu_probe") {
      bool b = false;
      if (parseBool(value, &b)) out->imu_probe = b;
      else out->warnings.push_back(key + " = " + value + " is not on/off");
      continue;
    }
    if (key == "gnss_lidar_4s_window") {
      bool b = false;
      if (parseBool(value, &b)) out->gnss_lidar_4s_window = b;
      else out->warnings.push_back(key + " = " + value + " is not on/off");
      continue;
    }
    if (key == "lidar_local_ba" || key == "visual_local_ba" || key == "lidar_local_imu") {
      bool b = false;
      if (parseBool(value, &b)) {
        if (key == "lidar_local_ba") out->lidar_local_ba = b;
        else if (key == "visual_local_ba") out->visual_local_ba = b;
        else out->lidar_local_imu = b;
      } else {
        out->warnings.push_back(key + " = " + value + " is not on/off");
      }
      continue;
    }

    // The LiDAR-window group is parsed here, before the main ladder; see parseLidarWindowKey for why it is
    // not eight more else-ifs (MSVC C1061).
    if (key.rfind("lidar_window", 0) == 0) {
      if (!parseLidarWindowKey(key, value, out)) {
        out->unknown_keys.push_back(key);
        out->warnings.push_back("line " + std::to_string(line_no) + ": unrecognised key `" + key + "`");
      }
      continue;
    }
    if (key.rfind("gnss_", 0) == 0) {
      if (!parseGnssKey(key, value, out)) {
        out->unknown_keys.push_back(key);
        out->warnings.push_back("line " + std::to_string(line_no) + ": unrecognised key `" + key + "`");
      }
      continue;
    }
    if (key.rfind("lvi_", 0) == 0) {
      if (!parseLviKey(key, value, out)) {
        out->unknown_keys.push_back(key);
        out->warnings.push_back("line " + std::to_string(line_no) + ": unrecognised key `" + key + "`");
      }
      continue;
    }

    // ALI.  Handled here rather than in the key chain below: that chain is already at MSVC's block-nesting
    // limit (C1061) and one more `else if` pushes it over.
    if (key == "ali" || key == "ali_on" || key == "anisotropic_landmark_init") {
      if (!parseBool(value, &out->ali)) {
        out->warnings.push_back("line " + std::to_string(line_no) + ": `ali` wants on or off");
      }
      continue;
    }

    // ---- identity -----------------------------------------------------------------------------
    if (key == "name") {
      out->name = value;
    } else if (key == "description") {
      out->description = value;
    } else if (key == "mode") {
      const std::string v = lower(value);
      if (v == "mono") {
        out->mode = Mode::kMono;
      } else if (v == "mono_inertial" || v == "mono-inertial" || v == "vi" || v == "mono_imu") {
        out->mode = Mode::kMonoInertial;
      } else if (v == "stereo") {
        out->mode = Mode::kStereo;
      } else if (v == "lidar" || v == "lidar-only" || v == "lo") {
        out->mode = Mode::kLidar;
      } else if (v == "lidar_inertial" || v == "lidar-inertial" || v == "lidar_imu" ||
                 v == "li") {
        out->mode = Mode::kLidarInertial;
      } else if (v == "stereo_inertial" || v == "stereo-inertial" || v == "stereo_imu") {
        out->mode = Mode::kStereoInertial;
      } else if (v == "epipolar" || v == "epipolar-stereo" || v == "epipolar_stereo" ||
                 v == "point-line") {
        out->mode = Mode::kEpipolarStereo;
      } else if (v == "epipolar-inertial" || v == "epipolar_inertial" || v == "epipolar_imu") {
        out->mode = Mode::kEpipolarStereoInertial;
      } else if (v == "epipolar-vio" || v == "epipolar_vio" || v == "epipolar-tight" ||
                 v == "epipolar_tight") {
        out->mode = Mode::kEpipolarVio;
      } else if (v == "gnss" || v == "gnss-only" || v == "gnss_only" || v == "gnsso") {
        out->mode = Mode::kGnss;
      } else if (v == "gnss-inertial" || v == "gnss_inertial" || v == "gnss-imu" ||
                 v == "gnss_imu" || v == "gnss-ins" || v == "gnss_ins") {
        out->mode = Mode::kGnssInertial;
      } else if (v == "gnss-inertial-epipolar" || v == "gnss_inertial_epipolar" ||
                 v == "gnss-ie" || v == "gnss_ie" || v == "gvi") {
        out->mode = Mode::kGnssInertialEpipolar;
      } else if (v == "gnss-inertial-lidar" || v == "gnss_inertial_lidar" ||
                 v == "gnss-il" || v == "gnss_il" || v == "gil") {
        out->mode = Mode::kGnssInertialLidar;
      } else if (v == "gnss-inertial-lidar-epipolar" ||
                 v == "gnss_inertial_lidar_epipolar" || v == "gnss-ile" || v == "gnss_ile" ||
                 v == "gvil" || v == "gvli") {
        out->mode = Mode::kGnssInertialLidarEpipolar;
      } else if (v == "lvi" || v == "lidar-visual-inertial" || v == "lidar_visual_inertial" ||
                 v == "lidio") {
        out->mode = Mode::kLvi;
      } else {
        out->warnings.push_back(
            key + " = " + value +
            " is not one of mono | mono-inertial | stereo | stereo-inertial | lidar | "
            "lidar-inertial | epipolar | epipolar-inertial | epipolar-vio | gnss | gnss-inertial | gnss-inertial-epipolar | gnss-inertial-lidar | gnss-inertial-lidar-epipolar | lvi");
      }

    // ---- the input ----------------------------------------------------------------------------
    } else if (key == "input") {
      const std::string v = lower(value);
      if (v == "images" || v == "image") {
        out->image_source = ImageSource::kImages;
      } else if (v == "bag" || v == "rosbag") {
        out->image_source = ImageSource::kBag;
      } else {
        out->warnings.push_back(key + " = " + value + " is not `images` or `bag`");
      }
    } else if (key == "images") {
      out->images = value;
    } else if (key == "lidar_dir" || key == "lidar") {
      out->lidar_dir = value;
    } else if (key == "lidar_min_range") {
      double v = 0;
      if (parseDouble(value, &v)) out->lidar_min_range = static_cast<float>(v); else recognised = false;
    } else if (key == "lidar_max_range") {
      double v = 0;
      if (parseDouble(value, &v)) out->lidar_max_range = static_cast<float>(v); else recognised = false;
    } else if (key == "lidar_scan_voxel") {
      double v = 0;
      if (parseDouble(value, &v) && v > 0) out->lidar_scan_voxel = static_cast<float>(v); else recognised = false;
    } else if (key == "lidar_map_voxel") {
      double v = 0;
      if (parseDouble(value, &v) && v > 0) out->lidar_map_voxel = static_cast<float>(v); else recognised = false;
    } else if (key == "lidar_max_iterations") {
      double v = 0;
      if (parseDouble(value, &v) && v >= 1) out->lidar_max_iterations = static_cast<int>(v); else recognised = false;
    } else if (key == "lidar_huber") {
      double v = 0;
      if (parseDouble(value, &v) && v > 0) out->lidar_huber = static_cast<float>(v); else recognised = false;
    } else if (key == "r_bl" || key == "lidar_r_bl") {
      std::vector<double> v;
      if (parseList(value, &v) && v.size() == 9) {
        for (int i = 0; i < 9; ++i) out->r_bl[i] = v[(std::size_t)i];
        out->has_lidar_extrinsic = true;
      } else {
        out->warnings.push_back(key + " wants nine numbers, row major");
      }
    } else if (key == "t_bl" || key == "lidar_t_bl") {
      std::vector<double> v;
      if (parseList(value, &v) && v.size() == 3) {
        out->t_bl[0] = v[0];
        out->t_bl[1] = v[1];
        out->t_bl[2] = v[2];
        out->has_lidar_extrinsic = true;
      } else {
        out->warnings.push_back(key + " wants three numbers");
      }
    } else if (key == "images_dir" || key == "image_dir") {
      out->images_dir = value;
    } else if (key == "images_manifest" || key == "manifest" || key == "frames") {
      out->images_manifest = value;
    } else if (key == "images_right_dir" || key == "image_right_dir") {
      out->images_right_dir = value;
    } else if (key == "images_right") {
      out->images_right = value;
    } else if (key == "images_right_manifest" || key == "manifest_right") {
      out->images_manifest_right = value;
    } else if (key == "bag" || key == "bags") {
      // A FILE or a DIRECTORY.  A directory is read as every `.bag` in it, in playback order -- these
      // recordings ship as thirty-six one-minute bags and a trajectory needs all of them.
      out->bag = value;
      out->image_source = ImageSource::kBag;
    } else if (key == "topic" || key == "image_topic" || key == "left_topic") {
      out->bag_image_topic = value;
    } else if (key == "right_topic" || key == "image_topic_right") {
      out->bag_image_topic_right = value;
    } else if (key == "bag_start") {
      double v = 0.0;
      if (parseDouble(value, &v)) { out->bag_start = static_cast<int>(v); } else { recognised = false; }
    } else if (key == "bag_frames" || key == "frames_count") {
      double v = 0.0;
      if (parseDouble(value, &v)) { out->bag_frames = static_cast<int>(v); } else { recognised = false; }

    // ---- the inertial input --------------------------------------------------------------------
    } else if (key == "imu") {
      // A single key that says both "there is inertial data" and "here it is": a `.bag` is a bag, anything
      // else is a text file.  It is a convenience over the two explicit keys below, which win if both are
      // given, because the acceptance criterion asks for the source to be nameable and a solver should not
      // have to spell out `imu_source` when the extension already says it.
      const std::string v = lower(value);
      if (v == "off" || v == "none" || v.empty()) {
        out->imu_source = ImuSource::kNone;
      } else if (lower(value).size() > 4 && lower(value).compare(lower(value).size() - 4, 4, ".bag") == 0) {
        out->imu_source = ImuSource::kBag;
        out->imu_bag = value;
      } else {
        out->imu_source = ImuSource::kFile;
        out->imu_file = value;
      }
    } else if (key == "imu_source") {
      const std::string v = lower(value);
      if (v == "none" || v == "off") {
        out->imu_source = ImuSource::kNone;
      } else if (v == "bag") {
        out->imu_source = ImuSource::kBag;
      } else if (v == "file" || v == "text" || v == "csv") {
        out->imu_source = ImuSource::kFile;
      } else {
        out->warnings.push_back(key + " = " + value + " is not `none`, `bag` or `file`");
      }
    } else if (key == "imu_bag") {
      out->imu_bag = value;
      out->imu_source = ImuSource::kBag;
    } else if (key == "imu_topic") {
      out->imu_topic = value;
    } else if (key == "imu_file") {
      out->imu_file = value;
      out->imu_source = ImuSource::kFile;

    // ---- camera -----------------------------------------------------------------------------
    } else if (key == "vocabulary" || key == "vocab") {
      out->vocabulary = value;
    } else if (key == "settings_out") {
      out->settings_out = value;
    } else if (key == "camera_model") {
      out->camera.model = lower(value);
    } else if (key == "camera_right_model") {
      out->camera_right.model = lower(value);
    } else if (key == "camera_fx") { double v=0; if (parseDouble(value,&v)) out->camera.fx = v; else recognised=false; }
      else if (key == "camera_fy") { double v=0; if (parseDouble(value,&v)) out->camera.fy = v; else recognised=false; }
      else if (key == "camera_cx") { double v=0; if (parseDouble(value,&v)) out->camera.cx = v; else recognised=false; }
      else if (key == "camera_cy") { double v=0; if (parseDouble(value,&v)) out->camera.cy = v; else recognised=false; }
      else if (key == "camera_width") { double v=0; if (parseDouble(value,&v)) out->camera.width = (int)v; else recognised=false; }
      else if (key == "camera_height") { double v=0; if (parseDouble(value,&v)) out->camera.height = (int)v; else recognised=false; }
      else if (key == "camera_fps") { double v=0; if (parseDouble(value,&v)) out->camera.fps = v; else recognised=false; }
      else if (key == "camera_distortion" || key == "distortion" || key == "camera_distortion_coeffs") {
        if (!parseList(value, &out->camera.distortion)) { recognised = false; }
    } else if (key == "camera_right_fx") { double v=0; if (parseDouble(value,&v)) out->camera_right.fx = v; else recognised=false; }
      else if (key == "camera_right_fy") { double v=0; if (parseDouble(value,&v)) out->camera_right.fy = v; else recognised=false; }
      else if (key == "camera_right_cx") { double v=0; if (parseDouble(value,&v)) out->camera_right.cx = v; else recognised=false; }
      else if (key == "camera_right_cy") { double v=0; if (parseDouble(value,&v)) out->camera_right.cy = v; else recognised=false; }
      else if (key == "camera_right_width") { double v=0; if (parseDouble(value,&v)) out->camera_right.width = (int)v; else recognised=false; }
      else if (key == "camera_right_height") { double v=0; if (parseDouble(value,&v)) out->camera_right.height = (int)v; else recognised=false; }
      else if (key == "camera_right_distortion") {
        if (!parseList(value, &out->camera_right.distortion)) { recognised = false; }
    } else if (key == "t_bc" || key == "camera_t_bc") {
      std::vector<double> v;
      if (parseList(value, &v) && v.size() == 3) {
        out->camera.t_bc[0] = v[0]; out->camera.t_bc[1] = v[1]; out->camera.t_bc[2] = v[2];
        out->camera.has_extrinsic = true;
      } else {
        out->warnings.push_back(key + " wants three numbers, metres");
      }
    } else if (key == "r_bc" || key == "camera_r_bc") {
      std::vector<double> v;
      if (parseList(value, &v) && v.size() == 9) {
        for (int i = 0; i < 9; ++i) { out->camera.r_bc[i] = v[(std::size_t)i]; }
        out->camera.has_extrinsic = true;
      } else {
        out->warnings.push_back(key + " wants nine numbers, row major");
      }
    } else if (key == "t_bc_right" || key == "camera_right_t_bc") {
      std::vector<double> v;
      if (parseList(value, &v) && v.size() == 3) {
        out->camera_right.t_bc[0] = v[0]; out->camera_right.t_bc[1] = v[1]; out->camera_right.t_bc[2] = v[2];
        out->camera_right.has_extrinsic = true;
      } else {
        out->warnings.push_back(key + " wants three numbers, metres");
      }
    } else if (key == "r_bc_right" || key == "camera_right_r_bc") {
      std::vector<double> v;
      if (parseList(value, &v) && v.size() == 9) {
        for (int i = 0; i < 9; ++i) { out->camera_right.r_bc[i] = v[(std::size_t)i]; }
        out->camera_right.has_extrinsic = true;
      } else {
        out->warnings.push_back(key + " wants nine numbers, row major");
      }
    } else if (key == "baseline" || key == "stereo_baseline") {
      double v = 0.0;
      if (parseDouble(value, &v)) { out->baseline = v; out->has_baseline = true; } else { recognised = false; }

    // ---- imu noise ---------------------------------------------------------------------------
    } else if (key == "imu_frequency" || key == "imu_rate") {
      double v=0; if (parseDouble(value,&v)) out->imu.frequency = v; else recognised=false;
    } else if (key == "imu_noise_gyro" || key == "noise_gyro") {
      double v=0; if (parseDouble(value,&v)) out->imu.noise_gyro = v; else recognised=false;
    } else if (key == "imu_noise_acc" || key == "noise_acc") {
      double v=0; if (parseDouble(value,&v)) out->imu.noise_acc = v; else recognised=false;
    } else if (key == "imu_gyro_walk" || key == "gyro_walk") {
      double v=0; if (parseDouble(value,&v)) out->imu.gyro_walk = v; else recognised=false;
    } else if (key == "imu_acc_walk" || key == "acc_walk") {
      double v=0; if (parseDouble(value,&v)) out->imu.acc_walk = v; else recognised=false;
    } else if (key == "imu_time_shift" || key == "timeshift") {
      double v=0; if (parseDouble(value,&v)) out->imu.time_shift = v; else recognised=false;
    } else if (key == "imu_gyro_unit" || key == "imu_angular_unit" || key == "imu_gyro_units") {
      const std::string v = lower(value);
      if (v == "rad/s" || v == "rad" || v == "radians" || v == "radians/s" || v == "si") {
        out->imu.angular_unit = ImuConf::AngularUnit::kRadPerSec;
      } else if (v == "deg/s" || v == "deg" || v == "degrees" || v == "degrees/s" || v == "dps") {
        out->imu.angular_unit = ImuConf::AngularUnit::kDegPerSec;
      } else if (v == "rad/sample" || v == "rad/samp" || v == "radians/sample") {
        out->imu.angular_unit = ImuConf::AngularUnit::kRadPerSample;
      } else if (v == "deg/sample" || v == "deg/samp" || v == "degrees/sample") {
        out->imu.angular_unit = ImuConf::AngularUnit::kDegPerSample;
      } else {
        out->warnings.push_back(key + " = " + value +
                                " is not `rad/s`, `deg/s`, `rad/sample` or `deg/sample`");
        recognised = false;
      }
    } else if (key == "imu_accel_unit" || key == "imu_acc_unit" || key == "imu_accel_units") {
      const std::string v = lower(value);
      if (v == "m/s2" || v == "m/s^2" || v == "mps2" || v == "si") {
        out->imu.accel_unit = ImuConf::AccelUnit::kMetresPerSec2;
      } else if (v == "g" || v == "gravity") {
        out->imu.accel_unit = ImuConf::AccelUnit::kG;
      } else if (v == "m/s/sample" || v == "m/s/samp" || v == "dv") {
        out->imu.accel_unit = ImuConf::AccelUnit::kMetresPerSecPerSample;
      } else if (v == "g/sample" || v == "g/samp") {
        out->imu.accel_unit = ImuConf::AccelUnit::kGPerSample;
      } else {
        out->warnings.push_back(key + " = " + value +
                                " is not `m/s2`, `g`, `m/s/sample` or `g/sample`");
        recognised = false;
      }
    } else if (key == "min_initial_parallax_statistic" || key == "parallax_statistic") {
      const std::string v = lower(value);
      if (v == "mean" || v == "average" || v == "avg") {
        out->imu.parallax_statistic = ImuConf::ParallaxStatistic::kMean;
      } else if (v == "median" || v == "med") {
        out->imu.parallax_statistic = ImuConf::ParallaxStatistic::kMedian;
      } else {
        out->warnings.push_back(key + " = " + value + " is not `mean` or `median`");
        recognised = false;
      }
    } else if (key == "imu_gravity") {
      double v=0; if (parseDouble(value,&v)) out->imu.gravity = v; else recognised=false;
    } else if (key == "imu_reset_map_if_not_initialized" || key == "reset_map_if_not_initialized") {
      bool b = false;
      if (parseBool(value, &b)) {
        out->imu.reset_map_if_imu_not_init = b;
      } else {
        recognised = false;
      }
    } else if (key == "scale_refinement" || key == "slam_scale_refinement") {
      bool b = false;
      if (parseBool(value, &b)) {
        out->imu.scale_refinement = b;
      } else {
        recognised = false;
      }
    } else if (key == "imu_min_initial_parallax" || key == "min_initial_parallax") {
      double v = 0;
      if (parseDouble(value, &v)) {
        out->imu.min_initial_parallax = v;
      } else {
        recognised = false;
      }
    } else if (key == "imu_first_alignment_seconds" || key == "first_alignment_seconds") {
      double v = 0;
      if (parseDouble(value, &v) && v > 0.0) {
        out->first_alignment_seconds = v;
      } else {
        recognised = false;
      }
    } else if (key == "imu_second_alignment_seconds" || key == "second_alignment_seconds") {
      double v = 0;
      if (parseDouble(value, &v) && v > 0.0) {
        out->second_alignment_seconds = v;
      } else {
        recognised = false;
      }
    } else if (key == "prefetch_frames" || key == "prefetch") {
      bool b = false;
      if (parseBool(value, &b)) {
        out->prefetch_frames = b;
      } else {
        recognised = false;
      }
    } else if (key == "imu_reset_if_not_enough_motion" || key == "reset_if_not_enough_motion") {
      bool b = false;
      if (parseBool(value, &b)) {
        out->imu.reset_if_not_enough_motion = b;
      } else {
        recognised = false;
      }
    } else if (key == "imu_insert_kfs_when_lost" || key == "insert_kfs_when_lost") {
      bool b = false;
      if (parseBool(value, &b)) {
        out->imu.insert_kfs_when_lost = b;
      } else {
        recognised = false;
      }
    } else if (key == "imu_inertial_pose_tracking" || key == "inertial_pose_tracking") {
      bool b = false;
      if (parseBool(value, &b)) {
        out->imu.inertial_pose_tracking = b;
      } else {
        recognised = false;
      }
    } else if (key == "keyframe_min_frames") {
      double v = 0;
      if (parseDouble(value, &v) && v >= 0.0) {
        out->keyframe_min_frames = static_cast<int>(v);
      } else {
        recognised = false;
      }
    } else if (key == "imu_use_inertial_factors" || key == "use_inertial_factors") {
      bool b = false;
      if (parseBool(value, &b)) {
        out->imu.use_inertial_factors = b;
      } else {
        recognised = false;
      }
    } else if (key == "imu_init_min_keyframes" || key == "init_min_keyframes") {
      double v = 0;
      if (parseDouble(value, &v) && v >= 0.0) {
        out->imu.init_min_keyframes = static_cast<int>(v);
      } else {
        recognised = false;
      }
    } else if (key == "imu_estimate_scale" || key == "estimate_scale") {
      bool b = false;
      if (parseBool(value, &b)) {
        out->imu.estimate_scale = b;
      } else {
        recognised = false;
      }
    } else if (key == "imu_min_inliers_pre_init" || key == "min_inliers_pre_init") {
      double v = 0;
      if (parseDouble(value, &v) && v >= 1.0) {
        out->imu.min_inliers_pre_init = static_cast<int>(v);
      } else {
        recognised = false;
      }
    } else if (key == "relocalize_before_imu_init" || key == "imu_relocalize_before_init" ||
               key == "relocalize_when_imu_not_initialized") {
      bool b = false;
      if (parseBool(value, &b)) {
        out->imu.relocalize_before_imu_init = b;
      } else {
        recognised = false;
      }
    } else if (key == "imu_fast_init" || key == "fast_init") {
      bool b=false; if (parseBool(value,&b)) out->imu.fast_init = b; else recognised=false;
    } else if (key == "mounting_rpy") {
      std::vector<double> v;
      if (parseList(value, &v) && v.size() == 3) {
        out->mounting.rpy_deg[0]=v[0]; out->mounting.rpy_deg[1]=v[1]; out->mounting.rpy_deg[2]=v[2];
        out->mounting.has_rpy = true;
      } else {
        out->warnings.push_back(key + " wants three numbers, degrees ZYX");
      }

    // ---- the run ------------------------------------------------------------------------------
    } else if (key == "epipolar_features") {
      double v = 0.0;
      if (parseDouble(value, &v) && v > 0.0) {
        out->epipolar_features = static_cast<int>(v);
      } else {
        recognised = false;
      }
    } else if (key == "epipolar_keyframe_distance") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 0.0) {
        out->epipolar_keyframe_distance = v;
      } else {
        recognised = false;
      }
    } else if (key == "epipolar_keyframe_degrees") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 0.0) {
        out->epipolar_keyframe_degrees = v;
      } else {
        recognised = false;
      }
    } else if (key == "epipolar_window") {
      const std::string v = lower(value);
      if (v == "auto" || v == "on" || v == "off") {
        out->epipolar_window = v;
      } else {
        out->warnings.push_back(key + " = " + value + " is not `auto`, `on` or `off`");
      }
    } else if (key == "epipolar_window_frames") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 3.0 && v <= 10.0) {
        out->epipolar_window_frames = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " expects an integer in [3, 10]");
      }
    } else if (key == "epipolar_prefetch") {
      const std::string v = lower(value);
      if (v == "on" || v == "off") {
        out->epipolar_prefetch = (v == "on");
      } else {
        out->warnings.push_back(key + " = " + value + " is not `on` or `off`");
      }
    } else if (key == "epipolar_vio_output") {
      const std::string v = lower(value);
      if (v == "vio" || v == "vo") {
        out->epipolar_vio_output = v;
      } else {
        out->warnings.push_back(key + " = " + value + " is not `vio` or `vo`");
      }
    } else if (key == "epipolar_imu_weight") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 0.0 && v <= 1.0) {
        out->epipolar_imu_weight = v;
      } else {
        out->warnings.push_back(key + " expects a value in [0, 1]");
      }
    } else if (key == "epipolar_vio_slide") {
      const std::string v = lower(value);
      if (v == "partial" || v == "full") {
        out->epipolar_vio_slide = v;
      } else {
        out->warnings.push_back(key + " = " + value + " is not `partial` or `full`");
      }
    } else if (key == "epipolar_vio_window") {
      double v = 0.0;
      // 8 retained frames x 15 states is the dense prior edge's compiled maximum (126 rows).
      if (parseDouble(value, &v) && v >= 2.0 && v <= 8.0) {
        out->epipolar_vio_window = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " expects an integer in [2, 8]");
      }
    } else if (key == "epipolar_vio_high_rate") {
      const std::string v = lower(value);
      if (v == "on" || v == "off") {
        out->epipolar_vio_high_rate = (v == "on");
      } else {
        out->warnings.push_back(key + " = " + value + " is not `on` or `off`");
      }
    } else if (key == "epipolar_vio_high_rate_frames") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 10.0 && v <= 200.0) {
        out->epipolar_vio_high_rate_frames = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " expects an integer in [10, 200]");
      }
    } else if (key == "epipolar_vio_visual_keyframe") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 1.0 && v <= 20.0) {
        out->epipolar_vio_visual_keyframe = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " = " + value + " wants an integer in [1, 20]");
      }
    } else if (key == "epipolar_vio_matches") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 20.0 && v <= 1000.0) {
        out->epipolar_vio_matches = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " expects an integer in [20, 1000]");
      }
    } else if (key == "epipolar_vio_vo_prior_trans" || key == "epipolar_vio_trans_sigma") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 0.001 && v <= 5.0) {
        out->epipolar_vio_vo_prior_trans = v;
      } else {
        out->warnings.push_back(key + " expects a value in [0.001, 5] m");
      }
    } else if (key == "epipolar_vio_vo_prior_rot_deg" || key == "epipolar_vio_rot_sigma_deg") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 0.01 && v <= 30.0) {
        out->epipolar_vio_vo_prior_rot_deg = v;
      } else {
        out->warnings.push_back(key + " expects a value in [0.01, 30] deg");
      }
    } else if (key == "epipolar_vio_visual_sigma") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 0.05 && v <= 20.0) {
        out->epipolar_vio_visual_sigma = v;
      } else {
        out->warnings.push_back(key + " expects a value in [0.05, 20]");
      }
    } else if (key == "epipolar_vio_bias_sigma") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 0.001 && v <= 5.0) {
        out->epipolar_vio_bias_sigma = v;
      } else {
        out->warnings.push_back(key + " expects a value in [0.001, 5]");
      }
    } else if (key == "epipolar_vio_fix_biases") {
      const std::string v = lower(value);
      if (v == "on" || v == "off") {
        out->epipolar_vio_fix_biases = (v == "on");
      } else {
        out->warnings.push_back(key + " = " + value + " is not `on` or `off`");
      }
    } else if (key == "epipolar_vio_marginalize") {
      const std::string v = lower(value);
      if (v == "on" || v == "off") {
        out->epipolar_vio_marginalize = (v == "on");
      } else {
        out->warnings.push_back(key + " = " + value + " is not `on` or `off`");
      }
    } else if (key == "epipolar_vio_warmup_moving") {
      const std::string v = lower(value);
      if (v == "on" || v == "off") {
        out->epipolar_vio_warmup_moving = (v == "on");
      } else {
        out->warnings.push_back(key + " = " + value + " is not `on` or `off`");
      }
    } else if (key == "epipolar_vio_warmup_distance") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 0.0 && v <= 20.0) {
        out->epipolar_vio_warmup_distance = v;
      } else {
        out->warnings.push_back(key + " expects a value in [0, 20] m");
      }
    } else if (key == "epipolar_vio_vo_anchor") {
      const std::string v = lower(value);
      if (v == "on" || v == "off") {
        out->epipolar_vio_vo_anchor = (v == "on");
      } else {
        out->warnings.push_back(key + " = " + value + " is not `on` or `off`");
      }
    } else if (key == "epipolar_pnp_radius") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 2.0 && v <= 40.0) {
        out->epipolar_pnp_radius = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " expects a value in [2, 40]");
      }
    } else if (key == "epipolar_pnp_features") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 0.0 && v <= 4000.0) {
        out->epipolar_pnp_features = static_cast<int>(v);
      } else {
        out->warnings.push_back(key + " expects a value in [0, 4000] (0 = all)");
      }
    } else if (key == "epipolar_pnp_max_depth") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 5.0 && v <= 200.0) {
        out->epipolar_pnp_max_depth = v;
      } else {
        out->warnings.push_back(key + " expects a value in [5, 200] m");
      }
    } else if (key == "epipolar_stationary_flow") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 0.0 && v <= 5.0) {
        out->epipolar_stationary_flow = v;
      } else {
        out->warnings.push_back(key + " expects a value in [0, 5] px");
      }
    } else if (key == "epipolar_threads") {
      double v = 0.0;
      if (parseDouble(value, &v) && v >= 0.0) {
        out->epipolar_threads = static_cast<int>(v);
      } else {
        recognised = false;
      }
    } else if (key == "features" || key == "n_features") {
      double v=0; if (parseDouble(value,&v)) out->features = (int)v; else recognised=false;
    } else if (key == "depth") {
      // 0 = one descriptor per keyframe kept for loop closure (default), 1 = one per point.
      double v=0; if (parseDouble(value,&v)) { /* ORB-SLAM3 reads it from the settings file */ } else recognised=false;
    } else if (key == "scale_factor") {
      double v=0; if (parseDouble(value,&v)) out->scale_factor = (float)v; else recognised=false;
    } else if (key == "levels" || key == "n_levels") {
      double v=0; if (parseDouble(value,&v)) out->levels = (int)v; else recognised=false;
    } else if (key == "ini_th_fast" || key == "ini_th_fast") {
      double v=0; if (parseDouble(value,&v)) out->ini_th_fast = (int)v; else recognised=false;
    } else if (key == "min_th_fast") {
      double v=0; if (parseDouble(value,&v)) out->min_th_fast = (int)v; else recognised=false;
    } else if (key == "duration") {
      double v=0; if (parseDouble(value,&v)) out->duration = v; else recognised=false;
    } else if (key == "start_frame") {
      double v=0; if (parseDouble(value,&v)) out->start_frame = (int)v; else recognised=false;
    } else if (key == "debug_every") {
      double v=0; if (parseDouble(value,&v) && v >= 1.0) out->debug_every = (int)v; else recognised=false;
    } else if (key == "viewer") {
      bool b=false; if (parseBool(value,&b)) out->viewer = b; else recognised=false;
    } else if (key == "save_map") {
      bool b=false; if (parseBool(value,&b)) out->save_map = b; else recognised=false;

    // ---- output -------------------------------------------------------------------------------
    } else if (key == "solution" || key == "output") {
      out->solution = value;
    } else if (key == "log") {
      out->log_path = value;
    } else {
      recognised = false;
    }

    if (!recognised) {
      out->unknown_keys.push_back(key);
      out->warnings.push_back("line " + std::to_string(line_no) + ": unrecognised key `" + key + "`");
    }
  }

  // ---- the mode decides what is required -------------------------------------------------------
  // Checked here rather than at the point of use, so that a configuration missing an intrinsic fails with
  // the key's name instead of with an ORB-SLAM3 assertion three seconds into a run.
  const bool lvi = isLviMode(out->mode);
  const bool stereo = isStereoMode(out->mode) || lvi;
  const bool inertial = isInertialMode(out->mode);
  const bool lidar_mode = isLidarMode(out->mode) || lvi;
  const bool gnss_mode = isGnssMode(out->mode);
  std::vector<std::string> missing;
  if (!gnss_mode) {
    if (out->image_source == ImageSource::kBag) {
      if (out->bag.empty()) { missing.push_back("bag"); }
      if (out->bag_image_topic.empty()) { missing.push_back("topic"); }
    } else {
      if (lidar_mode && out->lidar_dir.empty()) {
        missing.push_back("lidar_dir");
      }
      // LVI has a LiDAR and an image stream; the pure LiDAR modes have no images.
      if ((!lidar_mode || lvi) && out->images.empty() && out->images_dir.empty() &&
          out->images_manifest.empty()) {
        missing.push_back("images_dir (or images_manifest)");
      }
    }
  }
  const bool lidar_only = isLidarMode(out->mode);
  const bool no_camera = lidar_only || gnss_mode;
  // The epipolar branch never starts the visual SLAM system: no vocabulary, no ORB settings, only the
  // stereo geometry.  The GNSS routes have no camera at all.
  if (!no_camera && !isEpipolarMode(out->mode) && !lvi && out->vocabulary.empty()) {
    missing.push_back("vocabulary");
  }
  if (!no_camera && (out->camera.fx <= 0.0 || out->camera.fy <= 0.0)) {
    missing.push_back("camera_fx / camera_fy");
  }
  if (!no_camera && (out->camera.width <= 0 || out->camera.height <= 0)) {
    missing.push_back("camera_width / camera_height");
  }
  if (gnss_mode) {
    if (!out->gnss_dd) {
      missing.push_back("gnss_dd = on (the gnss modes are the DD solver)");
    }
    if (out->gnss_rover.empty() || out->gnss_base.empty() || out->gnss_nav.empty()) {
      missing.push_back("gnss_rover / gnss_base / gnss_nav");
    }
    // The first validation uses the reference antenna for the DD geometry/ambiguity initialisation.  A
    // deployment route has to replace this with an SPP/RTK initialiser; until then the reference is
    // required even for a code-only run.
    if (out->gnss_truth.empty()) {
      missing.push_back("gnss_truth (needed for the DD antenna/ambiguity initialisation)");
    }
  }
  if (stereo) {
    if (out->camera_right.fx <= 0.0) { missing.push_back("camera_right_fx"); }
    if (out->camera_right.width <= 0) { missing.push_back("camera_right_width / camera_right_height"); }
    if (!out->camera_right.has_extrinsic && !out->has_baseline) {
      missing.push_back("baseline (or t_bc_right / r_bc_right)");
    }
  }
  // `epipolar-vio` with `imu = none` is the visual-only fixed-lag window: the same point-line sliding
  // window and Schur marginalization without inertial factors.  It is the only inertial-labelled mode that
  // may legitimately name no IMU source.
  const bool visual_only_vio =
      out->mode == Mode::kEpipolarVio && out->imu_source == ImuSource::kNone;
  if (inertial && !visual_only_vio) {
    if (out->imu_source == ImuSource::kNone) {
      missing.push_back("imu (the mode asks for inertial data)");
    }
    if (out->imu_source == ImuSource::kBag && out->imu_topic.empty() && out->imu_bag.empty()) {
      missing.push_back("imu_bag / imu_topic");
    }
    if (out->imu_source == ImuSource::kFile && out->imu_file.empty()) {
      missing.push_back("imu_file");
    }
    // The epipolar branch estimates the gyro bias from the data; it does not consume a noise model, so
    // the noise densities are not inputs it must be given.
    if (!isEpipolarMode(out->mode) && (out->imu.noise_gyro <= 0.0 || out->imu.noise_acc <= 0.0)) {
      missing.push_back("imu_noise_gyro / imu_noise_acc");
    }
    // A per-sample unit is converted by multiplying by the sample rate, so without the rate there is no
    // conversion to make -- and guessing one would be a silent factor-of-N scale error, which is the
    // failure the unit keys exist to prevent.
    if (out->imu.unitsArePerSample() && out->imu.frequency <= 0.0) {
      missing.push_back("imu_frequency (the per-sample IMU units are converted with it)");
    }
  }
  if (!missing.empty()) {
    std::string text = "the " + std::string(modeName(out->mode)) +
                       " configuration is missing: ";
    for (std::size_t i = 0; i < missing.size(); ++i) {
      text += (i == 0 ? "" : ", ") + missing[i];
    }
    if (error != nullptr) {
      *error = text;
    }
    return false;
  }
  return true;
}

std::vector<std::string> describeConf(const Conf& conf) {
  std::vector<std::string> out;
  char buffer[512];
  out.push_back(std::string("mode                 ") + modeName(conf.mode));
  out.push_back(std::string("ALI                  ") +
                (conf.ali ? "on (anisotropic landmark initialization)" : "off (stock ORB-SLAM3)"));
  std::snprintf(buffer, sizeof(buffer), "input                %s",
                isGnssMode(conf.mode)
                    ? "none (GNSS route)"
                    : (conf.image_source == ImageSource::kBag ? "bag" : "images"));
  out.push_back(buffer);
  if (conf.image_source == ImageSource::kBag) {
    out.push_back("bag                  " + conf.bag);
    out.push_back("topic (left)         " + conf.bag_image_topic);
    if (!conf.bag_image_topic_right.empty()) {
      out.push_back("topic (right)        " + conf.bag_image_topic_right);
    }
  } else {
    if (!conf.images_dir.empty()) { out.push_back("images_dir           " + conf.images_dir); }
    if (!conf.images_manifest.empty()) { out.push_back("images_manifest      " + conf.images_manifest); }
  }
  const bool inertial = isInertialMode(conf.mode);
  std::snprintf(buffer, sizeof(buffer), "inertial             %s",
                conf.imu_source == ImuSource::kFile ? "file"
                : conf.imu_source == ImuSource::kBag ? "bag" : "none");
  out.push_back(buffer);
  if (conf.imu_source == ImuSource::kFile) { out.push_back("imu_file             " + conf.imu_file); }
  if (conf.imu_source == ImuSource::kBag) {
    out.push_back("imu_bag              " + conf.imu_bag);
    out.push_back("imu_topic            " + conf.imu_topic);
  }
  if (conf.imu_source != ImuSource::kNone) {
    // Printed because it is the one input whose being wrong produces a plausible-looking run that is
    // simply scaled wrong, so a reader should be able to see what the run was told without opening the file.
    const char* angular = "rad/s";
    switch (conf.imu.angular_unit) {
      case ImuConf::AngularUnit::kRadPerSec:    angular = "rad/s"; break;
      case ImuConf::AngularUnit::kDegPerSec:    angular = "deg/s"; break;
      case ImuConf::AngularUnit::kRadPerSample: angular = "rad/sample"; break;
      case ImuConf::AngularUnit::kDegPerSample: angular = "deg/sample"; break;
    }
    const char* accel = "m/s^2";
    switch (conf.imu.accel_unit) {
      case ImuConf::AccelUnit::kMetresPerSec2:         accel = "m/s^2"; break;
      case ImuConf::AccelUnit::kG:                     accel = "g"; break;
      case ImuConf::AccelUnit::kMetresPerSecPerSample: accel = "m/s per sample"; break;
      case ImuConf::AccelUnit::kGPerSample:            accel = "g per sample"; break;
    }
    if (conf.imu.unitsArePerSample()) {
      std::snprintf(buffer, sizeof(buffer),
                    "imu units in the file %s / %s  ->  x %.4g Hz  ->  rad/s and m/s^2", angular, accel,
                    conf.imu.frequency);
    } else {
      std::snprintf(buffer, sizeof(buffer),
                    "imu units in the file %s / %s  ->  converted to rad/s and m/s^2", angular, accel);
    }
    out.push_back(buffer);
  }
  std::snprintf(buffer, sizeof(buffer), "camera (left)        %s  fx %.4f fy %.4f cx %.4f cy %.4f  %dx%d",
                conf.camera.model.c_str(), conf.camera.fx, conf.camera.fy, conf.camera.cx, conf.camera.cy,
                conf.camera.width, conf.camera.height);
  out.push_back(buffer);
  std::string dist = "distortion (left)    ";
  for (double d : conf.camera.distortion) {
    std::snprintf(buffer, sizeof(buffer), "%.6f ", d);
    dist += buffer;
  }
  out.push_back(dist + (conf.camera.distortion.empty() ? "(none)" : ""));
  if (isStereoMode(conf.mode)) {
    std::snprintf(buffer, sizeof(buffer), "camera (right)       %s  fx %.4f fy %.4f cx %.4f cy %.4f  %dx%d",
                  conf.camera_right.model.c_str(), conf.camera_right.fx, conf.camera_right.fy,
                  conf.camera_right.cx, conf.camera_right.cy, conf.camera_right.width,
                  conf.camera_right.height);
    out.push_back(buffer);
    if (conf.has_baseline) {
      std::snprintf(buffer, sizeof(buffer), "baseline             %.8f m", conf.baseline);
      out.push_back(buffer);
    }
  }
  if (conf.camera.has_extrinsic) {
    std::snprintf(buffer, sizeof(buffer), "T_bc (left)          t (%.6f, %.6f, %.6f) m",
                  conf.camera.t_bc[0], conf.camera.t_bc[1], conf.camera.t_bc[2]);
    out.push_back(buffer);
  }
  if (inertial) {
    std::snprintf(buffer, sizeof(buffer),
                  "imu noise            gyro %.3e acc %.3e  walk %.3e / %.3e", conf.imu.noise_gyro,
                  conf.imu.noise_acc, conf.imu.gyro_walk, conf.imu.acc_walk);
    out.push_back(buffer);
    std::snprintf(buffer, sizeof(buffer), "imu time shift       %.8f s   fast init %s", conf.imu.time_shift,
                  conf.imu.fast_init ? "on" : "off");
    out.push_back(buffer);
  }
  if (isLidarMode(conf.mode)) {
    std::snprintf(buffer, sizeof(buffer), "lidar                %s", conf.lidar_dir.c_str());
    out.push_back(buffer);
    if (conf.lidar_window >= 2) {
      std::snprintf(buffer, sizeof(buffer),
                    "lidar window         %d scan(s), output %s, anchor %s%s, marginalize %s, biases %s",
                    conf.lidar_window, conf.lidar_window_output.c_str(),
                    conf.lidar_window_anchor ? "on" : "off",
                    conf.lidar_window_anchor_all ? " (all frames)" : " (oldest only)",
                    conf.lidar_window_marginalize ? "on" : "off",
                    conf.lidar_window_fix_biases ? "fixed" : "free");
      out.push_back(buffer);
      std::snprintf(buffer, sizeof(buffer),
                    "lidar window sigma   point %.3f m / %d pts, anchor %.3f m / %.2f deg",
                    conf.lidar_window_point_sigma, conf.lidar_window_points,
                    conf.lidar_window_prior_trans, conf.lidar_window_prior_rot_deg);
      out.push_back(buffer);
    }
  }
  // `scale_factor`, not `camera.fps`.  The fps argument sat in the `%.2f scale` slot, so every summary this
  // project has printed reported the frame rate as the ORB scale factor -- "features 4000 per frame, 10.00
  // scale" for a configuration whose scale factor is 1.2.
  std::snprintf(buffer, sizeof(buffer), "features             %d per frame, %.2f scale, %d levels",
                conf.features, conf.scale_factor, conf.levels);
  out.push_back(buffer);
  if (conf.imu.min_initial_parallax > 0.0 || conf.imu.parallax_statistic == ImuConf::ParallaxStatistic::kMedian) {
    // Printed because the two statistics give very different answers on the same footage, and the value
    // alone does not say which one produced it.
    std::snprintf(buffer, sizeof(buffer), "initial parallax     %.1f px of %s disparity%s",
                  conf.imu.min_initial_parallax,
                  conf.imu.parallax_statistic == ImuConf::ParallaxStatistic::kMedian ? "median" : "mean",
                  conf.imu.min_initial_parallax > 0.0 ? "" : "  (gate off)");
    out.push_back(buffer);
  }
  if (conf.imu.min_inliers_pre_init != 50) {
    // Printed only when it differs from upstream, so a run that leaves it alone stays silent and one that
    // changes it says so -- the value is the difference between a monocular-inertial map surviving its
    // first frames and not.
    std::snprintf(buffer, sizeof(buffer), "pre-init inliers     %d matched (upstream 50)",
                  conf.imu.min_inliers_pre_init);
    out.push_back(buffer);
  }
  out.push_back("vocabulary           " + conf.vocabulary);
  return out;
}

}  // namespace ali
