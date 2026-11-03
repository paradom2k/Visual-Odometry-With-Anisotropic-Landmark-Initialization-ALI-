// openvins_ali -- driver for the OpenVINS kernel.
//
// Feeds a 4Seasons manifest plus its IMU csv into ov_msckf::VioManager and writes the resulting trajectory in the
// scorer's format ("<timestamp> x y z"), which is what the rest of this project already consumes.
//
//   openvins_ali_run --manifest <file> --imu <file> --config <yaml> --out <trajectory>
//                             [--frames N] [--start K] [--base-dir <dir>] [--quiet]
//
// This replaces the old VINS feature_tracker + hand-written g2o sliding window, which tracked well for ~30 s and
// then diverged (measured: |v| 405 m/s, positions 50 km out, path ratio 22.8 over the full drive).  The estimator
// here is OpenVINS's MSCKF, used as-is: nothing in thirdparty/openvins is modified except the five variadic print
// macros that MSVC cannot parse.

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/sensor_data.h"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct ManifestEntry {
  double time = 0.0;
  std::string path;
};

std::vector<ManifestEntry> readManifest(const std::string& path, const std::string& base_dir, int start, int count) {
  std::vector<ManifestEntry> entries;
  std::ifstream stream(path.c_str());
  if (!stream) {
    std::fprintf(stderr, "[E] cannot open manifest %s\n", path.c_str());
    return entries;
  }
  std::string base = base_dir;
  if (base.empty()) {
    const std::size_t slash = path.find_last_of("/\\");
    if (slash != std::string::npos) base = path.substr(0, slash + 1);
  } else if (base.back() != '/' && base.back() != '\\') {
    base += '\\';
  }

  std::string line;
  int index = 0;
  while (std::getline(stream, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream parser(line);
    ManifestEntry entry;
    if (!(parser >> entry.time)) continue;
    // The path is everything after the timestamp, not the next whitespace-delimited
    // token: dataset directories contain spaces ("D:\driving dataset\COMPUTER VISION
    // GROUP\..."), and reading a token truncated the path to "D:/driving", which
    // surfaced as cv::imread failing on the first frame.  This is the same rule the
    // statistics tool's readManifest uses, so one manifest works for both.
    std::getline(parser, entry.path);
    const std::size_t first = entry.path.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) continue;
    const std::size_t last = entry.path.find_last_not_of(" \t\r\n");
    entry.path = entry.path.substr(first, last - first + 1);
    if (index++ < start) continue;
    // A drive-letter path is already absolute; the runner must not prefix it.
    if (!(entry.path.size() > 1 && entry.path[1] == ':')) entry.path = base + entry.path;
    entries.push_back(entry);
    if (count > 0 && static_cast<int>(entries.size()) >= count) break;
  }
  return entries;
}

/// The IMU text format this workspace uses: a header line then "t,gx,gy,gz,ax,ay,az".
/// OpenVINS's ImuData holds the raw gyro and accel in SI units, which is what those columns already are.
std::vector<ov_core::ImuData> readImu(const std::string& path) {
  std::vector<ov_core::ImuData> samples;
  std::ifstream stream(path.c_str());
  if (!stream) {
    std::fprintf(stderr, "[E] cannot open IMU file %s\n", path.c_str());
    return samples;
  }
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::vector<double> values;
    std::string field;
    std::stringstream parser(line);
    while (std::getline(parser, field, ',')) values.push_back(std::atof(field.c_str()));
    if (values.size() < 7) continue;
    ov_core::ImuData sample;
    sample.timestamp = values[0];
    sample.wm << values[1], values[2], values[3];
    sample.am << values[4], values[5], values[6];
    samples.push_back(sample);
  }
  return samples;
}

/// ov_core::YamlParser derives the folder of an external config with find_last_of('/'), so a path containing only
/// backslashes yields a nonsense filename ("...\config.yaml" + "/" + "calib.yaml").  Feeding it forward slashes
/// keeps the external calibration loadable -- and this is not hypothetical: without it parse_external_yaml exits.
std::string ToForwardSlashes(std::string path) {
  std::replace(path.begin(), path.end(), '\\', '/');
  return path;
}

}  // namespace

int main(int argc, char** argv) {
  std::string manifest;
  std::string imu_path;
  std::string config;
  std::string out_path;
  std::string base_dir;
  int frames = 0;
  int start = 0;
  bool verbose = true;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const bool has_next = (i + 1) < argc;
    if (arg == "--manifest" && has_next) manifest = argv[++i];
    else if (arg == "--imu" && has_next) imu_path = argv[++i];
    else if (arg == "--config" && has_next) config = argv[++i];
    else if (arg == "--out" && has_next) out_path = argv[++i];
    else if (arg == "--base-dir" && has_next) base_dir = argv[++i];
    else if (arg == "--frames" && has_next) frames = std::atoi(argv[++i]);
    else if (arg == "--start" && has_next) start = std::atoi(argv[++i]);
    else if (arg == "--quiet") verbose = false;
    else if (arg == "--help" || arg == "-h") {
      std::printf(
          "openvins_ali_run -- drives the OpenVINS MSCKF kernel over a 4Seasons manifest.\n\n"
          "  --manifest <file>   \"<timestamp> <image path>\" per line\n"
          "  --config <yaml>     OpenVINS estimator configuration; it names its own calibration files\n"
          "  --imu <file>        \"t,gx,gy,gz,ax,ay,az\" per line\n"
          "  --out <trajectory>  written as \"<timestamp> x y z\", full precision\n"
          "  --base-dir <dir>    prefix for relative image paths in the manifest\n"
          "  --frames N          stop after N frames (0 or absent = all)\n"
          "  --start K           skip the first K frames\n"
          "  --quiet             do not print the every-100-frames progress line\n\n"
          "Normally run through bin/openvins_ali_app.exe, which builds this command line from a conf/ file.\n");
      return 0;
    }
    else {
      std::fprintf(stderr, "unknown or incomplete argument: %s\n", arg.c_str());
      return 2;
    }
  }
  if (manifest.empty() || config.empty()) {
    std::fprintf(stderr,
                 "usage: %s --manifest <file> --config <yaml> [--imu <file>] [--out <trajectory>]\n"
                 "          [--base-dir <dir>] [--frames N] [--start K] [--quiet]\n",
                 argv[0]);
    return 2;
  }

  // The configuration is checked first, before the manifest and the IMU are read.  It is the one
  // input whose absence is fatal and unambiguous, and two configurations in conf/ deliberately point
  // at a file that does not exist yet (kitti_00, gici_1_1 -- no inertial data for either), so the
  // first line of such a run should be the reason, not a warning about the IMU that follows it.
  {
    std::ifstream probe(config.c_str());
    if (!probe) {
      std::fprintf(stderr, "[E] cannot open the configuration file %s\n", config.c_str());
      return 1;
    }
  }

  const std::vector<ManifestEntry> entries = readManifest(manifest, base_dir, start, frames);
  if (entries.empty()) {
    std::fprintf(stderr, "[E] no frames in %s\n", manifest.c_str());
    return 1;
  }
  const std::vector<ov_core::ImuData> imu = imu_path.empty() ? std::vector<ov_core::ImuData>() : readImu(imu_path);
  if (imu.empty()) {
    // Not fatal in OpenVINS's own design, but without inertial data this is a pure visual system and would be a
    // different experiment; say so rather than silently running one.
    std::fprintf(stderr, "[W] no IMU samples: the run will not be visual-inertial\n");
  }
  std::printf("[I] %d frame(s), %d IMU sample(s)\n", static_cast<int>(entries.size()), static_cast<int>(imu.size()));

  // ------------------------------------------------------------------ configuration
  ov_msckf::VioManagerOptions params;
  {
    auto parser = std::make_shared<ov_core::YamlParser>(ToForwardSlashes(config));
    params.print_and_load(parser);
    if (!parser->successful()) {
      // print_and_load marks the parser when a required key was missing.  Continuing would run with defaults, which
      // is exactly the kind of silent difference this project has been careful to avoid.
      std::fprintf(stderr, "[E] configuration %s is incomplete (a required key was not found)\n", config.c_str());
      return 1;
    }
  }
  std::printf("[I] OpenVINS configured: %d camera(s), %d points, %d clones, dyn init %s\n",
              params.state_options.num_cameras, params.num_pts, params.state_options.max_clone_size,
              params.init_options.init_dyn_use ? "on" : "off");
  std::fflush(stdout);

  // ------------------------------------------------------------------ run
  ov_msckf::VioManager manager(params);

  // The trajectory is written as it is produced rather than collected and dumped at the end, so that a viewer --
  // the application's plot, or a person with the file open -- can follow the run while it happens.  Everything is
  // flushed per pose: at ~30 poses a second the cost is irrelevant, and it is what makes the file readable at any
  // instant instead of only after the process exits.
  //
  // Full precision, not the stream default.  These timestamps are Unix-epoch seconds, so with the default six
  // significant digits every line reads "1.60208e+09" and the sub-second part -- which is the whole point of the
  // column -- is gone.  Measured: the scorer then reports "only 0 pose(s) overlap the reference window", because
  // every pose appears to be at the same instant.  Seventeen significant digits round-trips a double exactly.
  std::ofstream trajectory_out;
  if (!out_path.empty()) {
    trajectory_out.open(out_path.c_str());
    if (!trajectory_out) {
      std::fprintf(stderr, "[E] cannot write %s\n", out_path.c_str());
      return 1;
    }
    trajectory_out << std::setprecision(17);
  }

  std::vector<double> pose_time, pose_x, pose_y, pose_z;
  std::size_t imu_cursor = 0;
  double previous_frame_time = entries.front().time;
  int initialized_at = -1;

  for (std::size_t i = 0; i < entries.size(); ++i) {
    const double stamp = entries[i].time;

    // Every IMU sample up to this frame belongs to the interval that ends here.  They are fed one at a time in
    // timestamp order, which is what the propagator expects.
    while (imu_cursor < imu.size() && imu[imu_cursor].timestamp <= stamp) {
      manager.feed_measurement_imu(imu[imu_cursor]);
      ++imu_cursor;
    }

    cv::Mat image = cv::imread(entries[i].path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
      std::fprintf(stderr, "[E] cannot read %s\n", entries[i].path.c_str());
      return 1;
    }

    // Optional additive Gaussian noise, for the experiment ensemble: OV_NOISE=<sigma in grey levels>,
    // OV_SEED=<integer>.  Seeded per (seed, frame) so a run reproduces exactly, and inactive unless the
    // variable is set -- every ordinary run is bit-identical to before.
    {
      const char *ns = std::getenv("OV_NOISE");
      const double sigma = ns ? std::atof(ns) : 0.0;
      if (sigma > 0.0) {
        const char *ss = std::getenv("OV_SEED");
        const unsigned seed = (unsigned)(ss ? std::atoi(ss) : 0) * 7919u + (unsigned)i;
        cv::Mat noise(image.size(), CV_16S);
        cv::RNG rng(seed);
        rng.fill(noise, cv::RNG::NORMAL, 0.0, sigma);
        cv::Mat wide;
        image.convertTo(wide, CV_16S);
        wide += noise;
        wide.convertTo(image, CV_8U);
      }
    }

    ov_core::CameraData message;
    message.timestamp = stamp;
    message.sensor_ids.push_back(0);
    message.images.push_back(image);
    // One mask entry per camera.  "No mask" is an all-zero image of the frame's size, not an empty Mat: TrackKLT
    // resizes and indexes the mask unconditionally, so an empty one fails inside cv::resize, and 255 (not 0) is the
    // value that means "masked out", so an all-255 image would silently suppress every extracted feature.  This
    // matches what upstream does wherever it has no mask to apply -- see VioManager.cpp and the ROS visualizers,
    // which all push cv::Mat::zeros(rows, cols, CV_8UC1).
    message.masks.push_back(cv::Mat::zeros(image.rows, image.cols, CV_8UC1));
    manager.feed_measurement_camera(message);

    if (manager.initialized()) {
      if (initialized_at < 0) {
        initialized_at = static_cast<int>(i);
        std::printf("[I] initialised at frame %d (t %.3f, %.3f s after the first frame)\n", initialized_at, stamp,
                    stamp - entries.front().time);
        std::fflush(stdout);
      }
      // The current IMU pose.  Read after the update so the recorded pose is the one the filter now believes.
      const auto state = manager.get_state();
      const Eigen::Vector3d position = state->_imu->pos();
      pose_time.push_back(stamp);
      pose_x.push_back(position.x());
      pose_y.push_back(position.y());
      pose_z.push_back(position.z());
      if (trajectory_out) {
        trajectory_out << stamp << ' ' << position.x() << ' ' << position.y() << ' ' << position.z() << '\n';
        trajectory_out.flush();
      }
    } else if (i < 3) {
      std::printf("[I] frame %d: %s (%dx%d)\n", static_cast<int>(i), entries[i].path.c_str(), image.cols, image.rows);
      std::fflush(stdout);
    }

    previous_frame_time = stamp;
    if (verbose && (i + 1) % 100 == 0) {
      std::printf("[I] frame %d/%d, %d IMU sample(s) consumed, %d pose(s), %s\n", static_cast<int>(i + 1),
                  static_cast<int>(entries.size()), static_cast<int>(imu_cursor),
                  static_cast<int>(pose_time.size()), manager.initialized() ? "initialised" : "not initialised");
      std::fflush(stdout);
    }
  }
  (void)previous_frame_time;

  std::printf("[I] trajectory: %d pose(s), initialised=%s\n", static_cast<int>(pose_time.size()),
              initialized_at >= 0 ? "yes" : "no");
  if (!pose_time.empty()) {
    std::printf("[I]   first t %.3f  (%.2f, %.2f, %.2f)\n", pose_time.front(), pose_x.front(), pose_y.front(),
                pose_z.front());
    std::printf("[I]   last  t %.3f  (%.2f, %.2f, %.2f)\n", pose_time.back(), pose_x.back(), pose_y.back(),
                pose_z.back());
  }

  // The file has been written pose by pose as the run went; all that is left is to close it and report.  When no
  // pose was ever recorded the file exists but is empty, which is the honest representation of "did not initialise"
  // and is why the return code below distinguishes it.
  if (trajectory_out) {
    trajectory_out.flush();
    trajectory_out.close();
    std::printf("[I] wrote %s\n", out_path.c_str());
  }
  return pose_time.size() >= 2 ? 0 : 1;
}
