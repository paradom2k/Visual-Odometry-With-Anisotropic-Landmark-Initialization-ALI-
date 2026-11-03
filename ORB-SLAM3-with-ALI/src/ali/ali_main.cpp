// ali_run -- one configuration in, one trajectory out.
//
//   ali_run --config conf/fs_mono_inertial.conf
//   ali_run --config conf/fs_stereo.conf --duration 60
//   ali_run --check-only --config <conf>          # parse and report, run nothing
//
// The trajectory goes to `sol/<the conf file's own stem>.txt`, in the four-column shape ORB-SLAM3's own
// examples write (`timestamp tx ty tz qx qy qz qw`), which is what the scorer in `tools/` reads.

#include "ali_conf.h"
#include "ali_files.h"
#include "ali_run.h"

#include "common/log.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

void usage() {
  std::printf(
      "ali_run -- ORB-SLAM3 mono / mono-inertial / stereo / stereo-inertial, driven by a conf file.\n\n"
      "  --config <file>     the configuration to run (required)\n"
      "  --check-only        parse the configuration, report it, and exit without running\n"
      "  --duration <s>      override the configuration's duration (0 = the whole sequence)\n"
      "  --start-frame <n>   override the first frame index\n"
      "  --imu-time-shift <s> override the inertial time shift, seconds\n"
      "  --out <file>        override the trajectory path\n"
      "  --then <file>       after the first run, run this configuration in the SAME process\n"
      "                      (reproduces the UI's run-one-then-another path for cross-run state)\n"
      "  --repo <dir>        the repository root, when the executable is not inside it\n"
      "  --help, -h          this text\n\n"
      "Every other input -- mode, intrinsics, distortion, extrinsics, noise, whether the input is a bag or\n"
      "a directory of images, and which topics a bag is read on -- comes from the configuration file and\n"
      "nowhere else.\n");
}

}  // namespace

int main(int argc, char** argv) {
  jo::Log::instance().setLevel(jo::LogLevel::kInfo);
  jo::Log::instance().setEchoToStderr(true);

  std::string config_path;
  std::vector<std::string> then_paths;
  std::string out_override;
  std::string repo_override;
  double duration_override = -1.0;
  int start_override = -1;
  double time_shift_override = 0.0;
  bool has_time_shift = false;
  double min_parallax_override = -1.0;
  bool check_only = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const bool has_next = (i + 1) < argc;
    if (arg == "--config" && has_next) {
      config_path = argv[++i];
    } else if (arg == "--then" && has_next) {
      then_paths.push_back(argv[++i]);
    } else if (arg == "--out" && has_next) {
      out_override = argv[++i];
    } else if (arg == "--repo" && has_next) {
      repo_override = argv[++i];
    } else if (arg == "--duration" && has_next) {
      duration_override = std::atof(argv[++i]);
    } else if (arg == "--start-frame" && has_next) {
      start_override = std::atoi(argv[++i]);
    } else if (arg == "--imu-time-shift" && has_next) {
      time_shift_override = std::atof(argv[++i]);
      has_time_shift = true;
    } else if (arg == "--min-parallax" && has_next) {
      min_parallax_override = std::atof(argv[++i]);
    } else if (arg == "--check-only") {
      check_only = true;
    } else if (arg == "--help" || arg == "-h") {
      usage();
      return 0;
    } else {
      std::fprintf(stderr, "unknown or incomplete argument: %s\n", arg.c_str());
      usage();
      return 2;
    }
  }
  if (config_path.empty()) {
    usage();
    return 2;
  }

  ali::Conf conf;
  std::string error;
  if (!ali::loadConf(config_path, &conf, &error)) {
    std::fprintf(stderr, "[E] %s: %s\n", config_path.c_str(), error.c_str());
    return 1;
  }
  for (const std::string& warning : conf.warnings) {
    jo::logWarn("ali: " + warning);
  }
  if (duration_override >= 0.0) {
    conf.duration = duration_override;
  }
  if (start_override >= 0) {
    conf.start_frame = start_override;
  }
  if (!out_override.empty()) {
    conf.solution = out_override;
  }
  // The inertial time shift is the one input whose value is a property of the RECORDING rather than of the
  // algorithm -- the offset between the bag's IMU messages and the image stamps -- and measuring it means
  // sweeping it, which means not editing a configuration file per point.  The switch exists for that and
  // the configuration keeps the value that the measurement settles on.
  if (has_time_shift) {
    conf.imu.time_shift = time_shift_override;
  }

  // The monocular initialiser's disparity gate, sweepable for the same reason as the time shift: it is a
  // property of the recording's scenery rather than of the algorithm, and finding it means sweeping it.
  //
  // IT WAS NESTED INSIDE THE TIME-SHIFT BRANCH, so this flag silently did nothing unless `--imu-time-shift`
  // was passed as well -- a sweep of it returns results that look like a sweep and are the configuration's
  // own value repeated.  The two overrides are independent and are written as such.
  if (min_parallax_override >= 0.0) {
    conf.imu.min_initial_parallax = min_parallax_override;
  }

  std::string repo_root = repo_override;
  if (repo_root.empty()) {
    repo_root = ali::repoRootFrom(ali::executablePath());
  }

  std::printf("ali_run: %s (%s)\n", conf.name.empty() ? config_path.c_str() : conf.name.c_str(),
              ali::modeName(conf.mode));
  for (const std::string& line : ali::describeConf(conf)) {
    std::printf("  %s\n", line.c_str());
  }
  std::printf("  solution             %s\n", conf.resolvedSolution(repo_root).c_str());
  if (check_only) {
    std::printf("ali_run: --check-only, nothing was run\n");
    return 0;
  }

  const ali::RunResult result = ali::runConf(conf, repo_root, nullptr, nullptr);
  std::printf("ali_run: %zu frame(s), %zu pose(s), %.1f s wall\n", result.frames, result.poses,
              result.wall_seconds);
  // Where the wall time went.  `decode` is inside `other` when `prefetch_frames` is off (which is the
  // default) and overlaps `track` when it is on, so the label says which rather than asserting one.
  if (result.frames > 0) {
    const double per_frame = result.wall_seconds / static_cast<double>(result.frames);
    const double track = result.seconds_track / static_cast<double>(result.frames);
    const double callback = result.seconds_progress / static_cast<double>(result.frames);
    std::printf("ali_run: %.1f ms/frame -- track %.1f, decode %.1f (%s), callback %.1f, other %.1f\n",
                1000.0 * per_frame, 1000.0 * track,
                1000.0 * result.seconds_decode / static_cast<double>(result.frames),
                result.prefetched ? "overlapped" : "in other", 1000.0 * callback,
                1000.0 * (per_frame - track - callback));
  }
  std::printf("ali_run: trajectory %s\n", result.solution_path.c_str());
  std::printf("ali_run: settings   %s\n", result.settings_path.c_str());
  if (!result.ok) {
    std::fprintf(stderr, "[E] %s\n", result.error.c_str());
    return 1;
  }

  // `--then <conf>`: run a SECOND configuration in this same process.  This is what the UI does when the
  // user runs one sequence and then another, and it is how a difference between "the second run alone"
  // and "the second run after the first" becomes reproducible outside the window.  Anything that lives
  // in a static -- frame and keyframe ids, MapChanged's counter -- survives between the two runs here
  // exactly as it does in the UI.  The overrides that describe the RUN rather than the configuration
  // (duration, start frame) apply to both; the output path does not, so each sequence keeps its own.
  for (const std::string& then_path : then_paths) {
    ali::Conf conf2;
    std::string error2;
    if (!ali::loadConf(then_path, &conf2, &error2)) {
      std::fprintf(stderr, "[E] %s: %s\n", then_path.c_str(), error2.c_str());
      return 1;
    }
    for (const std::string& warning : conf2.warnings) {
      jo::logWarn("ali: " + warning);
    }
    if (duration_override >= 0.0) {
      conf2.duration = duration_override;
    }
    if (start_override >= 0) {
      conf2.start_frame = start_override;
    }
    std::printf("\n[--then] next configuration in the same process: %s (%s)\n",
                conf2.name.empty() ? then_path.c_str() : conf2.name.c_str(), ali::modeName(conf2.mode));
    for (const std::string& line : ali::describeConf(conf2)) {
      std::printf("  %s\n", line.c_str());
    }
    std::printf("  solution             %s\n", conf2.resolvedSolution(repo_root).c_str());
    const ali::RunResult result2 = ali::runConf(conf2, repo_root, nullptr, nullptr);
    std::printf("ali_run[then]: %zu frame(s), %zu pose(s), %.1f s wall\n", result2.frames,
                result2.poses, result2.wall_seconds);
    if (result2.frames > 0) {
      const double per_frame = result2.wall_seconds / static_cast<double>(result2.frames);
      std::printf("ali_run[then]: %.1f ms/frame\n", 1000.0 * per_frame);
    }
    std::printf("ali_run[then]: trajectory %s\n", result2.solution_path.c_str());
    if (!result2.ok) {
      std::fprintf(stderr, "[E] %s\n", result2.error.c_str());
      return 1;
    }
  }
  return 0;
}
