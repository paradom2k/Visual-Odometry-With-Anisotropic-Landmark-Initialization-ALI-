// vins_run -- ROS-free driver for the VINS-Mono estimator.
//
// Reads the same manifest/IMU/configuration schema as the other ALI experiments and writes a TUM
// trajectory.  The estimator core is unmodified except for per-feature whitening (ALI): the default
// ProjectionFactor uses the original global sqrt_info, while an ALI run passes the feature's own
// structure-tensor matrix.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "feature_tracker.h"
#include "ali/ali.h"

#ifdef _WIN32
#include <direct.h>
#endif

namespace {

struct ImageRecord {
  double t = 0.0;
  std::string path;
};

struct ImuRecord {
  double t = 0.0;
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
};

std::string Trim(const std::string& s) {
  std::size_t a = 0;
  std::size_t b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return s.substr(a, b - a);
}

bool ReadManifest(const std::string& path, std::vector<ImageRecord>& images, std::string& error) {
  std::ifstream in(path.c_str());
  if (!in) {
    error = "cannot open manifest " + path;
    return false;
  }
  std::string line;
  while (std::getline(in, line)) {
    line = Trim(line);
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    ImageRecord r;
    if (!(ss >> r.t)) continue;
    std::getline(ss, r.path);
    r.path = Trim(r.path);
    if (!r.path.empty()) images.push_back(r);
  }
  std::sort(images.begin(), images.end(),
            [](const ImageRecord& a, const ImageRecord& b) { return a.t < b.t; });
  return !images.empty();
}

bool ReadImu(const std::string& path, std::vector<ImuRecord>& imu, std::string& error) {
  std::ifstream in(path.c_str());
  if (!in) {
    error = "cannot open IMU csv " + path;
    return false;
  }
  std::string line;
  while (std::getline(in, line)) {
    line = Trim(line);
    if (line.empty() || line[0] == '#') continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream ss(line);
    ImuRecord r;
    if (!(ss >> r.t >> r.gyr.x() >> r.gyr.y() >> r.gyr.z() >> r.acc.x() >> r.acc.y() >> r.acc.z())) {
      continue;
    }
    imu.push_back(r);
  }
  std::sort(imu.begin(), imu.end(), [](const ImuRecord& a, const ImuRecord& b) { return a.t < b.t; });
  return !imu.empty();
}

void AddExperimentNoise(cv::Mat* image, double stamp) {
  static const double sigma = []() {
    const char* s = std::getenv("OV_NOISE");
    return s ? std::atof(s) : 0.0;
  }();
  if (sigma <= 0.0 || image == nullptr || image->empty()) return;
  static const unsigned base_seed = []() {
    const char* s = std::getenv("OV_SEED");
    return static_cast<unsigned>(s ? std::atoi(s) : 0) * 7919u;
  }();
  const unsigned seed = base_seed + static_cast<unsigned>(std::llround(stamp * 1000.0));
  cv::Mat noise(image->size(), CV_16S);
  cv::RNG rng(seed);
  rng.fill(noise, cv::RNG::NORMAL, 0.0, sigma);
  cv::Mat wide;
  image->convertTo(wide, CV_16S);
  wide += noise;
  wide.convertTo(*image, CV_8U);
}

void EnsureParentDirectory(const std::string& path) {
#ifdef _WIN32
  const std::size_t slash = path.find_last_of("/\\");
  if (slash == std::string::npos) return;
  std::string dir = path.substr(0, slash);
  for (std::size_t i = 1; i < dir.size(); ++i) {
    if (dir[i] == '/' || dir[i] == '\\') {
      const std::string sub = dir.substr(0, i);
      _mkdir(sub.c_str());
    }
  }
  _mkdir(dir.c_str());
#endif
}

struct Options {
  std::string config;
  std::string manifest;
  std::string imu;
  std::string out = "sol/vins.txt";
  int frames = 0;
  int start = 0;
  int max_features = -1;
};

bool ParseArgs(int argc, char** argv, Options& opt, std::string& error) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        error = std::string("missing value for ") + name;
        return nullptr;
      }
      return argv[++i];
    };
    if (a == "--config") {
      const char* v = next("--config");
      if (!v) return false;
      opt.config = v;
    } else if (a == "--manifest") {
      const char* v = next("--manifest");
      if (!v) return false;
      opt.manifest = v;
    } else if (a == "--imu") {
      const char* v = next("--imu");
      if (!v) return false;
      opt.imu = v;
    } else if (a == "--out") {
      const char* v = next("--out");
      if (!v) return false;
      opt.out = v;
    } else if (a == "--frames") {
      const char* v = next("--frames");
      if (!v) return false;
      opt.frames = std::atoi(v);
    } else if (a == "--start") {
      const char* v = next("--start");
      if (!v) return false;
      opt.start = std::atoi(v);
    } else if (a == "--max-features") {
      const char* v = next("--max-features");
      if (!v) return false;
      opt.max_features = std::atoi(v);
    } else if (a == "--base-dir") {
      const char* v = next("--base-dir");
      if (!v) return false;
      (void)v;
    } else if (a == "--help" || a == "-h") {
      std::printf("vins_run --config <yaml> --manifest <txt> --imu <csv> --out <tum>\n");
      std::exit(0);
    } else {
      error = "unknown argument " + a;
      return false;
    }
  }
  if (opt.config.empty() || opt.manifest.empty() || opt.imu.empty()) {
    error = "--config, --manifest and --imu are required";
    return false;
  }
  return true;
}

double EnvDouble(const char* name, double fallback) {
  const char* v = std::getenv(name);
  return v ? std::atof(v) : fallback;
}

}  // namespace

int main(int argc, char** argv) {
  cv::setNumThreads(1);
  Options opt;
  std::string error;
  if (!ParseArgs(argc, argv, opt, error)) {
    std::fprintf(stderr, "vins_run: %s\n", error.c_str());
    return 2;
  }

  std::vector<ImageRecord> images;
  std::vector<ImuRecord> imu;
  if (!ReadManifest(opt.manifest, images, error) || !ReadImu(opt.imu, imu, error)) {
    std::fprintf(stderr, "vins_run: %s\n", error.c_str());
    return 2;
  }

  readFeatureParameters(opt.config);
  readParameters(opt.config);
  if (opt.max_features > 0) MAX_CNT = opt.max_features;
  if (MAX_CNT <= 0) MAX_CNT = 150;

  ali::Config ali_config;
  {
    cv::FileStorage fs(opt.config, cv::FileStorage::READ);
    int enabled = 1;
    fs["ali"] >> enabled;
    ali_config.enabled = enabled != 0;
    fs["ali_window"] >> ali_config.window;
    fs["ali_sigma"] >> ali_config.sigma_pix;
    fs["ali_clip_lo"] >> ali_config.clip_lo;
    fs["ali_clip_hi"] >> ali_config.clip_hi;
    fs["ali_ref_alpha"] >> ali_config.ref_alpha;
  }
  std::printf("[vins] ALI %s (window %d, sigma %.2f, clip [%.2f, %.2f])\n",
              ali_config.enabled ? "on" : "off", ali_config.window, ali_config.sigma_pix,
              ali_config.clip_lo, ali_config.clip_hi);

  FeatureTracker tracker;
  tracker.readIntrinsicParameter(opt.config);
  if (!tracker.m_camera) {
    std::fprintf(stderr, "vins_run: camera model was not created from %s\n", opt.config.c_str());
    return 2;
  }
  std::fprintf(stderr, "vins_run: camera model loaded\n");

  Estimator estimator;
  estimator.setParameter();

  EnsureParentDirectory(opt.out);
  std::ofstream trajectory(opt.out.c_str());
  if (!trajectory) {
    std::fprintf(stderr, "vins_run: cannot write %s\n", opt.out.c_str());
    return 2;
  }
  trajectory << "# timestamp x y z qx qy qz qw\n";
  trajectory << std::fixed << std::setprecision(9);

  std::map<int, Eigen::Matrix2d> ali_whitening;
  std::size_t imu_cursor = 0;
  double current_time = -1.0;
  int written = 0;
  int processed = 0;
  int resets = 0;
  const double sigma = EnvDouble("OV_NOISE", 0.0);
  (void)sigma;

  for (std::size_t fi = 0; fi < images.size(); ++fi) {
    if (static_cast<int>(fi) < opt.start) continue;
    if (opt.frames > 0 && processed >= opt.frames) break;
    ++processed;

    cv::Mat image = cv::imread(images[fi].path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
      std::fprintf(stderr, "vins_run: cannot read %s\n", images[fi].path.c_str());
      return 2;
    }
    AddExperimentNoise(&image, images[fi].t);

    tracker.readImage(image, images[fi].t);
    for (unsigned int i = 0;; ++i) {
      bool completed = false;
      completed |= tracker.updateID(i);
      if (!completed) break;
    }

    ali::Frame ali_frame(tracker.cur_img, ali_config, FOCAL_LENGTH);
    for (std::size_t j = 0; j < tracker.ids.size(); ++j) {
      const int id = tracker.ids[j];
      if (ali_whitening.count(id)) continue;
      Eigen::Matrix2d info = (FOCAL_LENGTH / ali_config.sigma_pix) * Eigen::Matrix2d::Identity();
      ali_frame.whiten(tracker.cur_pts[j], &info);
      ali_whitening[id] = info;
    }

    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> feature_frame;
    std::map<int, Eigen::Matrix2d> frame_whitening;
    for (std::size_t j = 0; j < tracker.ids.size(); ++j) {
      if (tracker.track_cnt[j] <= 1) continue;
      const int id = tracker.ids[j];
      Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
      xyz_uv_velocity << tracker.cur_un_pts[j].x, tracker.cur_un_pts[j].y, 1.0,
          tracker.cur_pts[j].x, tracker.cur_pts[j].y,
          tracker.pts_velocity[j].x, tracker.pts_velocity[j].y;
      feature_frame[id].emplace_back(0, xyz_uv_velocity);
      auto it = ali_whitening.find(id);
      if (it != ali_whitening.end()) frame_whitening[id] = it->second;
    }

    const double img_t = images[fi].t;
    // Match the ROS node's publishing rule: every image is tracked, but the estimator is fed at FREQ Hz.
    // The node also drops the very first image because it has no optical-flow velocity yet.
    static double last_publish_time = -1.0;
    if (last_publish_time < 0.0) {
      last_publish_time = img_t;
      continue;
    }
    if (FREQ > 0 && (img_t - last_publish_time) < 1.0 / static_cast<double>(FREQ) - 1e-9) {
      continue;
    }
    last_publish_time = img_t;
    if (feature_frame.empty()) continue;

    while (imu_cursor < imu.size() && imu[imu_cursor].t <= img_t) {
      const ImuRecord& m = imu[imu_cursor];
      if (current_time < 0.0) current_time = m.t;
      const double dt = m.t - current_time;
      if (dt > 0.0) estimator.processIMU(dt, m.acc, m.gyr);
      current_time = m.t;
      ++imu_cursor;
    }
    if (imu_cursor < imu.size() && imu_cursor > 0) {
      const ImuRecord& last = imu[imu_cursor - 1];
      const ImuRecord& next = imu[imu_cursor];
      const double dt1 = img_t - current_time;
      const double dt2 = next.t - img_t;
      if (dt1 > 0.0 && dt2 >= 0.0 && dt1 + dt2 > 0.0) {
        const double w1 = dt2 / (dt1 + dt2);
        const double w2 = dt1 / (dt1 + dt2);
        const Eigen::Vector3d acc = w1 * last.acc + w2 * next.acc;
        const Eigen::Vector3d gyr = w1 * last.gyr + w2 * next.gyr;
        estimator.processIMU(dt1, acc, gyr);
        current_time = img_t;
      }
    }

    std_msgs::Header header;
    header.stamp.t = img_t;
    header.frame_id = "world";
    estimator.processImage(feature_frame, header, ali_config.enabled ? frame_whitening
                                                                   : std::map<int, Eigen::Matrix2d>());

    if (estimator.solver_flag == Estimator::NON_LINEAR) {
      const Eigen::Vector3d& P = estimator.Ps[WINDOW_SIZE];
      Eigen::Quaterniond Q(estimator.Rs[WINDOW_SIZE]);
      Q.normalize();
      trajectory << img_t << ' ' << P.x() << ' ' << P.y() << ' ' << P.z() << ' '
                 << Q.x() << ' ' << Q.y() << ' ' << Q.z() << ' ' << Q.w() << '\n';
      ++written;
    }
    if (estimator.failure_occur) ++resets;

    if (processed % 200 == 0) {
      std::printf("[vins] frame %d/%d  state %s  poses %d  features %zu  reset %d\n",
                  processed, static_cast<int>(images.size()), 
                  estimator.solver_flag == Estimator::NON_LINEAR ? "nonlinear" : "initial",
                  written, feature_frame.size(), resets);
      std::fflush(stdout);
    }
  }

  trajectory.close();
  std::printf("vins_run: %d frame(s), %d pose(s), %d reset(s)\n", processed, written, resets);
  std::printf("vins_run: trajectory %s\n", opt.out.c_str());
  return 0;
}
