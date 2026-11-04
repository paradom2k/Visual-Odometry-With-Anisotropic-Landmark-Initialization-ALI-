#include "ali_sensors.h"

#include "ali_files.h"

#include "data/bag_reader.h"

#include "common/log.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

namespace ali {
namespace {

/// The directory or manifest a camera's frames come from, resolved from the configuration.
/// Returned as a pair so that "which of the three keys is in use" is answered once.
bool imageSourceOf(const Conf& conf, bool right, std::string* directory, std::string* manifest,
                   std::string* error) {
  // The right camera needs its own directory.  Conventions, in order: an explicit `images_right_dir`, else
  // the left directory with `cam0` swapped for `cam1` (the 4Seasons extraction's layout), else the same
  // directory -- which would be wrong and is therefore refused rather than silently run.
  if (!right) {
    *directory = conf.images_dir.empty() ? conf.images : conf.images_dir;
    *manifest = conf.images_manifest;
    return true;
  }
  *directory = conf.images_right_dir.empty() ? conf.images_right : conf.images_right_dir;
  *manifest = conf.images_manifest_right;
  // EITHER source is enough.  This used to require both, so naming `images_right_dir` without also naming a
  // manifest fell through to the derive-from-the-left branch below -- and a left directory with no `cam0` in
  // its name then failed with "name `images_right_dir`" while that very key was set.  The 4Seasons sets never
  // exposed it: their left directory is `.../cam0`, so the derivation succeeds and this line is not reached.
  if (!directory->empty() || !manifest->empty()) {
    return true;
  }
  // Derive from the left.
  const std::string left_dir = conf.images_dir.empty() ? conf.images : conf.images_dir;
  if (left_dir.empty()) {
    if (error != nullptr) {
      *error = "the stereo mode needs the right camera's images: name `images_right_dir` (or a left "
               "directory whose name contains `cam0`, which is swapped for `cam1`)";
    }
    return false;
  }
  const std::size_t at = left_dir.find("cam0");
  if (at == std::string::npos) {
    if (error != nullptr) {
      *error = "cannot derive the right camera's directory from " + left_dir +
               " (no `cam0` in it) -- name `images_right_dir`";
    }
    return false;
  }
  *directory = left_dir.substr(0, at) + "cam1" + left_dir.substr(at + 4);
  return true;
}

}  // namespace

/// Join a directory and a file name without caring which separator the configuration used.
static std::string joinPath(const std::string& directory, const std::string& name) {
  if (directory.empty()) {
    return name;
  }
  const char last = directory.back();
  if (last == '/' || last == '\\') {
    return directory + name;
  }
  return directory + "/" + name;
}

/// Load the sidecar index the packer writes next to its bags.
///
/// Walking a bag to discover its timestamps costs a full pass over it; for a packed sequence that is the
/// whole sequence.  The index is one line per frame (`stamp bag-file message-index`) and turns startup into
/// a few file reads.  `found` distinguishes "there is no index, fall back to walking" from a malformed one,
/// which is an error rather than a reason to silently do the expensive thing.
static bool loadFrameIndex(const Conf& conf, bool right, std::vector<FrameRef>* out, bool* found,
                           std::string* error) {
  *found = false;
  const bool directory = ali::isDirectory(conf.bag);
  const std::string index_path =
      directory ? joinPath(conf.bag, right ? "right.idx" : "left.idx")
                : conf.bag + (right ? ".right.idx" : ".left.idx");
  if (!ali::isFile(index_path)) {
    return true;
  }
  std::ifstream in(index_path);
  if (!in) {
    if (error != nullptr) {
      *error = "cannot open bag index " + index_path;
    }
    return false;
  }
  std::string line;
  if (!std::getline(in, line) || line != "# ali stereo bag index v1") {
    if (error != nullptr) {
      *error = "unrecognised bag index header in " + index_path;
    }
    return false;
  }
  std::vector<FrameRef> frames;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream row(line);
    double stamp = 0.0;
    std::string bag_name;
    long long index = -1;
    if (!(row >> stamp >> bag_name >> index) || index < 0) {
      if (error != nullptr) {
        *error = "malformed bag index line in " + index_path + ": " + line;
      }
      return false;
    }
    FrameRef ref;
    ref.stamp = stamp;
    ref.bag_index = static_cast<int>(index);
    ref.bag = directory ? joinPath(conf.bag, bag_name) : conf.bag;
    ref.topic = right ? conf.bag_image_topic_right : conf.bag_image_topic;
    frames.push_back(ref);
  }
  if (frames.empty()) {
    if (error != nullptr) {
      *error = "empty bag index " + index_path;
    }
    return false;
  }
  // Same monotonic rule the bag walk applies: two frames with the same stamp would hand the tracker a
  // frame older than its predecessor.
  std::sort(frames.begin(), frames.end(),
            [](const FrameRef& a, const FrameRef& b) { return a.stamp < b.stamp; });
  std::vector<FrameRef> sorted;
  sorted.reserve(frames.size());
  for (const FrameRef& frame : frames) {
    if (sorted.empty() || frame.stamp > sorted.back().stamp) {
      sorted.push_back(frame);
    }
  }
  out->swap(sorted);
  *found = true;
  return true;
}

static void applyBagWindow(const Conf& conf, std::vector<FrameRef>* frames) {
  const std::size_t first = conf.bag_start < 0 ? 0u : static_cast<std::size_t>(conf.bag_start);
  const std::size_t count = conf.bag_frames > 0 ? static_cast<std::size_t>(conf.bag_frames) : 0u;
  if (first == 0 && count == 0) {
    return;
  }
  std::vector<FrameRef> window;
  for (std::size_t i = first; i < frames->size() && (count == 0 || window.size() < count); ++i) {
    window.push_back((*frames)[i]);
  }
  frames->swap(window);
}

/// The bags a configuration names: the file itself, or every `.bag` in a directory, in playback order.
///
/// These recordings ship as thirty-six one-minute bags and a trajectory needs all of them, so the source is
/// allowed to be a directory rather than a single file.  The order is the bag's own index and not the
/// alphabet's -- `_10_` sorts before `_2_` -- which is what `listBagFiles` exists for.
static bool bagsFor(const std::string& path, std::vector<std::string>* out, std::string* error) {
  out->clear();
  if (ali::isDirectory(path)) {
    return ali::listBagFiles(path, out, error);
  }
  out->push_back(path);
  return true;
}

/// Every frame of `topic` across `bags`, in order, each tagged with the bag and topic it came from.
static bool framesAcrossBags(const std::vector<std::string>& bags, const std::string& topic,
                             std::vector<FrameRef>* out, std::string* error) {
  std::string first_error;
  int empty_bags = 0;
  for (std::size_t b = 0; b < bags.size(); ++b) {
    std::vector<jo::ImageFrame> frames;
    // A bag that holds nothing on this topic is SKIPPED, not fatal.  These downloads carry one bag per
    // minute of wall clock and the recording stops before the last minute is up, so the final bag is a
    // 4 KB stub with no connections at all -- and failing on it loses the other thirty-five, which is a
    // whole drive thrown away over an empty file.  The first error is kept and reported only if no bag
    // yields anything, so a genuinely wrong topic name still says so.
    std::string bag_error;
    if (!jo::BagReader::readImages(bags[b], topic, 0, 0, &frames, &bag_error)) {
      if (first_error.empty()) first_error = bag_error;
      ++empty_bags;
      continue;
    }
    if (frames.empty()) {
      ++empty_bags;
      continue;
    }
    for (std::size_t i = 0; i < frames.size(); ++i) {
      FrameRef ref;
      ref.stamp = frames[i].time;
      ref.bag_index = static_cast<int>(i);
      ref.bag = bags[b];
      ref.topic = topic;
      out->push_back(ref);
    }
  }
  // Two bags can share a boundary frame; a stamp that does not advance would be handed to the tracker as a
  // frame older than the one before it, which ORB-SLAM3 answers by discarding its map.  Only strictly
  // increasing stamps survive.
  std::vector<FrameRef> sorted;
  sorted.reserve(out->size());
  for (std::size_t i = 0; i < out->size(); ++i) {
    if (sorted.empty() || (*out)[i].stamp > sorted.back().stamp) {
      sorted.push_back((*out)[i]);
    }
  }
  out->swap(sorted);
  if (out->empty()) {
    if (error != nullptr) {
      *error = first_error.empty() ? (topic + ": no bag in the source yielded a frame") : first_error;
    }
    return false;
  }
  if (empty_bags > 0) {
    jo::logWarnF("ali: %d of %d bag(s) in the source held nothing on %s", empty_bags,
                 static_cast<int>(bags.size()), topic.c_str());
  }
  return true;
}

bool collectFrames(const Conf& conf, std::vector<FrameRef>* out, std::string* error) {
  out->clear();
  if (conf.image_source == ImageSource::kBag) {
    bool indexed = false;
    if (!loadFrameIndex(conf, false, out, &indexed, error)) {
      return false;
    }
    if (indexed) {
      applyBagWindow(conf, out);
      return !out->empty();
    }
    std::vector<std::string> bags;
    if (!bagsFor(conf.bag, &bags, error)) {
      return false;
    }
    if (bags.size() > 1 || !conf.bag_start || conf.bag_frames) {
      // Several bags, or a window asked for: read them all and take the window here, because the reader's
      // own index counts inside one bag and there is no longer one bag for it to count in.
      if (!framesAcrossBags(bags, conf.bag_image_topic, out, error)) {
        return false;
      }
      const std::size_t first = conf.bag_start < 0 ? 0u : static_cast<std::size_t>(conf.bag_start);
      if (first > 0 || conf.bag_frames > 0) {
        const std::size_t count = conf.bag_frames > 0 ? static_cast<std::size_t>(conf.bag_frames) : 0u;
        std::vector<FrameRef> window;
        for (std::size_t i = first; i < out->size() && (count == 0 || window.size() < count); ++i) {
          window.push_back((*out)[i]);
        }
        out->swap(window);
      }
      return !out->empty();
    }
    std::vector<jo::ImageFrame> frames;
    if (!jo::BagReader::readImages(conf.bag, conf.bag_image_topic, 0, 0, &frames, error)) {
      return false;
    }
    for (std::size_t i = 0; i < frames.size(); ++i) {
      FrameRef ref;
      ref.stamp = frames[i].time;
      ref.bag_index = static_cast<int>(i);
      ref.bag = conf.bag;
      ref.topic = conf.bag_image_topic;
      out->push_back(ref);
    }
    return !out->empty();
  }

  std::string directory;
  std::string manifest;
  if (!imageSourceOf(conf, false, &directory, &manifest, error)) {
    return false;
  }
  if (!manifest.empty()) {
    std::vector<ManifestRow> rows;
    if (!readManifest(manifest, std::string(), &rows, error)) {
      return false;
    }
    for (const ManifestRow& row : rows) {
      FrameRef ref;
      ref.stamp = row.stamp;
      ref.path = row.path;
      out->push_back(ref);
    }
  } else {
    std::vector<std::string> files;
    if (!listImages(directory, &files, error)) {
      return false;
    }
    bool timestamps_look_real = true;
    for (const std::string& file : files) {
      FrameRef ref;
      bool looked_like_time = false;
      const double stamp = stampFromFileName(file, &looked_like_time);
      if (!looked_like_time) {
        timestamps_look_real = false;
      }
      ref.stamp = stamp;
      ref.path = file;
      out->push_back(ref);
    }
    if (!timestamps_look_real) {
      // Sequential names carry no clock, so the frame rate does -- and if it does not either, saying so is
      // better than inventing a time base that the inertial factors would then be integrated against.
      if (conf.camera.fps > 0.0) {
        for (std::size_t i = 0; i < out->size(); ++i) {
          (*out)[i].stamp = static_cast<double>(i) / conf.camera.fps;
        }
      } else if (error != nullptr) {
        *error = "the image names carry no timestamp and `camera_fps` is not set, so the frames have no "
                 "clock";
        return false;
      }
    }
  }
  // A bag index is meaningless for a directory source; keep it -1 so `readFrame` knows where to look.
  return !out->empty();
}

bool collectFramesRight(const Conf& conf, std::vector<FrameRef>* out, std::string* error) {
  out->clear();
  if (conf.image_source == ImageSource::kBag) {
    bool indexed = false;
    if (!loadFrameIndex(conf, true, out, &indexed, error)) {
      return false;
    }
    if (indexed) {
      applyBagWindow(conf, out);
      return !out->empty();
    }
    std::vector<std::string> bags;
    if (!bagsFor(conf.bag, &bags, error)) {
      return false;
    }
    if (bags.size() > 1 || !conf.bag_start || conf.bag_frames) {
      // Several bags, or a window asked for: read them all and take the window here, because the reader's
      // own index counts inside one bag and there is no longer one bag for it to count in.
      if (!framesAcrossBags(bags, conf.bag_image_topic_right, out, error)) {
        return false;
      }
      const std::size_t first = conf.bag_start < 0 ? 0u : static_cast<std::size_t>(conf.bag_start);
      if (first > 0 || conf.bag_frames > 0) {
        const std::size_t count = conf.bag_frames > 0 ? static_cast<std::size_t>(conf.bag_frames) : 0u;
        std::vector<FrameRef> window;
        for (std::size_t i = first; i < out->size() && (count == 0 || window.size() < count); ++i) {
          window.push_back((*out)[i]);
        }
        out->swap(window);
      }
      return !out->empty();
    }
    std::vector<jo::ImageFrame> frames;
    if (!jo::BagReader::readImages(conf.bag, conf.bag_image_topic_right, 0, 0, &frames, error)) {
      return false;
    }
    for (std::size_t i = 0; i < frames.size(); ++i) {
      FrameRef ref;
      ref.stamp = frames[i].time;
      ref.bag_index = static_cast<int>(i);
      ref.bag = conf.bag;
      ref.topic = conf.bag_image_topic_right;
      out->push_back(ref);
    }
    return !out->empty();
  }

  std::string directory;
  std::string manifest;
  if (!imageSourceOf(conf, true, &directory, &manifest, error)) {
    return false;
  }
  if (!manifest.empty()) {
    std::vector<ManifestRow> rows;
    if (!readManifest(manifest, std::string(), &rows, error)) {
      return false;
    }
    for (const ManifestRow& row : rows) {
      FrameRef ref;
      ref.stamp = row.stamp;
      ref.path = row.path;
      out->push_back(ref);
    }
  } else {
    std::vector<std::string> files;
    if (!listImages(directory, &files, error)) {
      return false;
    }
    bool timestamps_look_real = true;
    for (const std::string& file : files) {
      FrameRef ref;
      bool looked_like_time = false;
      const double stamp = stampFromFileName(file, &looked_like_time);
      if (!looked_like_time) {
        timestamps_look_real = false;
      }
      ref.stamp = stamp;
      ref.path = file;
      out->push_back(ref);
    }
    if (!timestamps_look_real && conf.camera.fps > 0.0) {
      for (std::size_t i = 0; i < out->size(); ++i) {
        (*out)[i].stamp = static_cast<double>(i) / conf.camera.fps;
      }
    }
  }
  return !out->empty();
}

/// One bag's frames, kept between calls.
///
/// `BagReader::readImages` loads the WHOLE bag and walks it to the message asked for, so fetching one frame
/// at a time costs a bag read per frame -- measured at 0.45 s/frame on a 460 MB SMART_PNT bag, which is
/// three hours for a thirty-six-bag drive.  The frames are consumed in order, so one bag is current at a
/// time and holding its frames turns that into one read per bag.  The cache is a single slot on purpose:
/// thirty-six bags will not fit in memory together, and the next bag is the only one that can be asked for.
struct BagFrameCacheEntry {
  std::string bag;
  std::string topic;
  std::vector<jo::ImageFrame> frames;
};

/// Two slots, not one.  A stereo bag holds left and right under different topics, and the reader
/// alternates between them on every frame; a single slot would throw one side away and walk the whole bag
/// again each frame.  The bags are chunked (a few hundred frames), so two topics of one chunk bound the
/// memory instead of holding a whole sequence.
static std::vector<BagFrameCacheEntry> g_bag_cache;

/// The frame at `index` of `topic` in `bag`, loading the bag if it is not already in hand.
static bool bagFrameAt(const std::string& bag, const std::string& topic, std::size_t index,
                       const jo::ImageFrame** out, std::string* error) {
  for (std::size_t i = 0; i < g_bag_cache.size(); ++i) {
    if (g_bag_cache[i].bag == bag && g_bag_cache[i].topic == topic) {
      if (index >= g_bag_cache[i].frames.size()) {
        if (error != nullptr) {
          *error = "the bag " + bag + " holds " + std::to_string(g_bag_cache[i].frames.size()) +
                   " frame(s) on " + topic + ", and frame " + std::to_string(index) + " was asked for";
        }
        return false;
      }
      *out = &g_bag_cache[i].frames[index];
      return true;
    }
  }

  if (g_bag_cache.size() >= 2) {
    g_bag_cache.erase(g_bag_cache.begin());
  }
  BagFrameCacheEntry entry;
  entry.bag = bag;
  entry.topic = topic;
  if (!jo::BagReader::readImages(bag, topic, 0, 0, &entry.frames, error)) {
    return false;
  }
  if (index >= entry.frames.size()) {
    if (error != nullptr) {
      *error = "the bag " + bag + " holds " + std::to_string(entry.frames.size()) +
               " frame(s) on " + topic + ", and frame " + std::to_string(index) + " was asked for";
    }
    return false;
  }
  g_bag_cache.push_back(std::move(entry));
  *out = &g_bag_cache.back().frames[index];
  return true;
}


/// Optional image-noise realization shared by the ALI experiment.  The seed is per
/// (configuration seed, frame stamp), so a given seed reproduces the same noise for every arm.
void addExperimentNoise(cv::Mat *image, double stamp) {
  static const double sigma = []() {
    const char *s = std::getenv("OV_NOISE");
    return s ? std::atof(s) : 0.0;
  }();
  if (sigma <= 0.0 || image == nullptr || image->empty()) return;
  static const unsigned base_seed = []() {
    const char *s = std::getenv("OV_SEED");
    return (unsigned)(s ? std::atoi(s) : 0) * 7919u;
  }();
  const unsigned seed = base_seed + (unsigned)std::llround(stamp * 1000.0);
  cv::Mat noise(image->size(), CV_16S);
  cv::RNG rng(seed);
  rng.fill(noise, cv::RNG::NORMAL, 0.0, sigma);
  cv::Mat wide;
  image->convertTo(wide, CV_16S);
  wide += noise;
  wide.convertTo(*image, CV_8U);
}

bool readFrame(const Conf& conf, const FrameRef& frame, cv::Mat* image, std::string* error) {
  if (frame.bag_index >= 0) {
    // A bag frame is fetched on its own so that a run which steps forward does not re-walk the file.  The
    // reader's `readImages` is the only way in; asking it for one frame at this index is exactly the
    // "count and drop without copying" path it documents.
    // The topic travels on the ref: a stereo bag holds both cameras and the right frames must not read the
    // left topic.  The fallbacks keep older callers (and single-camera bags) working.
    const std::string topic = !frame.topic.empty()
                                  ? frame.topic
                                  : (!frame.path.empty() ? frame.path : conf.bag_image_topic);
    const std::string& source = frame.bag.empty() ? conf.bag : frame.bag;
    const jo::ImageFrame* one = nullptr;
    if (!bagFrameAt(source, topic, static_cast<std::size_t>(frame.bag_index), &one, error)) {
      return false;
    }
    // Two payload shapes, and the reader says which: `sensor_msgs/Image` arrives as a raster whose row
    // stride is the message's own, `sensor_msgs/CompressedImage` as a JPEG.  Getting this wrong does not
    // crash -- it produces a plausible-looking garbled image -- so the flag is honoured rather than guessed.
    const jo::ImageFrame& frame_in_bag = *one;
    if (frame_in_bag.raw_raster) {
      const int step = frame_in_bag.step > 0 ? frame_in_bag.step : frame_in_bag.width;
      if (frame_in_bag.payload.size() <
          static_cast<std::size_t>(step) * static_cast<std::size_t>(frame_in_bag.height)) {
        if (error != nullptr) {
          *error = "the bag's raster at index " + std::to_string(frame.bag_index) +
                   " is shorter than its own geometry";
        }
        return false;
      }
      cv::Mat view(frame_in_bag.height, frame_in_bag.width, CV_8UC1,
                   const_cast<std::uint8_t*>(frame_in_bag.payload.data()),
                   static_cast<std::size_t>(step));
      view.copyTo(*image);
    } else {
      const cv::Mat encoded(1, static_cast<int>(frame_in_bag.payload.size()), CV_8UC1,
                            const_cast<std::uint8_t*>(frame_in_bag.payload.data()));
      *image = cv::imdecode(encoded, cv::IMREAD_GRAYSCALE);
    }
    if (image->empty() && error != nullptr) {
      *error = "the bag's message at index " + std::to_string(frame.bag_index) + " did not decode";
    }
    addExperimentNoise(image, frame.stamp);
    return !image->empty();
  }
  *image = cv::imread(frame.path, cv::IMREAD_GRAYSCALE);
  if (image->empty() && error != nullptr) {
    *error = "cannot read " + frame.path;
  }
  addExperimentNoise(image, frame.stamp);
  return !image->empty();
}

/// Scale factors taking whatever the source writes into SI, which is what the estimator works in.
///
/// The two units are separate settings rather than one: a recording can be metric in one column and not
/// the other, and the failure of getting the angular one wrong is the one this project has already paid
/// for -- a gyroscope read as deg/s when it is rad/s shrinks it 57x, the pre-integration then reports
/// almost no rotation while the camera watches the car turn, and the visual-inertial initialisation locks
/// a scale that is wrong by whatever the ratio was.
///
/// The `/sample` units multiply by the configured frequency, because an increment over one sample becomes a
/// rate when it is divided by the sample period.  That makes the conversion depend on `imu_frequency`
/// being right, which is why the configuration refuses a per-sample unit without one rather than assuming.
void imuUnitScales(const ImuConf& imu, double* gyro_scale, double* accel_scale) {
  const double kDegToRad = 3.14159265358979323846 / 180.0;
  const double kG = 9.80665;
  const double rate = imu.frequency > 0.0 ? imu.frequency : 1.0;
  switch (imu.angular_unit) {
    case ImuConf::AngularUnit::kRadPerSec:    *gyro_scale = 1.0; break;
    case ImuConf::AngularUnit::kDegPerSec:    *gyro_scale = kDegToRad; break;
    case ImuConf::AngularUnit::kRadPerSample: *gyro_scale = rate; break;
    case ImuConf::AngularUnit::kDegPerSample: *gyro_scale = kDegToRad * rate; break;
  }
  switch (imu.accel_unit) {
    case ImuConf::AccelUnit::kMetresPerSec2:        *accel_scale = 1.0; break;
    case ImuConf::AccelUnit::kG:                    *accel_scale = kG; break;
    case ImuConf::AccelUnit::kMetresPerSecPerSample: *accel_scale = rate; break;
    case ImuConf::AccelUnit::kGPerSample:           *accel_scale = kG * rate; break;
  }
}

bool collectImu(const Conf& conf, std::vector<ImuSample>* out, std::string* error) {
  out->clear();
  if (conf.imu_source == ImuSource::kNone) {
    return true;
  }
  double gyro_scale = 1.0;
  double accel_scale = 1.0;
  imuUnitScales(conf.imu, &gyro_scale, &accel_scale);

  if (conf.imu_source == ImuSource::kFile) {
    std::ifstream in(conf.imu_file);
    if (!in) {
      if (error != nullptr) {
        *error = "cannot open the inertial file " + conf.imu_file;
      }
      return false;
    }
    const std::string kAngular =
        conf.imu.angular_unit == ImuConf::AngularUnit::kDegPerSec ? "deg/s"
        : conf.imu.angular_unit == ImuConf::AngularUnit::kRadPerSample ? "rad/sample"
        : conf.imu.angular_unit == ImuConf::AngularUnit::kDegPerSample ? "deg/sample" : "rad/s";
    const std::string kAccel =
        conf.imu.accel_unit == ImuConf::AccelUnit::kG ? "g"
        : conf.imu.accel_unit == ImuConf::AccelUnit::kMetresPerSecPerSample ? "m/s per sample"
        : conf.imu.accel_unit == ImuConf::AccelUnit::kGPerSample ? "g per sample" : "m/s^2";
    std::size_t reported = 0;
    std::string line;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      // Commas OR whitespace.  This reader originally split on commas only, and the 4Seasons files are
      // space separated, so every row parsed as a single field and was skipped -- which surfaced as "the
      // inertial file holds no usable rows" for a file that is entirely usable.
      std::vector<double> values;
      std::string field;
      std::string normalised = line;
      for (std::size_t i = 0; i < normalised.size(); ++i) {
        if (normalised[i] == ',') {
          normalised[i] = ' ';
        }
      }
      std::stringstream parser(normalised);
      while (parser >> field) {
        // A header line without a leading '#' ("Time Gyro_X ...", campus04) parses as seven fields whose
        // atof() is zero; accepting it pushed a fake t=0, a=0 sample, and the LiDAR opening then saw a zero
        // gravity vector.  Only fully numeric fields are data.
        char* end = nullptr;
        const double value = std::strtod(field.c_str(), &end);
        if (end == field.c_str() || *end != '\0') {
          values.clear();
          break;
        }
        values.push_back(value);
      }
      if (values.size() < 7) {
        continue;
      }
      ImuSample sample;
      // The stamps in these files are nanoseconds, and everything else in the project is in seconds --
      // the frame stamps are read out of file names as 1602075232.933.  Taking the column as seconds
      // directly puts the inertial stream 1.6e9 seconds away from the images, which reads as "no IMU
      // samples for this frame" rather than as a unit error.  A stamp is treated as nanoseconds when it
      // is too large to be a plausible second count.
      sample.stamp = values[0] > 1.0e12 ? values[0] * 1.0e-9 : values[0];
      sample.gyro = cv::Point3f(static_cast<float>(values[1] * gyro_scale),
                                static_cast<float>(values[2] * gyro_scale),
                                static_cast<float>(values[3] * gyro_scale));
      sample.accel = cv::Point3f(static_cast<float>(values[4] * accel_scale),
                                 static_cast<float>(values[5] * accel_scale),
                                 static_cast<float>(values[6] * accel_scale));
      out->push_back(sample);
      if (reported < 2) {
        ++reported;
        jo::logInfoF("ali: imu sample %.3f  file  gyro (%.8f, %.8f, %.8f) rad/s  accel (%.6f, %.6f, %.6f) "
                     "|%.4f| m/s^2   (read as %s and %s)",
                     sample.stamp, sample.gyro.x, sample.gyro.y, sample.gyro.z, sample.accel.x,
                     sample.accel.y, sample.accel.z,
                     std::sqrt(sample.accel.x * sample.accel.x + sample.accel.y * sample.accel.y +
                               sample.accel.z * sample.accel.z),
                     kAngular.c_str(), kAccel.c_str());
      }
    }
    if (out->empty() && error != nullptr) {
      *error = "the inertial file " + conf.imu_file + " holds no usable rows (want `t gx gy gz ax ay az`)";
    }
    return !out->empty();
  }

  // A bag.  `sensor_msgs/Imu` is decoded here rather than in the bag reader, because the reader's job is the
  // container and this is the one message type this project needs out of it.
  //
  // THE OFFSETS ARE NOT CONSTANTS.  `sensor_msgs/Imu` is
  //     Header{ uint32 seq; uint32 sec; uint32 nsec; string frame_id }
  //     Quaternion orientation; float64[9] orientation_covariance;
  //     Vector3 angular_velocity; float64[9] angular_velocity_covariance;
  //     Vector3 linear_acceleration; float64[9] linear_acceleration_covariance
  // and ROS serialises a string as a 4-byte length followed by that many bytes with NO padding.  So the
  // offset of `angular_velocity` is 16 + strlen(frame_id) + 104 and of `linear_acceleration` 96 further on,
  // and a reader that assumes a 16-byte header -- as the first version of this function did, with the two
  // constants 32 and 80 -- decodes three consecutive members of `orientation_covariance` as the gyroscope
  // and three more as the accelerometer.  The values are plausible numbers of the right magnitude, so the
  // run starts and diverges a minute later with a NaN half-way through an SO3 exponential; nothing in the
  // log names the decode.
  const std::string path = conf.imu_bag.empty() ? conf.bag : conf.imu_bag;
  std::size_t delivered = 0;
  std::size_t reported = 0;
  const bool ok = jo::BagReader::readRaw(
      path, conf.imu_topic, 0,
      [out, &reported, gyro_scale, accel_scale](double stamp, const std::uint8_t* payload, std::size_t size) {
        if (size < 16) {
          return true;
        }
        std::uint32_t frame_id_length = 0;
        std::memcpy(&frame_id_length, payload + 12, sizeof(frame_id_length));
        const std::size_t base = 16 + frame_id_length;
        const std::size_t gyro_at = base + 32 + 72;   // orientation, then its covariance
        const std::size_t accel_at = gyro_at + 24 + 72;
        if (accel_at + 24 > size) {
          return true;
        }
        double gyro[3];
        double accel[3];
        std::memcpy(gyro, payload + gyro_at, 24);
        std::memcpy(accel, payload + accel_at, 24);
        // RADIANS PER SECOND.  This recording's gyroscope is already SI, and an earlier version of this
        // function multiplied it by pi/180 on the strength of a comparison against another project's
        // conversion of the same bag -- which is circular, because that value had been through the same
        // conversion.  Three checks settle it, and all three were done against the data rather than against
        // anyone's output:
        //
        //   1. `/imu_raw` and the recording's own `imu.txt` are bit-identical on all 626881 samples
        //      (tools/compare_imu_sources.py).  The units question is therefore about imu.txt, not the bag.
        //   2. Magnitude.  The gyro peaks at 0.594.  Read as rad/s that is 34.1 deg/s, which is a normal
        //      hard turn on a public road; read as deg/s it is 0.59 deg/s, which would mean the car never
        //      rotated more than a third of a degree per second anywhere in a drive with several 90-degree
        //      corners.
        //   3. Integral.  Integrating |gyro| over the drive gives 1709.6 deg read as rad/s and 29.8 deg read
        //      as deg/s.  The reference trajectory's own total heading change over the same drive is
        //      1316.9 deg, so rad/s is right and deg/s is off by a factor of 44.
        //
        // A gyro scaled down by 57 is not a small error: the pre-integration then reports almost no
        // rotation while the camera sees the car turning, and the visual-inertial initialisation absorbs
        // the disagreement as a bias at best and as a wrong scale at worst -- which is what a 530x
        // oversized stereo-inertial map looks like.
        // The unit scale comes from the configuration, so the same key governs the file source and the bag:
        // this recording's raw values are rad/s and m/s^2, which is the default, and the checks above are
        // what established that.  A run that names deg/s gets the conversion applied here rather than being
        // silently mis-scaled, which is the failure this project already made once.
        ImuSample sample;
        sample.stamp = stamp;
        sample.gyro = cv::Point3f(static_cast<float>(gyro[0] * gyro_scale),
                                  static_cast<float>(gyro[1] * gyro_scale),
                                  static_cast<float>(gyro[2] * gyro_scale));
        sample.accel = cv::Point3f(static_cast<float>(accel[0] * accel_scale),
                                   static_cast<float>(accel[1] * accel_scale),
                                   static_cast<float>(accel[2] * accel_scale));
        out->push_back(sample);
        // The first sample, printed.  A decode that is wrong by a fixed offset cannot be told from a
        // correct one by any summary statistic -- both give numbers of the right magnitude -- so the one
        // thing that settles it is looking at a sample whose value is known: this recording is at rest at
        // its start, and a unit at rest reads about 9.8 m/s^2 of specific force and a fraction of a degree
        // per second of rotation.
        if (reported < 3) {
          ++reported;
          // The CONVERTED values, not the raw ones.  Printing the raw arrays made this line useless for
          // the one thing it exists to show: whether the unit conversion ran.  A per-sample stream scaled by
          // 200 and one left alone looked identical here.
          jo::logInfoF("ali: imu sample %.3f  frame_id %u byte(s)  accel (%.6f, %.6f, %.6f) |%.4f| m/s^2  "
                       "gyro (%.8f, %.8f, %.8f) rad/s   (x%.4g gyro, x%.4g accel)",
                       sample.stamp, frame_id_length, sample.accel.x, sample.accel.y, sample.accel.z,
                       std::sqrt(sample.accel.x * sample.accel.x + sample.accel.y * sample.accel.y +
                                 sample.accel.z * sample.accel.z),
                       sample.gyro.x, sample.gyro.y, sample.gyro.z, gyro_scale, accel_scale);
        }
        return true;
      },
      &delivered, error);
  if (!ok) {
    return false;
  }
  if (out->empty() && error != nullptr) {
    *error = "the bag " + path + " delivered no message on `" + conf.imu_topic + "`";
  }
  return !out->empty();
}

double lastStamp(const std::vector<ImuSample>& samples) {
  return samples.empty() ? 0.0 : samples.back().stamp;
}

}  // namespace ali
