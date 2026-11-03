#pragma once

#include <opencv2/core.hpp>

#include "ali_conf.h"

#include <atomic>
#include <string>
#include <vector>

namespace ali {

/// What a run produced, for the log, the UI and the acceptance test.
struct RunResult {
  bool ok = false;
  bool cancelled = false;        ///< ended because `cancel` was raised, not because the frames ran out
  std::string error;
  std::size_t frames = 0;        ///< frames handed to the tracker
  std::size_t poses = 0;         ///< poses the tracker produced (a frame can produce none)
  double wall_seconds = 0.0;
  /// Where the wall time went.  `seconds_decode` is the total the frame reader spent decoding; that work runs
  /// on its own thread and overlaps `seconds_track`, so it is NOT part of the serial per-frame cost -- the
  /// point of the prefetcher is that this number can be as large as it likes as long as it stays under the
  /// tracker's.  `seconds_track` is the tracker's call, which is everything ORB-SLAM3 does on this thread;
  /// the local and loop mapping threads overlap it too.  `seconds_progress` is the caller's callback.
  double seconds_decode = 0.0;
  /// Time the estimator thread spent waiting for images: the prefetch queue on a prefetched run, the
  /// synchronous read/decode otherwise.  On a slow external drive this is what makes wall time exceed
  /// track time, so it is reported separately and cannot be mistaken for estimator cost.
  double seconds_io_wait = 0.0;
  bool prefetched = false;   ///< whether the decode ran on its own thread (see Conf::prefetch_frames)
  double seconds_track = 0.0;
  double seconds_progress = 0.0;
  std::string solution_path;
  std::string settings_path;     ///< the ORB-SLAM3 settings file this run generated
  std::size_t keyframes = 0;
};

/// Write the ORB-SLAM3 settings file the configuration describes.
///
/// This is the bridge between the two worlds: the configuration holds every input in this project's own
/// vocabulary (metres, degrees, SI noise densities, a baseline), and ORB-SLAM3 reads a YAML with its own
/// camera model names, its own `T_b_c1` matrix layout and its own scaling for the distortion vector.  Doing
/// the translation in one function means the run and the UI cannot disagree about it, and it means a
/// configuration that is missing something fails here with the key's name rather than inside Settings.
bool writeSettings(const Conf& conf, const std::string& path, std::string* error);

/// Populate the ORB-SLAM3 settings path a run should use: the configuration's own `settings_out`, or a
/// generated file beside the solution.
std::string settingsPathFor(const Conf& conf, const std::string& repo_root);

/// What the UI needs to draw a run while it is happening.
///
/// The pose is `Twc` in ORB-SLAM3's own world frame -- the same quantity the trajectory file records, but
/// before the writer's "first keyframe at the origin" anchoring is applied.  The two differ by one rigid
/// transform, so the live view's SHAPE is the trajectory's shape; the file is what an evaluation reads,
/// and the UI reloads it when the run ends so the finished picture is the written one.
struct ProgressInfo {
  std::size_t frame = 0;
  std::size_t frames = 0;
  double stamp = 0.0;
  bool tracked = false;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  /// The frame just processed, still owned by the runner.  Null when the caller does not want images,
  /// which is what keeps a headless run from paying for a copy per frame.
  const unsigned char* gray = nullptr;
  int width = 0;
  int height = 0;
  /// The same frame as a refcounted Mat, when the runner has one.  The UI keeps this header (no pixel
  /// copy); the raw `gray` pointer above remains for callers without a Mat.
  cv::Mat gray_mat;
  /// The tracker's own keypoints for this frame, as x,y pairs, when the caller asked for the image (null
  /// otherwise).  These are the undistorted keypoints the estimator actually used, so a view can draw what
  /// the front end was handed instead of running a second ORB extraction per frame -- which is what the UI
  /// did, and what made it 44 % slower than the same run without a view (78 s against 54 s over a 60 s
  /// window).  Owned by the runner and valid only for the duration of the call.
  const float* keypoints_xy = nullptr;
  std::size_t keypoint_count = 0;
  /// The right-camera frame and its feature overlay, when the mode has a stereo pair.  Same ownership
  /// rules as the left frame above; empty for monocular modes.
  const unsigned char* gray_right = nullptr;
  int width_right = 0;
  int height_right = 0;
  cv::Mat gray_right_mat;
  const float* keypoints_right_xy = nullptr;
  std::size_t keypoint_right_count = 0;
  /// The LiDAR local cloud around the vehicle: x,y,z triples and one intensity per point, both owned by
  /// the runner and valid only for the duration of the call.  Drawn under the trajectory.
  const float* cloud_xyz = nullptr;
  const float* cloud_intensity = nullptr;
  std::size_t cloud_count = 0;
  /// Bumped when the runner rebuilds the cloud.  The same buffer is handed over on the scans in between,
  /// and a view that wants to ACCUMULATE a whole-map copy has to be able to tell one rebuild from the
  /// next -- otherwise it re-inserts every point ten times a second.
  std::size_t cloud_generation = 0;
};

/// Run one configuration end to end: frames in, trajectory out.
///
/// `progress`, when given, is called after every frame.  It runs on the calling thread, so a UI that wants
/// to draw has to marshal it itself -- the alternative, calling back from another thread, would make every
/// drawing call's thread-safety the caller's problem.
///
/// `cancel`, when given, is polled once per frame.  Raising it ends the run at the next frame boundary: the
/// estimator is shut down, the poses collected so far are written as usual, and the result says
/// `cancelled`.  It exists so the UI's STOP can end a drive without killing the process, and it is an
/// atomic because the thread that raises it is not this one.  A null pointer means "run to the end", which
/// is what every command-line run wants.
RunResult runConf(const Conf& conf, const std::string& repo_root,
                  void (*progress)(const ProgressInfo&, void*) = nullptr,
                  void* progress_user = nullptr,
                  const std::atomic<bool>* cancel = nullptr);

/// The sensor constant ORB-SLAM3 wants for the mode.
int sensorFor(Mode mode);

}  // namespace ali
