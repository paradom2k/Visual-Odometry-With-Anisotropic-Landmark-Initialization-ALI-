#pragma once

// The configuration file: everything a run needs, in one place.
//
// The requirement this file answers is "the conf must be able to name ALL of the input -- the mode, the
// extrinsics, the distortion, the noise, and whether the input is a bag or a folder of images, and which
// topics if it is a bag".  So there is no compiled default that a run depends on and no path that only
// exists in code: what is not in the file is reported as missing rather than silently substituted.
//
// FORMAT
//   `key = value`, one per line, `#` starts a comment, blank lines ignored, case-insensitive keys.  A path
//   for an INPUT datum is used as written; the two output paths (solution, and the UI's own state) are
//   resolved against the repository root.  Each key names its own units, and every value that has units is
//   spelled with them in the key or stated in the comment beside it -- a rotation in degrees is `_deg`, a
//   translation is metres, a noise density is the sensor's own unit per sqrt(Hz).

#include <string>
#include <vector>

namespace ali {

/// Which of ORB-SLAM3's four sensor configurations a run uses.  These are the four the acceptance
/// criterion names, and each is one of ORB-SLAM3's own modes rather than an arrangement of this project's.
enum class Mode {
  kMono,           ///< one camera, no inertial
  kMonoInertial,   ///< one camera plus IMU
  kStereo,         ///< two cameras, no inertial
  kStereoInertial, ///< two cameras plus IMU
  kLidar,          ///< one LiDAR, point-to-plane scan-to-map
  kLidarInertial,  ///< one LiDAR plus IMU (IMU-aided prediction)
  kEpipolarStereo,    ///< stereo, point-to-epipolar-line visual odometry
  kEpipolarStereoInertial,  ///< the same, with the gyro fused into the frame-to-frame rotation
  kEpipolarVio,       ///< the same visual front end, IMU preintegration + point-line tight window
  kGnss,              ///< pure GNSS: DD pseudorange/carrier positions, one solve per epoch, no camera/IMU
  kGnssInertial,      ///< GNSS + IMU: the same DD edges with IMU preintegration in a VioWindow, no camera
  /// GNSS + IMU backbone with the epipolar front end: one window frame per GNSS epoch, IMU
  /// preintegration between epochs, and the front end's point-line matches between consecutive epochs.
  kGnssInertialEpipolar,
  /// LiDAR + IMU + DD on the same 1 Hz GNSS backbone, with LiDAR relative-pose factors.
  kGnssInertialLidar,
  /// LiDAR + epipolar visual + IMU + DD on the 1 Hz GNSS backbone.
  kGnssInertialLidarEpipolar,
  kLvi,               ///< LiDAR + epipolar visual + IMU in the LiDAR fixed-lag window
};

/// The two-camera modes, the only ones that read a right image at all.
inline bool isStereoMode(Mode m) {
  return m == Mode::kStereo || m == Mode::kStereoInertial || m == Mode::kEpipolarStereo ||
         m == Mode::kEpipolarStereoInertial || m == Mode::kEpipolarVio ||
         m == Mode::kGnssInertialEpipolar || m == Mode::kGnssInertialLidarEpipolar;
}
inline bool isInertialMode(Mode m) {
  return m == Mode::kMonoInertial || m == Mode::kStereoInertial ||
         m == Mode::kEpipolarStereoInertial || m == Mode::kEpipolarVio ||
         m == Mode::kGnssInertial || m == Mode::kGnssInertialEpipolar ||
         m == Mode::kGnssInertialLidar || m == Mode::kGnssInertialLidarEpipolar ||
         m == Mode::kLvi;
}
inline bool isEpipolarMode(Mode m) {
  return m == Mode::kEpipolarStereo || m == Mode::kEpipolarStereoInertial ||
         m == Mode::kEpipolarVio;
}
inline bool isLidarMode(Mode m) { return m == Mode::kLidar || m == Mode::kLidarInertial; }
/// Modes whose only inputs are GNSS (and optionally IMU): no images, vocabulary or camera geometry.
inline bool isGnssMode(Mode m) {
  return m == Mode::kGnss || m == Mode::kGnssInertial || m == Mode::kGnssInertialEpipolar ||
         m == Mode::kGnssInertialLidar || m == Mode::kGnssInertialLidarEpipolar;
}
/// LiDAR + epipolar visual + IMU, using the LiDAR branch's fixed-lag window.
inline bool isLviMode(Mode m) { return m == Mode::kLvi; }

/// Where the images come from.
enum class ImageSource {
  kImages,  ///< a directory of images, named so that sorting gives the time order, or a manifest
  kBag,     ///< a ROS1 bag, from which a topic is read
};

/// Where the inertial data comes from.  Independent of `ImageSource` on purpose: the 4Seasons recording
/// ships its images as PNGs and its IMU inside the bag, and a run has to be able to take one from each.
enum class ImuSource {
  kNone,    ///< no inertial data; only valid with kMono or kStereo
  kBag,     ///< a ROS1 bag, from which a topic is read
  kFile,    ///< a text file: `t,gx,gy,gz,ax,ay,az` per line, `#` comments
};

/// One camera: intrinsics, distortion, and where it is in the body frame.
struct CameraConf {
  /// `pinhole` or `kannala_brandt` (ORB-SLAM3's "KannalaBrandt8"; the 4Seasons raw cam0 is equidistant).
  /// The 4Seasons *undistorted* images are pinhole, which is what the stereo configurations use.
  std::string model = "pinhole";
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  /// Pinhole: k1 k2 p1 p2 [k3].  Kannala-Brandt: k1 k2 k3 k4.  Up to 5 numbers, used in that order.
  std::vector<double> distortion;
  int width = 0;
  int height = 0;
  /// `fps` decides the frame period for the mono/stereo (non-inertial) runs, which have no IMU clock to
  /// follow.  Zero means "use the timestamps as they come".
  double fps = 0.0;
  /// Camera pose in the body frame: the camera's origin and its attitude, both expressed in the body frame.
  /// `T_bc` is stored as the six numbers the run prints and the settings file consumes: translation in
  /// metres, rotation as a 3x3 row-major matrix.  Identity means "the camera IS the body frame", which is
  /// what a configuration that names no extrinsic gets -- and the run says so in its log rather than
  /// assuming it quietly.
  bool has_extrinsic = false;
  double t_bc[3] = {0.0, 0.0, 0.0};
  double r_bc[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  /// Optional: a single row of a stereo pair may name a rectification instead of an extrinsic.
  bool has_rectification = false;
  double r_rect[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
};

/// The inertial unit's noise model, in the units each is measured in.
struct ImuConf {
  /// The units the SOURCE writes its inertial samples in.  The estimator works in SI and always will, so
  /// these are a property of the input and belong in the configuration beside the path that names it.
  ///
  /// They exist because getting them wrong is silent and expensive: the 4Seasons gyroscope is in rad/s, an
  /// earlier version of this project believed it was in deg/s and multiplied by pi/180, and the result was
  /// a stereo-inertial map 530x too large (ATE 156 m against 2 m once corrected).  Three checks settle it
  /// for a recording -- the bag against the dataset's own imu.txt, the magnitude of a hard turn, and the
  /// integral of |gyro| against the reference trajectory's heading change -- and they are worth running
  /// once per recording rather than assuming.  All three 4Seasons sequences measured so far are rad/s and
  /// m/s^2.
  ///
  /// The `/sample` variants are for a stream that carries the INCREMENT over one sample rather than a rate,
  /// which is how an INS reports its raw output and which is not what `sensor_msgs/Imu` promises.  Measured
  /// on SMART_PNT 20240718: the bag's accelerometer reads 0.0495 and its integral of |gyro| over the drive
  /// is 25.7 deg, against gravity 9.81 m/s^2 and a reference heading change of 3621.6 deg -- both off by
  /// exactly the 200 Hz sample rate.  Converting those needs `imu_frequency`, so a configuration that names
  /// a per-sample unit without a frequency is rejected rather than silently scaled by one.
  enum class AngularUnit { kRadPerSec, kDegPerSec, kRadPerSample, kDegPerSample };
  enum class AccelUnit { kMetresPerSec2, kG, kMetresPerSecPerSample, kGPerSample };
  AngularUnit angular_unit = AngularUnit::kRadPerSec;
  AccelUnit accel_unit = AccelUnit::kMetresPerSec2;
  /// True when either unit is per-sample, i.e. when the conversion depends on `frequency`.
  bool unitsArePerSample() const {
    return angular_unit == AngularUnit::kRadPerSample || angular_unit == AngularUnit::kDegPerSample ||
           accel_unit == AccelUnit::kMetresPerSecPerSample || accel_unit == AccelUnit::kGPerSample;
  }

  /// `timeshift` is seconds: `t_imu = t_cam + shift`, the same sign convention the 4Seasons calibration
  /// report states.
  double time_shift = 0.0;
  double frequency = 0.0;
  double noise_gyro = 0.0;    ///< rad/s/sqrt(Hz)
  double noise_acc = 0.0;     ///< m/s^2/sqrt(Hz)
  double gyro_walk = 0.0;     ///< rad/s^2/sqrt(Hz)
  double acc_walk = 0.0;      ///< m/s^3/sqrt(Hz)
  /// `fastInit` in ORB-SLAM3's settings: skip the "the vehicle must change its average acceleration"
  /// initialisation gate.  A car rarely does, which is why the 4Seasons port needs it.
  bool fast_init = false;
  /// Let a monocular-inertial run relocalise while the IMU is still uninitialised.
  ///
  /// FALSE is upstream's behaviour and it is a dead end.  `Tracking::Track`'s RECENTLY_LOST branch, for the
  /// three inertial sensors, does `bOK = false` whenever the map is not yet IMU-initialised -- it does NOT
  /// call `Relocalization()`, which is exactly what the non-inertial branch does on the same line.  So a map
  /// that loses tracking before the inertial unit is up cannot be recovered, and five seconds later it is
  /// destroyed.  The inertial initialisation in turn needs a map with keyframes that survives a couple of
  /// seconds, so the two requirements close a loop: measured on SMART_PNT 20240718, 230 map resets over the
  /// drive and ZERO inertial initialisations, with the tracker in RECENTLY_LOST for 62 % of it.
  ///
  /// TRUE gives the inertial branch the non-inertial branch's recovery.  It is a behaviour change and it is
  /// a setting on purpose: on a recording where this never triggers it can only do harm.
  bool relocalize_before_imu_init = false;

  /// `resetMapIfNotInitialized` in ORB-SLAM3's settings.  TRUE in upstream's own code path and FALSE here,
  /// because with it on the map is discarded on the FIRST failed `TrackLocalMap` while the inertial unit is
  /// not yet initialised -- and the initialisation needs two seconds and ten keyframes to accumulate, which
  /// it then never can.  The failure is a reset loop that looks like a tracker problem: measured on this
  /// drive, mono-inertial produced "New Map created with N points" nine times in twenty seconds and the
  /// words "IMU is not or recently initialized" between each.  The reference project's own configuration
  /// for this recording sets the same switch to 0.
  bool reset_map_if_imu_not_init = false;
  /// `minInitialParallax` in ORB-SLAM3's settings: the least disparity, in pixels, the two frames the
  /// monocular initialiser starts from may have.  Eight pixels is the port's measured default; zero turns
  /// the gate off and restores upstream's behaviour of initialising from whatever pair it finds.
  double min_initial_parallax = 8.0;

  /// Which statistic of the matched points' disparity `min_initial_parallax` is compared against.
  ///
  /// Both are defensible and they behave very differently on a scene with a large sky.  The MEAN is what
  /// this port has always used and what the 4Seasons results were measured with: the matches there are
  /// dominated by points at infinity, whose disparity is zero, so the mean sits well below the disparity of
  /// the structure the initialisation actually needs.  The MEDIAN asks the question the gate was meant to
  /// ask -- did half the matched points move by this much -- and on SMART_PNT 20240718 the difference is
  /// between a run that never initialises at all and one that tracks half the drive.
  ///
  /// The default is `mean`, so a configuration that does not name it keeps the behaviour its results were
  /// recorded with.
  enum class ParallaxStatistic { kMean, kMedian };
  ParallaxStatistic parallax_statistic = ParallaxStatistic::kMean;

  /// `minMatchesInliersPreInit` in ORB-SLAM3's settings: the matched-inlier count a monocular-inertial
  /// frame must reach to count as still tracking, while the map is not yet IMU-initialised.
  ///
  /// 50 is upstream's value and the port's default.  The pure-monocular branch of the same decision uses 30,
  /// and before the inertial unit is up a monocular-inertial map IS a monocular map -- which makes 30 the
  /// defensible lower value.  On SMART_PNT 20240718 the frame after each visual initialisation returns
  /// 21..49 inliers, so the first few frames of every map sit right under 50 and the map is destroyed; the
  /// one initialisation that crossed it grew from 2 keyframes and 589 points to 74 and 18558 over 21.7 s.
  int min_inliers_pre_init = 50;

  /// `IMU.resetIfNotEnoughMotion`: whether a stationary stretch may destroy the active map.
  ///
  /// TRUE is upstream: `LocalMapping::Run` resets the map when the last keyframe displacements are under
  /// 2 cm and the inertial initialisation has accumulated under 10 s of motion.  That reasoning holds for a
  /// drone that is about to take off and not for a car waiting at a junction, where it throws away a map
  /// that is tracking.  Measured on 4Seasons recording_2020-12-22 mono-inertial (five stops, two of 50.8 s
  /// and 68.5 s): 88 resets, against 0 on a recording whose longest stop is 2.6 s.
  bool reset_if_not_enough_motion = true;

  /// `IMU.InsertKFsWhenLost`: whether the tracker keeps inserting keyframes while it is in
  /// RECENTLY_LOST, i.e. while it is between a failed TrackLocalMap and the relocalisation timeout.
  ///
  /// FALSE is what this port has always written, and it is load bearing for the monocular drives: with
  /// keyframes still being inserted the map grows from poses that have not been matched, and the measured
  /// result there was 2296/2829 poses with 33-40 resets against 9390 with none.  For STEREO the mapper
  /// queues up to three requests instead of dropping them, so a brief loss can keep the map alive rather
  /// than leaving it frozen at whatever it had when tracking failed -- which is what GREAT campus01 did:
  /// 10 keyframes forever, zero matches, and a map reset every 150 frames.
  bool insert_kfs_when_lost = false;

  /// Minimum number of keyframes the inertial initialisation must see before it solves.  0 keeps
  /// upstream's 10 (or 10 after the temporal-chain/time checks).  A longer window is the lever when the
  /// early keyframes carry little excitation -- a car driving straight initialises to a degenerate bias
  /// (measured: ba ~ 2 g) on a 10-keyframe window, and the same run solved sanely once the window
  /// covered more of the drive.
  int init_min_keyframes = 0;



  /// Whether the inertial factors are allowed to shape the MAP (inertial initialisation, local inertial
  /// BA, VIBA).  true is upstream.  false still reads and preintegrates the IMU, and the stereo
  /// initialisation still uses it, but the map is never marked IMU-initialised, so every optimisation is
  /// the visual one -- a stable, metric stereo run with the inertial stream loaded but not trusted.
  /// This is the setting for a dataset whose camera-IMU calibration is not good enough for the inertial
  /// factors: GREAT campus01's shipped baseline is off by a factor of six (see the baseline note), and
  /// with the inertial factors on the map tilts and fails 35-100 s after the car pulls away.
  bool use_inertial_factors = true;

  /// Whether the tracker's per-frame pose refinement uses the inertial solve
  /// (`PoseInertialOptimizationLastFrame/LastKeyFrame`) once the map is IMU-initialised.
  ///
  /// TRUE is upstream.  The inertial solve adds the IMU preintegration between the last frame or keyframe
  /// and the current one; that is only helpful while the inertial state and the map's metric scale agree.
  /// On GREAT campus01 they do not: the stereo map comes out at roughly one seventh of the ground-truth
  /// scale while the IMU is metric, so the inertial term pulls every pose towards a scale the images do
  /// not support.  Measured over 120 s: with the inertial solve the tracker failed 132 times and replaced
  /// its map once; the same frames under the visual solve track end to end.  Turning this off keeps the
  /// IMU in initialisation and local inertial BA (map side) and lets the tracker stay visual.
  bool inertial_pose_tracking = true;

  /// Let the inertial initialisation estimate the map scale in STEREO-inertial mode.
  ///
  /// FALSE is upstream: the stereo baseline fixes the scale vertex.  That is only sound while the stereo
  /// map and the IMU agree on scale; on GREAT campus01 they do not (the stereo trajectory is about one
  /// seventh of the ground-truth length, the gyro's yaw timing and magnitude match to a few percent).  TRUE
  /// frees the scale vertex and applies the estimate to the map, exactly as a monocular map already does.
  bool estimate_scale = false;

  /// `slam.scale_refinement` in ORB-SLAM3's settings: 1 re-estimates the scale and the gyro bias roughly
  /// every eight seconds on a monocular map, instead of leaving them wherever the first initialisation put
  /// them.  It is the knob that exists for exactly the failure this drive shows -- a 313-second monocular
  /// run whose scale drifts from 1.0 to 3.3 (mono-inertial) or 0.005 (mono) while the trajectory keeps
  /// tracking -- and the port carries it as a setting none of the delivered configurations had turned on.
  bool scale_refinement = false;
  /// Gravity magnitude, m/s^2.  Zero means "let ORB-SLAM3 use its own default".
  double gravity = 0.0;
};

/// The body frame's attitude in the world/ENU frame, for a configuration whose IMU is mounted at an angle.
/// Three angles in degrees, applied Z (yaw), then Y (pitch), then X (roll) -- the same convention the
/// sibling project's `mounting_rpy` uses, so the two agree on what a triple means.
struct MountConf {
  bool has_rpy = false;
  double rpy_deg[3] = {0.0, 0.0, 0.0};
};

/// The whole file.
struct Conf {
  /// `prefetch_frames`: decode the frames on a second thread, one queue ahead of the tracker.
  ///
  /// FALSE by default, and the reason is the answer rather than the cost.  Reading is 15.8 ms of a 44.0 ms
  /// frame on the full 4Seasons drive (25.2 ms of 51.4 ms in stereo-inertial), and moving it to its own
  /// thread took the mono full drive from 417.8 s to 213.0-288.7 s and the stereo window from 51.4 to
  /// 40.1 ms/frame.  But the tracker and the mapping threads read and write the same map without ordering,
  /// so a faster tracker lands on different interleavings and therefore on a different trajectory: four full
  /// drives of mono-inertial, two either side of the change, span 5007-9390 poses.  The frames are identical
  /// -- 48 of 48 sampled frames carry the same image mean -- so this is not an input difference.  A
  /// configuration that wants the speed says so; one being compared with older results leaves it off.
  bool prefetch_frames = false;

  /// ALI: weight every triangulated map point by the anisotropic pixel covariance of its two
  /// keypoints.  False is stock ORB-SLAM3 and is the baseline arm of every comparison; true is the
  /// project's contribution.  Written into the generated ORB-SLAM3 settings as `ALI`.
  bool ali = false;

  /// Minimum frames between keyframes; 0 is upstream.  See the note that used to sit with this field:
  /// on GREAT campus01 stereo the map grew by ~0.85 keyframes per frame and the per-frame time from 45
  /// to 380 ms over 200 s; a small value here bounds that growth.
  int keyframe_min_frames = 0;

  // ---- LiDAR ------------------------------------------------------------------------------------
  /// The directory that holds `timestamps.txt` and `data/` (the GREAT layout).  Scans are read in
  /// timestamp order and registered point-to-plane against a voxelised map.
  std::string lidar_dir;
  float lidar_min_range = 1.0f;
  float lidar_max_range = 120.0f;
  float lidar_scan_voxel = 0.3f;
  float lidar_map_voxel = 0.5f;
  int lidar_max_iterations = 8;
  float lidar_huber = 0.2f;
  /// Dense LiDAR map output.  Directory relative to the repository root; empty disables it.  One
  /// timestamped `map.pcd` + `map.json` is written per run, deduplicated on a `cloud_voxel` metre grid.
  std::string cloud_dir = "cloud";
  /// Optional prebuilt mapper map (PCD + JSON sidecar) used as the display/saved map.  The estimator's
  /// point-plane persistent map remains online; this only feeds the UI/cloud writer.
  std::string display_map;
  float cloud_voxel = 1.0f;
  int cloud_max_points = 2000000;
  /// The LiDAR's pose in the body frame (body = IMU), used by the inertial mode's prediction.  Unset
  /// means the LiDAR frame is taken as the body frame.
  bool has_lidar_extrinsic = false;
  double r_bl[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double t_bl[3] = {0, 0, 0};

  /// Fixed-lag window for the LiDAR branches, the analogue of `epipolar_vio_*`:
  ///   0 or 1  -- the legacy per-scan scan-to-map path (no window),
  ///   >= 2    -- a VioWindow of this many scans.  `lidar-inertial` adds IMU preintegration, biases and
  ///              gravity-aligned states; `lidar` runs the same window without inertial factors.
  /// The interval factor is the scan-to-map relative pose (EdgeLidarRelative); the per-frame scan-to-map
  /// pose anchors the window exactly as the VO pose anchors the epipolar window.
  int lidar_window = 0;
  /// `window` publishes the window's current pose; `icp` keeps the legacy scan-to-map pose while the
  /// window still runs (the A/B output switch).  The delivered GREAT configurations use `window`.
  std::string lidar_window_output = "icp";
  /// Slide the window with the Schur prior (on) or drop the oldest frame without one (off).  The delivered
  /// epipolar window uses off + the per-frame anchor; the LiDAR default follows that.
  bool lidar_window_marginalize = false;
  /// Solve the LiDAR window every Nth scan instead of every 10 Hz scan.  10 = 1 Hz on the GREAT scans.
  int lidar_window_solve_interval = 1;
  /// LVI: use the persistent Traj-LO map (built from scans that left the window) instead of rebuilding the
  /// point-to-plane map from the current window on every solve.  Same map policy as strict GVIL.
  bool lidar_window_persistent = false;
  /// Fix the oldest pose at its scan-to-map pose.  With `lidar_window_anchor_all` on, every frame also
  /// gets a scan-to-map pose prior (the epipolar VIO's per-frame VO anchor); off, only the oldest pose is
  /// fixed and the scan-to-map relative factors + IMU determine the rest.
  bool lidar_window_anchor = true;
  bool lidar_window_anchor_all = true;
  /// Hold the gyro/accel biases at their initial values instead of estimating them freely.
  bool lidar_window_fix_biases = false;
  int lvi_visual_keyframe = 3;
  /// Per-frame scan-to-map anchor sigmas (m, deg).
  double lidar_window_prior_trans = 0.05;
  double lidar_window_prior_rot_deg = 0.5;
  /// Relative scan-to-map pose factor sigmas (m, deg).  This is the fallback factor for a frame whose
  /// range-image normals produced no point-to-plane correspondence.
  double lidar_window_relative_trans = 0.05;
  double lidar_window_relative_rot_deg = 0.5;
  /// Point-to-plane window factor: cap on correspondences per scan and the residual sigma (m).
  int lidar_window_points = 120;
  double lidar_window_point_sigma = 0.10;
  /// LVI: sigmas of the epipolar VO's LiDAR-relative pose over each scan interval.  This factor is added
  /// alongside the point-to-plane residuals; it is not a replacement for them.
  double lvi_visual_trans_sigma = 0.05;
  double lvi_visual_rot_deg = 0.5;
  double lvi_visual_huber = 3.0;

  /// `IMU.firstAlignmentSeconds` / `IMU.secondAlignmentSeconds`: accumulated motion, in seconds, before the
  /// first and second inertial alignment (upstream 5 and 15).
  ///
  /// The alignment solves for the map scale from the IMU against the visual trajectory, and five seconds of
  /// a straight road is very little to observe a scale from.  Measured on 4Seasons recording_2020-10-07
  /// mono-inertial: one drive landed on a scale four times too small and lost the map at frame 700, while six
  /// others of the same recording landed on a good value.  A longer window is the only lever on that
  /// estimate; it delays the metric scale and nothing else.
  double first_alignment_seconds = 5.0;
  double second_alignment_seconds = 15.0;

  std::string name;
  std::string description;
  std::string path;   ///< where it was read from

  Mode mode = Mode::kMonoInertial;

  // ---- input ------------------------------------------------------------------------------------
  ImageSource image_source = ImageSource::kImages;
  /// kImages: a directory of images, or a text manifest of `<stamp> <path>` lines.  A manifest is
  /// recognised by content, not by name: if the first non-comment line has two fields and the second is
  /// not a bare image name the file is a directory listing is impossible, so the rule is simpler than
  /// that -- a path that is a directory is a directory, anything else is read as a manifest.
  std::string images;
  std::string images_dir;        ///< a directory (kept separate so a conf can name both)
  /// A manifest of `<stamp_seconds> <image path>`, for a dataset whose extraction is not one file per
  /// frame in time order or whose timestamps are not in the file names.
  std::string images_manifest;
  /// The right camera's own source, for the stereo modes.  When it is left empty the left directory is
  /// used with `cam0` swapped for `cam1`, which is the layout the 4Seasons extraction has; a left path
  /// that does not contain `cam0` is refused rather than quietly paired with itself.
  std::string images_right_dir;
  std::string images_right;
  std::string images_manifest_right;
  // kBag:
  std::string bag;
  std::string bag_image_topic;     ///< LEFT camera (or the only one)
  std::string bag_image_topic_right;
  int bag_start = 0;
  int bag_frames = 0;              ///< 0 = all

  // ---- the inertial input -----------------------------------------------------------------------
  ImuSource imu_source = ImuSource::kNone;
  std::string imu_bag;
  std::string imu_topic;
  std::string imu_file;

  // ---- camera -----------------------------------------------------------------------------------
  std::string vocabulary;
  std::string settings_out;        ///< where the ORB-SLAM3 settings YAML is written; empty = temp
  CameraConf camera;               ///< left / only
  CameraConf camera_right;         ///< right, for the stereo modes
  /// metres; ORB-SLAM3's `STEREO.T_c1_c2` translation.  `baseline` is accepted as an alias for the
  /// x-component when the pair is rectified, which is the 4Seasons case.
  bool has_baseline = false;
  double baseline = 0.0;

  ImuConf imu;
  MountConf mounting;

  // ---- the run ----------------------------------------------------------------------------------
  int features = 1500;             ///< ORB features per frame
  float scale_factor = 1.2f;
  int levels = 8;
  int ini_th_fast = 20;
  int min_th_fast = 7;
  /// Seconds to run; 0 means the whole sequence.
  double duration = 0.0;
  /// First frame index to use; skips without decoding.
  /// The epipolar branch's own knobs: feature cap for its ORB front end, and the worker-thread count
  /// (0 = all cores).  Kept apart from `features`, which sizes the visual SLAM extractor.
  int epipolar_features = 1600;
  bool epipolar_pointline_mask = false;
  int epipolar_threads = 0;
  /// When the window adjustment runs: `auto` (low-parallax, wide-baseline frames), `on`, or `off`.
  std::string epipolar_window = "auto";
  /// Window length N (>= 3): 5(N-1)+(N-2) variables, all-pair residuals, run every Nth frame.
  int epipolar_window_frames = 3;
  /// Tight VIO equation handling: `partial` re-adds every retained factor and marginalises only the
  /// dropped velocity/biases; `full` is the FEJ full slide (the prior replaces every retained factor and
  /// only the newest interval is solved).
  std::string epipolar_vio_slide = "partial";
  /// `vio` (default): the written trajectory is the VIO window's current pose.  `vo`: the written
  /// trajectory is the VO's current pose (the delivered visual/loose result), while the VIO still runs and
  /// publishes its states/covariance.  GREAT needs the latter: the tight VIO's own pose is 1.41 m against
  /// the delivered loose 0.68 m, and its path is 4 % long even on the FEJ full slide.
  std::string epipolar_vio_output = "vio";
  /// Blend weight of the gyro-predicted camera rotation into the VO rotation when the tight branch
  /// publishes the VO pose (`output=vo`) or runs the loose branch.  0 = pure image geometry, 0.5 = the
  /// historical setting, 1 = gyro rotation.  The VIO window itself always keeps the gyro as a factor.
  double epipolar_imu_weight = 0.5;
  /// Decode the next image pair on a producer thread while the tracker works.  A win when image I/O is the
  /// bottleneck (GREAT 60 s 87.7 -> 77.9 ms/frame); on the cached 4Seasons 60 s the extra thread competes
  /// with the CPU-bound front end (58.0 against 44.2 ms), so it is off by default and on for GREAT.
  bool epipolar_prefetch = false;
  /// Tight VIO window length (frames) for mode `epipolar-vio`.  The visual window above is a separate,
  /// visual-only estimator; this one carries the 15 inertial/visual states per frame.
  int epipolar_vio_window = 6;
  /// High-rate VI window: 10 Hz image states + 20/30 Hz point-line observations, 45-state window,
  /// solved at 1 Hz.  No GNSS or LiDAR; the GVIL-style marginalization is the only long-term memory.
  bool epipolar_vio_high_rate = false;
  int epipolar_vio_high_rate_frames = 45;
  /// Cap on the point-line matches per interval that enter the tight window, evenly subsampled.  The
  /// factor is a direction constraint and is heavily redundant; the numeric Jacobian's cost is linear in
  /// this number.
  int epipolar_vio_matches = 100;
  int epipolar_vio_visual_keyframe = 3;
  /// State-layer weights exposed for the debug/variance study: the per-frame VO pose anchor, the point-line
  /// visual sigma, and the bias prior (used when no Schur prior is active).  Defaults are the delivered
  /// values; changing them changes the VIO states/covariance, not the VO trajectory.
  double epipolar_vio_vo_prior_trans = 0.05;     ///< metres
  double epipolar_vio_vo_prior_rot_deg = 0.5;    ///< degrees
  double epipolar_vio_visual_sigma = 1.0;
  double epipolar_vio_bias_sigma = 0.1;
  /// Hold the gyro/accel biases at their initial values in the epipolar VIO window.  The GREAT
  /// stereo/IMU calibration is not accurate enough for a free bias over a long drive: the window's
  /// accel bias ran away after ~120 s and took the velocity with it, while the same run with fixed
  /// biases stays within 0.6 % of the reference path length.  Off keeps the original free-bias mode.
  bool epipolar_vio_fix_biases = false;
  /// Slide the window with the Schur prior (on) or drop the oldest frame (off, the delivered behaviour).
  bool epipolar_vio_marginalize = false;
  /// Schur-prior warm-up: run the plain full-window solve until the window has moved this many metres.
  /// `epipolar_vio_warmup_moving` extends it to a sequence that is already moving at its first frame
  /// (GREAT); the parked case (4Seasons) is covered by the front-end stationary flag.
  bool epipolar_vio_warmup_moving = false;
  double epipolar_vio_warmup_distance = 2.0;
  /// Plain full-window state mode: fix the oldest pose at its VO pose and add every frame's VO pose prior,
  /// with no Schur prior.  Needed with marginalization off; the unanchored plain window free-runs.
  bool epipolar_vio_vo_anchor = false;
  /// Mean flow (px) below which the epipolar front end and the tight VIO's ZUPT treat the frame as parked.
  /// Keep it near the parked noise floor (0.02-0.03 px): a high threshold freezes a slow creep and the
  /// first measured PnP then snaps to the accumulated motion in one step.
  double epipolar_stationary_flow = 0.10;
  /// Keyframe-PnP NCC radius (px) and the cap on keyframe features used by the PnP (0 = all).
  int epipolar_pnp_radius = 10;
  int epipolar_pnp_features = 600;
  /// Maximum stereo depth (m) of a keyframe point admitted to the PnP.  Far points have large depth
  /// error and do not constrain the translation; 80 m was the old unconditional cap.
  double epipolar_pnp_max_depth = 80.0;
  /// How far the vehicle may move, or turn, before the PnP's reference frame is refreshed.  A longer
  /// reference baseline observes the translation direction better; beyond it the constant-velocity
  /// projection of the older reference's points degrades.  0 refreshes every frame.
  double epipolar_keyframe_distance = 0.5;
  double epipolar_keyframe_degrees = 4.0;

  /// Double-difference GNSS factors in the existing VioWindow.  The observations are the GREAT rover
  /// RINEX (1 Hz), the WUH2 base (30 s or 1 Hz; base corrections are reconstructed at the rover epochs)
  /// and the BRDM ephemeris; the factor edges are unary on the existing window poses.  `gnss_truth` is
  /// used for the world-to-ECEF gauge and the fixed ambiguity initialisation in this first validation.
  bool gnss_dd = false;
  /// Which DD measurements enter the window.  Keeping the two factors separable lets a code-only run show
  /// whether a phase inconsistency or the pseudorange geometry is responsible for a bad solve.
  bool gnss_code = true;
  bool gnss_phase = true;
  std::string gnss_rover;
  std::string gnss_base;
  std::string gnss_nav;
  std::string gnss_truth;
  double gnss_base_ecef[3] = {-2267750.1464, 5009154.5069, 3221294.3799};
  double gnss_lever[3] = {0.0, 0.0, 0.0};
  /// DD pseudorange / carrier zenith sigmas, metres.
  double gnss_code_sigma = 0.6;
  double gnss_phase_sigma = 0.05;
  /// Huber threshold for the DD pseudorange factors, in **metres of the unwhitened residual** (the edge
  /// returns metres and the window applies 1/sigma^2 once).  The campus code outliers are already
  /// removed by `gnss_code_outlier` at preparation time, so a tight threshold also downweights the
  /// legitimate decimetre/metre state error this branch is meant to correct.  Larger values let the code
  /// factors pull; phase retains its own 3-sigma kernel.
  double gnss_code_huber = 3.0;
  /// Multiplier on the information (1/sigma^2) of every GNSS code and phase factor.  This lets RTK
  /// dominate a strong VO/LiDAR anchor without pretending the DD sigmas are smaller than their
  /// elevation-weighted values: 1 = the original weighting.
  double gnss_weight = 1.0;
  /// Absolute RTK antenna-position factor sigma (metres); 0 disables the extra position factor.
  double gnss_position_sigma = 0.0;
  /// Yaw-only prior sigma (degrees); 0 disables it.  In GNSS mode the prior is ECEF position + yaw,
  /// not the full local VO pose: roll/pitch stay on gravity/IMU.
  double gnss_yaw_sigma_deg = 0.0;
  /// Carrier-phase arc reset threshold (metres); see GnssPrepOptions::phase_slip_threshold.
  double gnss_phase_slip = 0.5;
  /// Code-DD outlier gate at the reference antenna (metres); 0 disables it.
  double gnss_code_outlier = 0.0;
  /// Estimate one float DD phase ambiguity per carrier arc in the window instead of holding the
  /// initial value.  Enabled for the camera-less GNSS routes; the visual routes keep the fixed path.
  bool gnss_float_ambiguity = false;
  double gnss_ambiguity_prior = 1.0;
  /// Epoch selection for the GNSS-only / GNSS+IMU routes.  0 means "first prepared epoch" (start) or
  /// "the end of the requested duration / all epochs" (end).  `start_frame` is the epoch offset when
  /// `gnss_start_time` is not set, so the existing `--start-frame`/`--duration` CLI overrides still mean
  /// something in these camera-less modes.
  double gnss_start_time = 0.0;
  double gnss_end_time = 0.0;
  /// Frames retained by the GNSS+IMU fixed-lag window (1 Hz epochs: seconds of history).
  int gnss_window = 8;
  /// GI marginalization window: keep a short 1 Hz window and carry the dropped epoch's
  /// information in a Schur prior instead of hard-dropping it.  This is the visual/LiDAR-free
  /// counterpart of the strict GVLI window; it does not add high-rate states.
  bool gnss_gi_marginalize = false;
  /// Number of 1 Hz states in the GI marginalization window (1 state = 1 s).
  int gnss_gi_frames = 8;
  /// GIL: run a local 10 Hz LiDAR BA over each 1 s interval and feed the low-rate window a
  /// Schur-compressed relative pose factor instead of the 1 Hz point-plane map.
  bool lidar_local_ba = false;
  /// GIL prototype: one 4 s high-rate window (10 Hz scans) with DD/IMU/point-plane in the same graph,
  /// solved once per GNSS epoch instead of the 1 Hz frame + local-BA structure.
  bool gnss_lidar_4s_window = false;
  /// Number of high-rate frames in the 4 s prototype (45 = 40 scans at 10 Hz + epoch frames).
  int gnss_lidar_4s_frames = 45;
  /// Use the mapper's scan-to-map point-plane rows directly instead of rebuilding the joint map.
  bool gnss_lidar_4s_mapper_points = false;
  /// Per-image cap on the 20 Hz epipolar matches interpolated into the 4 s GVLI window.  At 20 Hz a
  /// small cap already gives several times the 1 Hz route's observation rate, and it bounds the
  /// numeric-Jacobian cost of the interpolated point-line edges.
  int gnss_lidar_4s_visual_matches = 100;
  /// Add epipolar point-line factors only on every Nth 10 Hz scan state.  The intermediate images are
  /// still tracked (and published) but do not enter the graph; this is the keyframe reduction that
  /// shrinks the visual batch without changing the scan/IMU/GNSS chain.
  int gnss_lidar_4s_visual_keyframe = 1;
  /// Add only every Nth 10 Hz scan to the VioWindow as an optimization state.  Intermediate scans are
  /// still mapped/tracked for the UI, but do not create vertices; the IMU preintegration spans the
  /// keyframe interval.  This is the state-window reduction, separate from the visual-residual keyframe.
  int gnss_lidar_4s_scan_keyframe = 1;
  /// Use Gauss-Newton instead of Levenberg-Marquardt for the 4 s window.  GN is materially cheaper
  /// per iteration; the caller opts in because the 4 s graph is usually well initialised from the
  /// mapper and the previous epoch.
  bool gnss_lidar_4s_gauss_newton = false;
  /// Solver iterations per GVIL epoch.  The 60 s sweep showed 3 iterations is the practical lower bound
  /// for the 45-state graph; fewer iterations trade accuracy for wall time.
  int gnss_lidar_4s_solver_iterations = 5;
  /// GVI high-rate visual window: 10 Hz image states + 20 Hz point-line observations, with the 1 Hz
  /// DD interpolated between the same states.  This is the visual-only counterpart of the strict 4 s
  /// LiDAR window (no LiDAR scan or persistent map).
  bool gnss_visual_4s_window = false;
  /// GVI visual keyframe: add point-line factors on every Nth 10 Hz image state.
  int gnss_visual_4s_visual_keyframe = 3;
  bool gnss_visual_4s_gauss_newton = true;
  int gnss_visual_4s_solver_iterations = 5;
  /// Number of 10 Hz image states in the visual high-rate window (45 states = 4.5 s).
  int gnss_visual_4s_frames = 45;
  /// Per-image cap on the 20 Hz point-line matches in the visual high-rate window.
  int gnss_visual_4s_matches = 100;
  /// Diagnostic: compare 0.1 s IMU preintegration with the mapper's body relative motion, then exit.
  bool imu_probe = false;
  /// Constant IMU scale calibration applied to the high-rate 4 s window (1 = raw).
  double imu_gyro_scale = 1.0;
  double imu_accel_scale = 1.0;
  /// Experimental: include IMU preintegration in the local LiDAR BA.  A 1 s interval anchored at a
  /// GNSS state is not observable enough to carry the inertial state (measured: the end velocity
  /// collapses and the factor hurts); the delivered configs leave this off.  The strict design is a
  /// single 4 s high-rate window, not this periodic local re-anchor.
  bool lidar_local_imu = false;
  /// GIE/GVLI: run a local 20 Hz point-line BA over each 1 s image interval and add its compressed
  /// camera-relative factor to the 1 Hz window.
  bool visual_local_ba = false;
  /// Hold the biases at zero instead of estimating them.  Useful as a diagnostic when a 1 Hz GNSS stream
  /// cannot observe the IMU biases; off by default so the route is a genuine GNSS/INS estimate.
  bool gnss_fix_biases = false;

  int start_frame = 0;
  /// How many frames between the per-frame diagnostic lines in the run log.
  ///
  /// 200 is right for a full drive -- it is the resolution at which a trajectory's shape is readable and
  /// the log stays a few thousand lines.  It is too coarse to see a single event: the monocular drive
  /// changes character somewhere inside one 200-frame interval, and at that spacing the interval itself is
  /// all that is visible.  Setting this to 1 turns the log into a per-frame trace.
  int debug_every = 200;
  /// The window's own trailing zero is the viewer, which this project has not got.
  bool viewer = false;
  bool save_map = false;

  // ---- output -----------------------------------------------------------------------------------
  /// The trajectory, in the same four-column shape ORB-SLAM3's own examples write.
  std::string solution;
  /// Extra diagnostics this project adds on top of ORB-SLAM3's: how many keyframes, where the frames went.
  std::string log_path;

  std::vector<std::string> warnings;      ///< keys whose value did not parse, and unknown keys
  std::vector<std::string> unknown_keys;

  /// Resolve the output path: `sol/<the conf's stem>.txt` unless the file names one.
  std::string resolvedSolution(const std::string& repo_root) const;
};

/// Read a configuration.  Returns false and fills `error` when the file cannot be read, or when a datum
/// the selected mode needs is missing -- the mode decides what is required, so an incomplete STEREO
/// configuration fails here rather than halfway through a run.
bool loadConf(const std::string& path, Conf* out, std::string* error);

/// A human-readable line per field, for the run's log and the UI's panel.
std::vector<std::string> describeConf(const Conf& conf);

/// The mode's own name, for logs and file names: "mono", "mono-inertial", "stereo", "stereo-inertial".
const char* modeName(Mode mode);

/// The repository root, derived from an executable's own path (the parent of its directory when the build
/// places binaries one level down, or of `bin/` when they are staged).  Empty when it cannot be told.
std::string repoRootFrom(const std::string& executable_path);

}  // namespace ali
