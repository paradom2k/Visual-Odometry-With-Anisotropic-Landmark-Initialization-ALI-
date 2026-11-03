# Changes to ORB-SLAM3

`thirdparty/orbslam3/` is the unmodified upstream ORB-SLAM3 1.0 source subset (source files dated
2022-02-10).  Everything this project changes or adds is a file in `patches/orbslam3/overlay/`, with the
same relative path, and CMake applies the overlay to a copy in `build/thirdparty/orbslam3/`.  Nothing in
`thirdparty/` is edited by hand, so `git diff thirdparty/` is always empty and the provenance of a
changed file is its own path under `patches/`.

The overlay is a full-file mirror, not a patch series.  It has three parts, listed below in order of
importance.

## 1. ALI — Anisotropic Landmark Initialization (this project's contribution)

ORB-SLAM3 creates a map point from two matched keypoints with `GeometricTools::Triangulate`, an
unweighted midpoint/SVD estimate.  That treats both observations as equally precise in **both** image
directions.  ALI replaces the step with a weighted triangulation that uses each keypoint's 2x2 pixel
covariance, so a point whose local structure is a line is trusted across the line and distrusted along
it, instead of being treated as an isotropic dot.

| File | Change |
| --- | --- |
| `include/Frame.h`, `src/Frame.cc` | `mvKpCov`: a 2x2 pixel covariance per keypoint.  `ComputeKeypointCovariances()` fills it after undistortion from the inverse structure tensor of the patch each keypoint was detected on (3-pixel half window at its own pyramid level); it is called from the stereo, RGB-D and monocular constructors.  Eigenvalues are normalised to a determinant of 1 and clamped to a ratio of 4, so **only the direction** of the uncertainty differs from stock ORB-SLAM3 — every keypoint keeps the same total precision the unweighted triangulation gives it. |
| `include/KeyFrame.h`, `src/KeyFrame.cc` | `mvKpCov`, copied from the `Frame` that creates the keyframe (indexed like `mvKeys`). |
| `include/GeometricTools.h`, `src/GeometricTools.cc` | `TriangulateALI`: the SVD estimate above is the initial value; up to 5 Gauss-Newton steps then minimise the whitened reprojection error `sum_i (pi(T_i X) - x_i)^T Cov_i^-1 (pi(T_i X) - x_i)`.  Each step is capped at half the current depth.  The refined point is used only if it lowers the whitened cost of the SVD point **and** moves less than 10 % of its depth from it; otherwise the stock estimate is returned. |
| `include/LocalMapping.h`, `src/LocalMapping.cc` | `SetALI(bool)` / `ALITriangulations()`; in `CreateNewMapPoints` the triangulation branch calls `TriangulateALI` when ALI is on, both keyframes carry covariances, **and the pair has at least ~2.5 degrees of ray parallax** (`cos < 0.999`).  The parallax gate is a safety measure measured to be necessary: without it, `office_loop_5` lost tracking in 2 of 2 runs (see `docs/RESULTS.md`).  The stereo-`UnprojectStereo` branches are untouched: ALI changes triangulation, not depth-from-disparity. |
| `include/Settings.h`, `src/Settings.cc` | `ALI` settings key (integer, optional, default 0) and `aliEnabled()`. |
| `src/System.cc` | `LocalMapping::SetALI(settings_->aliEnabled())`, next to the existing settings hand-overs. |

What ALI does **not** change: the reprojection gates, the parallax gates, the number of map points
created, the descriptors, the matching, or anything in the optimizer.  A point that ALI rejects is
rejected by the same tests stock ORB-SLAM3 applies, computed at the ALI estimate.

The switch is per configuration (`ali = on|off` in the `.conf`), written into the generated ORB-SLAM3
settings file as `ALI: 0|1`, announced in the run log, and the run ends with the number of landmarks
initialised through the anisotropic path.

**Scope limit.**  ALI is applied when both observations come from `mvKeys`/`mvKeysUn` (`NLeft == -1`,
which is every monocular and every rectified-stereo configuration).  The two-independent-camera stereo
path (`mpCamera2`) and the monocular initialiser's `TwoViewReconstruction` keep stock behaviour; the
code says so at the branch.

## 2. Windows / MSVC port (no behaviour change on any platform)

These are the changes that make ORB-SLAM3 compile and run with MSVC; they do not alter the estimator.

| File / directory | Change |
| --- | --- |
| `src/MapPoint.cc` | variable-length array replaced by `std::vector` (MSVC C2131). |
| `Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp` | two GCC variable-length arrays replaced by `std::vector`. |
| `Thirdparty/g2o/config.h` | the CMake-generated g2o configuration, expanded for a standalone MSVC build. |
| `src/compat/orbslam3/unistd.h` | POSIX header stand-in (`usleep` etc. are in `jo_compat.h`). |
| `src/compat/orbslam3/jo_compat.h` | force-included into every kernel translation unit: `usleep` via `Sleep`, `M_PI`, `COMPILEDWITHC11`, `vasprintf`, `winsock2.h` before `windows.h`. |
| `src/compat/orbslam3/pangolin/pangolin.h`, `port/viewer_stubs.cpp` | no-op viewer: this project has no Pangolin, and the methods the core calls are defined as stubs. |
| `src/compat/orbslam3/boost/…` | no-op stand-ins for the Boost.Serialization/Algorithm headers ORB-SLAM3's atlas save/load path includes; that path is not called by this project. |
| `src/compat/orbslam3/openssl/md5.h`, `stdint-gcc.h` | self-contained MD5 and the GCC integer typedefs `System.cc` expects. |
| `src/Frame.cc` | static camera intrinsics and image bounds recomputed per frame.  Upstream updates them only for the first frame ever; a second `System` in the same process then builds frames with the first camera's intrinsics.  A correctness fix, visible only when two runs share a process (the UI's "run another" path). |

## 3. Optional switches inherited from the sibling GIO_GVIL project (all default to upstream)

The overlay also carries switches the sibling project uses for its own experiments.  Each one is read
from the settings file, defaults to the upstream value, and **none is enabled by the configurations
delivered here**.

| File | Switch |
| --- | --- |
| `src/LocalMapping.cc`, `include/LocalMapping.h` | periodic scale/IMU-bias refinement (`scaleRefinement`, default 0 = upstream's six early windows); IMU fast-init; minimum keyframe spacing; motion-only VIBA window. |
| `src/Optimizer.cc`, `include/Optimizer.h`, `include/G2oTypes.h`, `src/G2oTypes.cc` | optional Schur-complement marginalization inside the inertial BA, bias priors, velocity prior — all zero/off by default. |
| `include/MarginalizationInfo.h`, `src/MarginalizationInfo.cc`, `include/MarginalizationPrior.h`, `include/GnssAnchor.h` | the marginalization and GNSS-anchor modules those switches use.  `GnssAnchor.h` is not referenced at all in this build. |
| `src/Tracking.cc`, `include/Tracking.h` | monocular parallax gate statistic, map-reset switch, relocalisation-before-IMU-init, diagnostic getters used by the runner and the UI. |
| `src/LoopClosing.cc`, `include/LoopClosing.h` | configurable roll/pitch gate for inertial loops (default 0.008 rad = upstream). |
| `include/ORBextractor.h`, `src/ORBextractor.cc` | expose the descriptor pattern/orientation and accept externally supplied keypoints. |
| `src/TwoViewReconstruction.cc` | diagnostic prints for rejected initialisations. |
| `include/System.h`, `src/System.cc` | diagnostics getters, right-camera keypoints, empty-atlas guard. |

## 4. New code outside the ORB-SLAM3 tree (not a modification of it)

| Path | What |
| --- | --- |
| `src/ali/` | the configuration parser, image/IMU sources, the headless runner and the GLFW UI (`ali_run`, `ali_ui`). |
| `src/common/` | logging, time conversion, geodesy helpers. |
| `src/data/bag_reader.*` | a ROS-free ROS1 bag reader, so a configuration can name a bag instead of a directory. |
| `src/compat/orbslam3/` | the Windows/POSIX layer listed in section 2. |
| `tools/gauge_free_score.py` | yaw+translation ATE scorer (scale fixed at 1), the ruler every number in the README uses. |
