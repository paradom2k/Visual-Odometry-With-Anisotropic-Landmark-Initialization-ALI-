# ali-orbslam3

ORB-SLAM3 with **Anisotropic Landmark Initialization (ALI)**, built for Windows with MSVC, driven by a
single configuration file, and evaluated on two 4Seasons driving recordings.

ORB-SLAM3 triangulates a new map point from two matched keypoints with an unweighted midpoint/SVD
estimate, which treats both observations as equally precise in both image directions.  ALI gives every
keypoint a 2x2 pixel covariance from the structure tensor of the patch it was detected on, and solves the
triangulation as a whitened least-squares problem instead: a keypoint on an edge is trusted across the
edge and distrusted along it.  No other stage of ORB-SLAM3 is changed — the same matching, the same gates,
the same optimizer.

This is the ORB-SLAM3 counterpart of the ALI strategy evaluated in the sibling OpenVINS port, where it
is applied to the SLAM landmark's initial covariance.

## Results

The delivered configurations are **monocular-inertial** (single cam0 + IMU), matching the single-camera
setup of the OpenVINS experiments this work started from.  ATE is RMS 3D after yaw+translation alignment
with the scale fixed at 1 (`tools/gauge_free_score.py`).  Baseline and ALI are the same binary, same
configuration except `ali`.

### Monocular-inertial, full recordings

| Sequence | Arm | RMS 3D (m) | Path ratio | Poses | Maps / lost frames | ALI landmarks |
| --- | --- | --- | --- | --- | --- | --- |
| neighborhood_3 (309.8 s, 9487 frames) | baseline | 14.955 | 0.953 | 9390 | 1 / 0 | — |
| neighborhood_3 | ALI | **13.992** | 0.962 | 9390 | 1 / 0 | 125 534 |
| office_loop_5 (447.5 s, 13746 frames) | baseline | — | — | — | 17 / 2434 by frame 7200 | — |
| office_loop_5 | ALI | not yet measured | — | — | — | — |

`neighborhood_3` is a clean paired run: both arms finish with one map, zero lost frames and the same
9390 poses, and ALI is 6.4 % lower.  That is one run per arm, and ORB-SLAM3 is not bit-reproducible, so
treat it as a first measurement rather than an established effect.

`office_loop_5` is **not** stable yet with the monocular-inertial recipe: the baseline re-created the
map 17 times and reported 2434 lost frames by frame 7200, so the run was stopped before the trajectory
was worth scoring.  The same recipe is stable on `neighborhood_3`; the difference between the two
recordings was not diagnosed.  The office numbers in the table below are from the earlier
stereo-inertial configurations and are kept because they are what that mode measured.

### Earlier stereo-inertial evaluation (kept for reference)

| Sequence | Baseline (two runs) | ALI | Same-binary comparison |
| --- | --- | --- | --- |
| neighborhood_3 | 4.960 / 7.151 | 5.724 | +0.76 m (worse) |
| office_loop_5 | 6.545 / 4.214 | 6.563 | +0.02 m (indistinguishable) |
| office_loop_5, before the safety gates | 6.545 | 216.596, 9863.115 | **lost tracking, 2 of 2 runs** |

The two baseline runs of the same configuration differ by 2.19 m on `neighborhood_3` and 2.33 m on
`office_loop_5`; loop closure succeeding or not is what moves the number by metres.  Against that spread,
the gated ALI effect in stereo mode was not measurable, and the ungated form lost tracking on office.

### What the two modes say together

* In **stereo-inertial**, most landmarks come from stereo disparity, not triangulation, so ALI has little
  to act on; measured, it was neutral.
* In **monocular-inertial**, every landmark is triangulated, ALI applies to all of them (125 534 on the
  neighborhood drive), and the one stable paired run improved by 6.4 %.  This is the mode where the idea
  has leverage, and the mode the delivered configurations now use.
* Monocular accuracy is about 15 m on this drive against 5-7 m for stereo, because the scale comes from
  the IMU alone.  The configurations carry the settings that make that mode survive (2000 Hz IMU rate,
  2000 features, `imu_reset_map_if_not_initialized = off`, `relocalize_before_imu_init = on`,
  `scale_refinement = off`, and `prefetch_frames = off`; `conf/README.md` says why each one).

## Layout

```
CMakeLists.txt            the build description
build_msvc.bat            Windows/MSVC build with the paths of the development machine
patches/orbslam3/         the changed/added ORB-SLAM3 files, and why
thirdparty/orbslam3/      unmodified upstream ORB-SLAM3 1.0 source subset

src/ali/                  configuration parser, image/IMU sources, ali_run, ali_ui
src/common/               logging, time, geodesy
src/data/bag_reader.*     ROS-free ROS1 bag reader (optional input path)
src/compat/orbslam3/      the Windows/POSIX/Pangolin/Boost stand-ins

conf/                     one configuration per sequence, plus the two references and an ALI variant
tools/gauge_free_score.py the scorer every number above comes from
bin/                      the built executables and the OpenCV runtime DLLs they need
```

## Build

Requirements: **MSVC 2019/2022 (v142/v143)**, **CMake ≥ 3.20**, **Ninja**, **OpenCV 4.x** as a
self-contained install (headers, import libraries, DLLs), **Eigen 3.3+**, and **GLFW** only if you want
`ali_ui`.  No ROS, no Pangolin, no Boost installation (stand-ins are in `src/compat/orbslam3/boost/`).

```bat
build_msvc.bat [OpenCV root] [Eigen root] [Eigen parent] [GLFW root] [jobs]
```

or by hand:

```bat
cmake -G Ninja -S . -B build -DCMAKE_BUILD_TYPE=Release ^
      -DCMAKE_CXX_COMPILER=<cl.exe> ^
      -DALI_OPENCV_ROOT=<opencv install> ^
      -DALI_EIGEN_ROOT=<dir containing Eigen/Core> ^
      -DALI_EIGEN_PARENT=<dir containing eigen3/> ^
      -DALI_GLFW_ROOT=<glfw root>
cmake --build build -j 8
```

`ALI_EIGEN_PARENT` is the directory that contains `eigen3/`, because the bundled g2o includes
`eigen3/Eigen/...`.  `-DALI_BUILD_UI=OFF` builds `ali_run` only.

The build copies `thirdparty/orbslam3/` into `build/thirdparty/orbslam3/` and overlays
`patches/orbslam3/overlay/`; `thirdparty/` is never edited, and `git diff thirdparty/` stays empty.

## Running

```
bin\ali_ui.exe                              pick a configuration, press Run, watch it
bin\ali_run.exe --config conf\4seasons_neighborhood_3.conf
bin\ali_run.exe --config conf\4seasons_neighborhood_3_ali.conf --out sol\ali.txt
bin\ali_run.exe --check-only --config <conf>    parse and report, run nothing
```

The trajectory is written to `sol/<configuration stem>.txt` in ORB-SLAM3's own format
(`timestamp tx ty tz qx qy qz qw`, seconds).  Score it:

```
python tools\gauge_free_score.py --est sol\4seasons_neighborhood_3.txt ^
       --ref conf\data\neighborhood_3_reference.csv --time-scale 1
```

The configurations name the dataset by **absolute path**, because the recordings live outside the
repository.  Edit `images_dir`, `images_right_dir` and `imu` before running; `--check-only` prints every
resolved path and refuses a configuration with a missing required key.

### The configuration file

`key = value`, one per line, `#` starts a comment.  The keys that matter here:

| Key | Meaning |
| --- | --- |
| `mode` | `mono`, `mono-inertial`, `stereo`, `stereo-inertial` |
| `input`, `images_dir`, `images_right_dir` | a directory of timestamp-named images (or `input = bag`) |
| `imu`, `imu_frequency`, `imu_noise_gyro`, … | a `t,gx,gy,gz,ax,ay,az` CSV or a bag |
| `camera_*`, `camera_right_*`, `baseline`, `r_bc`, `t_bc` | calibration; `t_bc`/`r_bc` are the camera pose **in the IMU frame** |
| `vocabulary` | the ORB vocabulary (`ORBvoc.txt`) |
| **`ali`** | **`on` = anisotropic landmark initialization, `off` = stock ORB-SLAM3** |
| `prefetch_frames` | decode on a second thread (much faster; the delivered configs use it) |

The runner generates the ORB-SLAM3 YAML settings file from this (`sol/settings.yaml`), so the
camera-model names, `T_b_c1` layout and distortion scaling are written once, in one place.

## What ALI changes, exactly

Each keypoint's patch gives a structure tensor `J = sum [Ix², IxIy; IxIy, Iy²]`.  Its inverse is the
covariance of a correlation-style localisation: large along an edge, small across it.  The covariance is
normalised to determinant 1 and its eigenvalue ratio clamped to 4, so the *total* precision of a keypoint
is unchanged and only the direction is new.

`GeometricTools::TriangulateALI` starts from stock ORB-SLAM3's SVD estimate and runs up to five
Gauss-Newton steps on

```
sum_i ( pi(T_i X) - x_i )^T  Cov_i^-1  ( pi(T_i X) - x_i )
```

with each step capped at half the current depth.  Two safety gates decide whether the result is used at
all, and both were added after the measurements showed what happens without them:

* only pairs with at least ~2.5 degrees of ray parallax (cos < 0.999) are refined — below that the depth
  is barely observable and any reweighting moves it by tens of percent;
* the refined point is kept only if it lowers that cost **and** moves less than 10 % of its depth from
  the SVD estimate.

`Triangulate` itself is untouched, so the baseline arm is exactly upstream's estimator.

Where the code is: `Frame::ComputeKeypointCovariances`, `GeometricTools::TriangulateALI`,
`LocalMapping::CreateNewMapPoints` (the ALI branch), `Settings` (`ALI` key), `System` (wires it up).
`CHANGES.md` lists every file; `patches/orbslam3/README.md` explains the overlay mechanism.

## Changes to ORB-SLAM3 and provenance

`CHANGES.md` is the authoritative list: the ALI additions, the Windows/MSVC port (no behaviour change),
and the optional switches inherited from the sibling GIO_GVIL project (all default to upstream and none is
enabled by the delivered configurations).

## Licence

**GPL-3.0-or-later**, because this is a derivative of ORB-SLAM3 (GPLv3).  See `LICENSE` and
`THIRD_PARTY_NOTICES.md` (ORB-SLAM3, DBoW2, g2o, Sophus, OpenCV, Eigen, GLFW).

## Known limitations

* ALI is applied when both observations are single-camera keypoints (monocular and rectified stereo).
  The two-independent-camera stereo path and the monocular initialiser keep stock behaviour.  In the
  delivered monocular configurations that covers every triangulated landmark.
* The covariance models a correlation-style localisation (the KLT model ALI comes from).  ORB keypoints
  are FAST detections whose error is more isotropic; the measured effect on ATE is what the Results
  section reports, not what the model alone would predict.
* Monocular-inertial is much weaker than stereo on this data (≈15 m against 5-7 m over the drive) and, on
  `office_loop_5`, still unstable with the delivered recipe — the run resets its map.  `neighborhood_3`
  is stable.  The difference was not diagnosed.
* Runs are not bit-reproducible (multiple estimator threads).  Compare arms with the same binary and
  average over repeats.
* The delivered configurations hard-code absolute dataset paths, as the recordings are not part of the
  repository.  The IMU is read from a CSV at the recording's native 2000 Hz; a `.bag` with
  `imu_topic = /imu_raw` is the alternative the configuration also supports.
