# ALI-VINS

**ALI-VINS** follows the KLT tracking and SLAM estimator design of OpenVINS, uses GFTT
(block-wise Shi--Tomasi) detection, and applies ALI (Anisotropic Landmark Initialization) at
landmark initialization.  It keeps a lightweight Win32 UI and is a standalone visual-inertial
odometry implementation.

## Layout

```text
ALI-VINS/
  CMakeLists.txt          CMake project
  build_msvc.bat          builds with MSVC from E:\jo_gvil\visual studio and CMake from E:\jo_gvil\cmake
  cmake/openvins.cmake    OpenVINS source subset, dependency paths and link items
  src/
    app_main.cpp          Win32 UI / --list / --run / --plot / --compare
    ali_vins_run.cpp      driver: manifest + IMU -> VioManager -> trajectory
  thirdparty/openvins/    ov_core + ov_init + ov_msckf (no ROS, no ArUco)
  config/
    4seasons_neighborhood_3.conf
    4seasons_office_loop_5.conf
    openvins_4seasons.yaml / _calib.yaml / _imu.yaml
    openvins_4seasons_office.yaml / _office_calib.yaml
    data/                 manifests and references for the two sequences
  tools/
    gauge_free_score.py   ATE scorer used by the UI
    bag_extract.py, bag_inspect.py, make_manifests.py
  bin/                    build outputs and runtime DLLs
  sol/                    trajectories, logs and scores
```

## Build

```bat
build_msvc.bat
```

The build requires:

| Dependency | Version / note |
|---|---|
| CMake | 3.20 or newer |
| Ninja | any recent version |
| MSVC | Visual Studio 2022 / MSVC 19.44 |
| Windows SDK | 10.0.26100.0 |
| OpenCV | 4.6.0 |
| Eigen | 3.3.5 |
| Ceres | 1.14.0 |
| Boost | 1.92.0 |

All four dependency roots can be overridden:

```bat
build_msvc.bat [OpenCV] [Eigen] [Ceres] [Boost] [jobs]
```

## Run

GUI:

```bat
bin\ali_vins_app.exe
```

Headless:

```bat
bin\ali_vins_app.exe --list
bin\ali_vins_app.exe --run 4seasons_neighborhood_3
bin\ali_vins_app.exe --run 4seasons_office_loop_5
```

`--frames N` overrides the frame count in the configuration; `--no-score` skips scoring.

## ALI and the MSCKF/SLAM design

* The experiment configurations use `max_slam: 50`, `max_slam_in_update: 25`,
  `dt_slam_delay: 1`, and `feat_rep_slam: ANCHORED_MSCKF_INVERSE_DEPTH`.  The estimator has no
  `use_slam` switch: `max_slam > 0` enables the SLAM branch, while MSCKF still runs in parallel.
* The driver enables ALI by default with `OV_WEIGHT=l2cov`.  To run the unweighted baseline,
  clear the variable before starting:

```bat
set OV_WEIGHT=
bin\ali_vins_run.exe ...
```

* The only ALI implementation change is in `delayed_init` in
  `ov_msckf/src/update/UpdaterSLAM.cpp`: the dense LLT of the full `R` is replaced by block-wise
  whitening of each 2x2 measurement block.  `R` is block diagonal, so the two forms are
  mathematically equivalent; the maximum position difference over 1000 frames is `1.2e-6 m`, while
  a 2000-frame run drops from about 240 s to about 86 s.

## Feature extractor

ALI-VINS uses a **block-wise Shi-Tomasi (GFTT)** feature extractor:

* Each grid cell calls `cv::goodFeaturesToTrack` with `qualityLevel = 0.01` and
  `minDistance = min_px_dist` (15 in the current configuration), followed by sub-pixel refinement.
* The ROI mask uses 255 for valid pixels; the code applies `bitwise_not` to the internal mask to
  match the GFTT interface.
* KLT tracking and the grid layout are unchanged; the detector is GFTT and ALI is used at landmark
  initialization.  The per-frame feature cap is set by `max_features` in the `.conf` file (see the
  next section).

GFTT is the default; to switch back to FAST for an A/B comparison:

```bat
set OV_DETECTOR=fast
```

## Maximum features per frame

The `.conf` files expose `max_features`.  The UI parses it and passes it to the driver as
`--max-features`; the driver overrides only the estimator's `num_pts` (the post-initialization
per-frame cap) and leaves `init_max_features` at the YAML value of 200, so the dynamic
initialization behavior is unchanged.  Both current configurations use `max_features = 600`.

GFTT cap sweep on the first 2000 frames of neighborhood_3:

| max_features | 300 | 500 | 600 | 700 | 800 |
|---|---:|---:|---:|---:|---:|
| RMS 3D | 17.25 m | 7.22 m | **6.89 m** | 9.10 m | 15.26 m |

600 is best; 500--600 is a plateau, while 700/800 are worse.  The number of tracked features grows
with the cap as well: 300 -> 223, 500 -> 348, 600 -> 434, 800 -> 486 tracks at frame 2000.

## Dataset

Download 4Seasons from the official page:

https://cvg.cit.tum.de/data/datasets/4seasons-dataset/download

This project uses the `neighborhood_3` and `office_loop_5` recordings.  After downloading, prepare
or update the manifest, IMU stream and reference files according to the configurations in
`config/`; the repository does not contain the raw images, IMU data or ground truth.

## Notes

* This is a minimal ALI-VINS implementation built on the ROS-free `ov_core + ov_init + ov_msckf`
  subset; it does not use ROS, PCL, Pangolin, g2o or Sophus.
* `CpiV2.cpp` is not compiled: dynamic initialization uses only `CpiV1`, and keeping the second
  unit would add a very slow compilation unit.
* OpenVINS is GPLv3; keep `thirdparty/openvins/LICENSE` when redistributing.
