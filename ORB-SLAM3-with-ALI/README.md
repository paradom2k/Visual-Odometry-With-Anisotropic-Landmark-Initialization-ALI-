# ali-orbslam3

ORB-SLAM3 with **Anisotropic Landmark Initialization (ALI)**, built for Windows with MSVC, driven by a
single configuration file, and evaluated on four stereo sequences: two 4Seasons recordings and two GREAT campus recordings.

ORB-SLAM3 triangulates a new map point from two matched keypoints with an unweighted midpoint/SVD
estimate, which treats both observations as equally precise in both image directions.  ALI gives every
keypoint a 2x2 pixel covariance from the structure tensor of the patch it was detected on, and solves the
triangulation as a whitened least-squares problem instead: a keypoint on an edge is trusted across the
edge and distrusted along it.  No other stage of ORB-SLAM3 is changed — the same matching, the same gates,
the same optimizer.

This is the ORB-SLAM3 counterpart of the ALI strategy evaluated in the sibling OpenVINS port, where it
is applied to the SLAM landmark's initial covariance.

## Layout

```
CMakeLists.txt            the build description
build\_msvc.bat            Windows/MSVC build with the paths of the development machine
patches/orbslam3/         the changed/added ORB-SLAM3 files, and why
thirdparty/orbslam3/      unmodified upstream ORB-SLAM3 1.0 source subset

src/ali/                  configuration parser, image/IMU sources, ali\_run, ali\_ui
src/common/               logging, time, geodesy
src/data/bag\_reader.\*     ROS-free ROS1 bag reader (optional input path)
src/compat/orbslam3/      the Windows/POSIX/Pangolin/Boost stand-ins

conf/                     one configuration per sequence, plus references and an ALI variant
tools/gauge\_free\_score.py the scorer every number above comes from
bin/                      the built executables and the OpenCV runtime DLLs they need
```

## Build

Requirements: **MSVC 2019/2022 (v142/v143)**, **CMake ≥ 3.20**, **Ninja**, **OpenCV 4.x** as a
self-contained install (headers, import libraries, DLLs), **Eigen 3.3+**, and **GLFW** only if you want
`ali\_ui`.  No ROS, no Pangolin, no Boost installation (stand-ins are in `src/compat/orbslam3/boost/`).

```bat
build\_msvc.bat \[OpenCV root] \[Eigen root] \[Eigen parent] \[GLFW root] \[jobs]
```

or by hand:

```bat
cmake -G Ninja -S . -B build -DCMAKE\_BUILD\_TYPE=Release ^
      -DCMAKE\_CXX\_COMPILER=<cl.exe> ^
      -DALI\_OPENCV\_ROOT=<opencv install> ^
      -DALI\_EIGEN\_ROOT=<dir containing Eigen/Core> ^
      -DALI\_EIGEN\_PARENT=<dir containing eigen3/> ^
      -DALI\_GLFW\_ROOT=<glfw root>
cmake --build build -j 8
```

`ALI\_EIGEN\_PARENT` is the directory that contains `eigen3/`, because the bundled g2o includes
`eigen3/Eigen/...`.  `-DALI\_BUILD\_UI=OFF` builds `ali\_run` only.

The build copies `thirdparty/orbslam3/` into `build/thirdparty/orbslam3/` and overlays
`patches/orbslam3/overlay/`; `thirdparty/` is never edited, and `git diff thirdparty/` stays empty.

## Running

```
bin\\ali\_ui.exe                              pick a configuration, press Run, watch it
bin\\ali\_run.exe --config conf\\4seasons\_neighborhood\_3.conf
bin\\ali\_run.exe --config conf\\4seasons\_neighborhood\_3\_ali.conf --out sol\\ali.txt
bin\\ali\_run.exe --check-only --config <conf>    parse and report, run nothing
```

The trajectory is written to `sol/<configuration stem>.txt` in ORB-SLAM3's own format
(`timestamp tx ty tz qx qy qz qw`, seconds).  Score it:

```
python tools\\gauge\_free\_score.py --est sol\\4seasons\_neighborhood\_3.txt ^
       --ref conf\\data\\neighborhood\_3\_reference.csv --time-scale 1
```

The configurations name the dataset by **absolute path**, because the recordings live outside the
repository.  Edit `images\_dir`, `images\_right\_dir` (or `bag`/`topic`/`right\_topic`) and `imu` before
running; `--check-only` prints every resolved path and refuses a configuration with a missing required
key.

The GREAT campus stereo configs read the `stereo_bags` directly.  Those bags ship GIO-format
`left.idx`/`right.idx` sidecars; ORB-SLAM3 expects its own `# ali stereo bag index v1` header with the
same `stamp bag index` body, so the originals are kept as `left.gio.idx` and `right.gio.idx` beside the
bags.  The campus camera intrinsics, `T_bc` and the -0.02 s IMU shift are from
`E:/jo_gvil/recover_jo/GIO_GVIL_1.0/conf/great_campus0{1,3}_gvil.conf`; the images are already
undistorted/rectified.

### The configuration file

`key = value`, one per line, `#` starts a comment.  The keys that matter here:

|Key|Meaning|
|-|-|
|`mode`|`mono`, `mono-inertial`, `stereo`, `stereo-inertial`|
|`input`, `images\_dir`, `images\_right\_dir`|a directory of timestamp-named images (or `input = bag`)|
|`imu`, `imu\_frequency`, `imu\_noise\_gyro`, …|a `t,gx,gy,gz,ax,ay,az` CSV or a bag|
|`camera\_\*`, `camera\_right\_\*`, `baseline`, `r\_bc`, `t\_bc`|calibration; `t\_bc`/`r\_bc` are the camera pose **in the IMU frame**|
|`vocabulary`|the ORB vocabulary (`ORBvoc.txt`)|
|**`ali`**|**`on` = anisotropic landmark initialization, `off` = stock ORB-SLAM3**|
|`prefetch\_frames`|decode on a second thread (much faster; the delivered configs use it)|

The runner generates the ORB-SLAM3 YAML settings file from this (`sol/settings.yaml`), so the
camera-model names, `T\_b\_c1` layout and distortion scaling are written once, in one place.

## What ALI changes, exactly

Each keypoint's patch gives a structure tensor `J = sum \[Ix², IxIy; IxIy, Iy²]`.  Its inverse is the
covariance of a correlation-style localisation: large along an edge, small across it.  The covariance is
normalised to determinant 1 and its eigenvalue ratio clamped to 4, so the *total* precision of a keypoint
is unchanged and only the direction is new.

`GeometricTools::TriangulateALI` starts from stock ORB-SLAM3's SVD estimate and runs up to five
Gauss-Newton steps on

```
sum\_i ( pi(T\_i X) - x\_i )^T  Cov\_i^-1  ( pi(T\_i X) - x\_i )
```

with each step capped at half the current depth.  Two safety gates decide whether the result is used at
all, and both were added after the measurements showed what happens without them:

* only pairs with at least \~2.5 degrees of ray parallax (cos < 0.999) are refined — below that the depth
is barely observable and any reweighting moves it by tens of percent;
* the refined point is kept only if it lowers that cost **and** moves less than 10 % of its depth from
the SVD estimate.

`Triangulate` itself is untouched, so the baseline arm is exactly upstream's estimator.

Where the code is: `Frame::ComputeKeypointCovariances`, `GeometricTools::TriangulateALI`,
`LocalMapping::CreateNewMapPoints` (the ALI branch), `Settings` (`ALI` key), `System` (wires it up).
`CHANGES.md` lists every file; `patches/orbslam3/README.md` explains the overlay mechanism.

## Changes to ORB-SLAM3 and provenance

`CHANGES.md` is the authoritative list: the ALI additions, the Windows/MSVC port (no behaviour change),
and the optional switches inherited from the sibling GIO\_GVIL project (all default to upstream and none is
enabled by the delivered configurations).

## Licence

**GPL-3.0-or-later**, because this is a derivative of ORB-SLAM3 (GPLv3).  See `LICENSE` and
`THIRD\_PARTY\_NOTICES.md` (ORB-SLAM3, DBoW2, g2o, Sophus, OpenCV, Eigen, GLFW).

## Known limitations

* ALI is applied when both observations are single-camera keypoints (monocular and rectified stereo).
The two-independent-camera stereo path and the monocular initialiser keep stock behaviour.  In the
delivered monocular configurations that covers every triangulated landmark.
* The covariance models a correlation-style localisation (the KLT model ALI comes from).  ORB keypoints
are FAST detections whose error is more isotropic; ALI is therefore an initialization-time approximation rather than a calibrated ORB measurement model.
* Runs are not bit-reproducible (multiple estimator threads).  Compare arms with the same binary and
average over repeats.
* The delivered configurations hard-code absolute dataset paths, as the recordings are not part of the
repository.  The IMU is read from a CSV at the recording's native 2000 Hz; a `.bag` with
`imu\_topic = /imu\_raw` is the alternative the configuration also supports.

