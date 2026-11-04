# Visual Odometry with Anisotropic Landmark Initialization (ALI)

This repository collects four implementations of **Anisotropic Landmark
Initialization (ALI)** for landmark-based visual odometry and SLAM:

| Directory | Role |
|---|---|
| [`ALI-VINS/`](ALI-VINS/README.md) | The proposed independent implementation: GFTT detection, KLT tracking + SLAM design adopted from OpenVINS, with ALI at landmark initialization. |
| [`openvins-with-ali/`](openvins-with-ali/README.md) | OpenVINS integration: ALI supplies the anisotropic covariance for SLAM landmark initialization. |
| [`ORB-SLAM3-with-ALI/`](ORB-SLAM3-with-ALI/README.md) | ORB-SLAM3 integration: ALI supplies a structure-tensor covariance for the observations used in map-point triangulation. |
| [`vins-mono-with-ALI/`](vins-mono-with-ALI/README.md) | VINS-Mono integration: ROS-free Windows port with ALI whitening on the first reprojection after landmark birth. |

ALI uses the local structure tensor of a detected feature to build a \(2\times
2\) anisotropic covariance.  A keypoint on an edge is trusted across the edge
and distrusted along it, instead of being treated as equally precise in both
image directions.

## VINS-Mono with ALI

`vins-mono-with-ALI/` is a ROS-free MSVC port of **VINS-Mono** (GPL-3.0) with
ALI added to the sliding-window estimator.  The ROS node, publishers and ROS
parameter reader were replaced by a file-based driver, and the estimator uses
the same MSVC, CMake, Ninja, OpenCV, Eigen, Ceres and Boost toolchain as
`ALI-VINS`.

Stock VINS-Mono uses \(f/1.5\,I\) as the whitening matrix for every projection
factor.  At a new feature's first observation the port computes the local
structure tensor \(M_i\), its eigendecomposition
\(M_i=U_i\operatorname{diag}(\lambda_{1,i},\lambda_{2,i})U_i^\top\), the scene
reference \(\bar\lambda_2\leftarrow0.995\bar\lambda_2+0.005\lambda_{2,i}\), and

\[
C_i=U_i\operatorname{diag}\!\left(
\operatorname{clip}\!\left(\frac{\bar\lambda_2}{\lambda_{1,i}},0.25,4\right),
\operatorname{clip}\!\left(\frac{\bar\lambda_2}{\lambda_{2,i}},0.25,4\right)
\right)U_i^\top .
\]

The feature's whitening is then
\(\text{sqrt\_info}_i=f/\sigma_{\mathrm{pix}}\,C_i^{-1/2}\), applied to the
first reprojection after the feature's start frame; later observations keep
the stock \(f/1.5\,I\).  With `ali: 0` the configuration is stock VINS-Mono
apart from the ROS-free driver.

Four configurations are provided: `neighborhood_3` and `office_loop_5`, each
with and without ALI.  `bin\vins_run.exe` is the estimator and
`bin\vins_app.exe` is the same Win32 UI as `ALI-VINS`.  A single-seed check
with `OV_NOISE=2`, `OV_SEED=1`, 6000 frames gives:

| Sequence | VINS-Mono | VINS-Mono + ALI |
|---|---:|---:|
| 4Seasons neighborhood\_3 | 11.66 m | 11.09 m |
| 4Seasons office\_loop\_5 | 25.96 m | 25.42 m |

These are port/stability checks, not a 10-seed Monte Carlo.  The detailed
build and run instructions are in `vins-mono-with-ALI/README.md`.

## Repository layout

```text
ALI-VINS/                 proposed implementation; source, configs, Windows binaries
openvins-with-ali/        OpenVINS + ALI; source, configs, Windows binaries
ORB-SLAM3-with-ALI/       ORB-SLAM3 + ALI; source, configs, Windows binaries
vins-mono-with-ALI/       VINS-Mono + ALI; ROS-free MSVC port
LICENSE                   GPL-3.0 for the combined repository
THIRD\_PARTY.md            shared third-party notices
```

Each directory has its own README with build, run, and configuration details.

## Running

Windows x64 executables are included in each `bin/` directory.  The
configurations are examples and point to local dataset locations; update the
paths in the corresponding `.conf` file before running.  A source build
requires MSVC, CMake, Ninja, OpenCV, Eigen, Ceres, and Boost (see the
individual READMEs for exact versions).

## Datasets

The 4Seasons dataset is not included.  Download it from the official page:

https://cvg.cit.tum.de/data/datasets/4seasons-dataset/download

After downloading, prepare the image manifest, IMU stream, and ground truth
reference according to the layout described in each project's configuration
documentation.

## License and third-party code

The combined repository is distributed under **GPL-3.0**.  The OpenVINS,
ORB-SLAM3 and VINS-Mono sources retain their upstream GPL notices; the
vendored third-party trees and the prebuilt runtime DLLs are listed in
[`THIRD\_PARTY.md`](THIRD_PARTY.md) and in each project's third-party notice.
The datasets are not part of this repository and remain subject to their own
terms.

