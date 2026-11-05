# Visual Odometry with Anisotropic Landmark Initialization (ALI)

This repository collects four implementations of **Anisotropic Landmark
Initialization (ALI)** for landmark-based visual odometry and SLAM:

|Directory|Role|
|-|-|
|[`ALI-VINS/`](ALI-VINS/README.md)|The proposed independent implementation: GFTT detection, KLT tracking + SLAM design adopted from OpenVINS, with ALI at landmark initialization.|
|[`openvins-with-ali/`](openvins-with-ali/README.md)|OpenVINS integration: ALI supplies the anisotropic covariance for SLAM landmark initialization.|
|[`ORB-SLAM3-with-ALI/`](ORB-SLAM3-with-ALI/README.md)|ORB-SLAM3 integration: ALI supplies a structure-tensor covariance for the observations used in map-point triangulation.|
|[`vins-mono-with-ALI/`](vins-mono-with-ALI/README.md)|VINS-Mono integration: ROS-free Windows port with ALI whitening on the first reprojection after landmark birth.|

ALI uses the local structure tensor of a detected feature to build a $2 \times 2$
anisotropic covariance.  A keypoint on an edge is trusted across the edge and
distrusted along it, instead of being treated as equally precise in both image
directions.

## VINS-Mono with ALI

`vins-mono-with-ALI/` is a ROS-free MSVC port of VINS-Mono (GPL-3.0).  Stock
VINS-Mono whitens every projection factor with $f/1.5\,I$; at a new feature's
first observation the port computes the structure tensor, its eigendecomposition
and the scene reference $\bar\lambda_2$, and uses

$$
\mathrm{sqrt\_info}_i=\frac{f}{\sigma_{\mathrm{pix}}}C_i^{-1/2}
$$

only for the first reprojection after the feature's start frame.  Later
observations keep the stock weight, so ALI acts as an initialization
covariance rather than a permanent reweighting.  The same MSVC/CMake/OpenCV/
Eigen/Ceres/Boost toolchain and Win32 UI as `ALI-VINS` are used.

Ten-seed Monte Carlo (`OV_NOISE=2`), mean ATE over common non-divergent seeds:

| Sequence | VINS-Mono | VINS-Mono + ALI | $p$ |
|---|---:|---:|---:|
| 4Seasons neighborhood_3 | 10.81 m | 11.59 m | 0.232 |
| 4Seasons office_loop_5 | 23.12 m | 22.39 m | 1.000 |
| GREAT campus03 | 2.76 m | 2.58 m | 0.432 |
| GREAT campus01 | 16.56 m | 14.17 m | 0.375 |

The GICI rows are omitted because fewer than five paired non-divergent seeds
remain.  Detailed build/run instructions and the full six-sequence table are
in `vins-mono-with-ALI/README.md`.

## Repository layout

```text
ALI-VINS/                 proposed implementation; source, configs, Windows binaries
openvins-with-ali/        OpenVINS + ALI; source, configs, Windows binaries
ORB-SLAM3-with-ALI/       ORB-SLAM3 + ALI; source, configs, Windows binaries
vins-mono-with-ALI/       VINS-Mono + ALI; ROS-free MSVC port
LICENSE                   GPL-3.0 for the combined repository
THIRD_PARTY.md            shared third-party notices
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
[`THIRD_PARTY.md`](THIRD_PARTY.md) and in each project's third-party notice.
The datasets are not part of this repository and remain subject to their own
terms.

