# Visual Odometry with Anisotropic Landmark Initialization (ALI)

This repository collects three implementations of **Anisotropic Landmark
Initialization (ALI)** for landmark-based visual odometry and SLAM:

| Directory | Role |
|---|---|
| [`ALI-VINS/`](ALI-VINS/README.md) | The proposed independent implementation: GFTT detection, KLT tracking + SLAM design adopted from OpenVINS, with ALI at landmark initialization. |
| [`openvins-with-ali/`](openvins-with-ali/README.md) | OpenVINS integration: ALI supplies the anisotropic covariance for SLAM landmark initialization. |
| [`ORB-SLAM3-with-ALI/`](ORB-SLAM3-with-ALI/README.md) | ORB-SLAM3 integration: ALI supplies a structure-tensor covariance for the observations used in map-point triangulation. |

ALI uses the local structure tensor of a detected feature to build a \(2\times
2\) anisotropic covariance.  A keypoint on an edge is trusted across the edge
and distrusted along it, instead of being treated as equally precise in both
image directions.

## Repository layout

```text
ALI-VINS/                 proposed implementation; source, configs, Windows binaries
openvins-with-ali/        OpenVINS + ALI; source, configs, Windows binaries
ORB-SLAM3-with-ALI/       ORB-SLAM3 + ALI; source, configs, Windows binaries
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

The combined repository is distributed under **GPL-3.0**.  The OpenVINS and
ORB-SLAM3 sources retain their upstream GPL notices; the vendored third-party
trees and the prebuilt runtime DLLs are listed in
[`THIRD_PARTY.md`](THIRD_PARTY.md) and in each project's third-party notice.
The datasets are not part of this repository and remain subject to their own
terms.
