# Third-party notices

This package combines:

- **VINS-Mono** (https://github.com/HKUST-Aerial-Robotics/VINS-Mono), GPL-3.0.  The
  `thirdparty/camera_model`, `thirdparty/feature_tracker` and `thirdparty/vins_estimator` trees are
  derived from it.  `LICENSE` contains the GPL-3.0 text.
- **camodocal**, distributed inside the VINS-Mono tree (GPL-3.0).
- **OpenCV**, Apache-2.0.
- **Ceres Solver**, BSD-3-Clause.
- **Eigen**, MPL-2.0.
- **Boost**, Boost Software License 1.0.

The Windows build links the OpenCV, Ceres and glog binaries supplied with the ALI experiments; their
licences are those of the upstream projects.  The VINS-Mono source files retain their original
copyright headers.
