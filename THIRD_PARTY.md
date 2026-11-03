# Third-party software and licenses

This package combines original code with a vendored OpenVINS kernel and a
prebuilt Windows runtime.  The combined work is distributed under GPL-3.0;
the components below retain their own licenses.

## Vendored source

| Component | Version / provenance | License | Included |
|---|---|---|---|
| OpenVINS (`ov_core`, `ov_init`, `ov_msckf`) | https://github.com/rpng/open_vins | **GPL-3.0** | `thirdparty/openvins/`, including `thirdparty/openvins/LICENSE` |
| OpenVINS CPI files (`ov_core/src/cpi/`) | Upstream OpenVINS subcomponent | MIT (headers retained) | Included in the vendored tree |

OpenVINS is the estimation kernel.  The ROS nodes, stand-alone entry points and
tests were removed when it was vendored; the remaining source has been patched
for this project as described in `CODE_CHANGES.md`.  The small CPI subcomponent
retains its upstream MIT headers; the combined work is distributed under
GPL-3.0.

## Prebuilt runtime DLLs in `bin/`

These binaries are Windows x64 runtime dependencies of the supplied executable.
They are not built from this repository.

| DLL(s) | Component | License | Upstream |
|---|---|---|---|
| `opencv_*460.dll` | OpenCV 4.6.0 | Apache-2.0 | https://github.com/opencv/opencv |
| `ceres.dll` | Ceres Solver | BSD-3-Clause | https://github.com/ceres-solver/ceres-solver |
| `glog.dll` | Google glog | BSD-3-Clause | https://github.com/google/glog |
| `gflags.dll` | gflags | BSD-3-Clause | https://github.com/gflags/gflags |
| `openblas.dll` | OpenBLAS | BSD-3-Clause | https://github.com/OpenMathLib/OpenBLAS |
| `lapack.dll` | LAPACK reference implementation | BSD-3-Clause | https://www.netlib.org/lapack/ |

## Build-time headers / libraries

| Component | License | Use |
|---|---|---|
| Eigen | MPL-2.0 | Header-only linear algebra |
| Boost | BSL-1.0 | Filesystem, date-time, thread |
| Ceres Solver | BSD-3-Clause | Dynamic initialization |
| OpenCV | Apache-2.0 | Image I/O, KLT, feature tracking |

## Original code in this package

The application, driver, configuration files, experiment hooks, and the
`tools/` scripts are distributed as part of this GPL-3.0 work.  The vendored
OpenVINS CPI subcomponent keeps its upstream MIT headers.

## Notes for redistribution

- Keep `LICENSE`, `thirdparty/openvins/LICENSE`, and this file with any copy of
  the source or `bin/` directory.
- Keep the upstream license and notice files when redistributing the
  third-party DLLs, and comply with their attribution requirements.
- The datasets (4Seasons, GREAT campus, GICI, etc.) are **not** included and
  remain subject to their own terms.
- This notice is provided for attribution clarity and is not legal advice.
