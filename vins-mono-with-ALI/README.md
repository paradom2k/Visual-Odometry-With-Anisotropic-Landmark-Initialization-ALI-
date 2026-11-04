# VINS-Mono + ALI (ROS-free Windows port)

This directory is a ROS-free MSVC port of **VINS-Mono**, with anisotropic landmark initialization (ALI)
added as a per-feature reprojection weight.  It uses the same MSVC/CMake toolchain as `ALI-VINS` and the
same Win32 front end as `ALI-VINS` (`vins_app.exe`).

## What was ported

* `camera_model` (camodocal), `feature_tracker` and `vins_estimator` are VINS-Mono's own sources.
* The ROS node (`estimator_node.cpp`), publishers (`visualization.cpp`, `CameraPoseVisualization.cpp`)
  and the ROS parameter reader are replaced by `src/vins_run.cpp` and a file-based parameter reader.
* `marginalization_factor.cpp` originally used pthreads; it now sums the four accumulators serially,
  which is deterministic and builds with MSVC.
* The remaining Linux-only pieces (`execinfo.h`, `pthread.h`, `dirent.h`, `clock_gettime`) were removed
  or replaced.

## ALI integration

At each feature's first observation the driver builds an `ali::Frame` from the current grayscale image.
For every new feature it computes

\[
M_i=\sum_{\mathbf p\in W_i}\mathbf g(\mathbf p)\mathbf g(\mathbf p)^\top,\qquad
C_i=U_i\operatorname{diag}\!\left(\operatorname{clip}\!\left(\frac{\bar\lambda_2}{\lambda_{1,i}},0.25,4\right),
\operatorname{clip}\!\left(\frac{\bar\lambda_2}{\lambda_{2,i}},0.25,4\right)\right)U_i^\top,
\]

updates \(\bar\lambda_2\leftarrow0.995\bar\lambda_2+0.005\lambda_{2,i}\), and passes

\[
\text{sqrt\_info}_i=\frac{f}{\sigma_{\text{pix}}}C_i^{-1/2}
\]

to the feature.  The estimator stores it in `FeaturePerId::sqrt_info` and uses it for the **first
observation after the feature's start frame**; later observations use VINS-Mono's original
\(f/1.5\,I\).  With `ali: 0`, every feature uses \(f/1.5\,I\), i.e. stock VINS-Mono.

Files:

* `src/ali/ali.h`, `src/ali/ali.cpp` — structure tensor, reference update, whitening.
* `thirdparty/vins_estimator/src/factor/projection_factor.*` — per-instance `sqrt_info_i`.
* `thirdparty/vins_estimator/src/feature_manager.*` — per-feature `sqrt_info`.
* `thirdparty/vins_estimator/src/estimator.cpp` — passes the matrix to the factor.

## Build

```bat
build_msvc.bat
```

It uses the same MSVC 14.44, CMake, Ninja, OpenCV, Eigen, Ceres and Boost roots as `ALI-VINS`.  Output:

* `bin/vins_run.exe` — headless estimator;
* `bin/vins_app.exe` — the same Win32 UI as ALI-VINS.

## Run

```bat
bin\vins_run.exe --config config\vins_neighborhood_3.yaml ^
  --manifest config\data\neighborhood_3_manifest.txt ^
  --imu "E:\jo_gvil\jo_gvil 1.0\build\inputs\fs_full\imu.csv" ^
  --out sol\run.txt --frames 6000
```

Set `OV_NOISE=2` and `OV_SEED=1` to match the experiment protocol.  The UI is
`bin\vins_app.exe`: it lists the four configurations (`neighborhood_3`, `neighborhood_3_ali`,
`office_loop_5`, `office_loop_5_ali`), runs the selected one and scores it with
`tools/gauge_free_score.py`.

## Validation (4Seasons, seed 1, `OV_NOISE=2`, 6000 frames)

| Sequence | VINS-Mono | VINS-Mono + ALI |
| --- | ---: | ---: |
| neighborhood_3 | 11.66 m | 11.09 m |
| office_loop_5 | 25.96 m | 25.42 m |

ATE is the gauge-free score (yaw + translation, scale fixed at 1) from `tools/gauge_free_score.py`.
The path ratios were 0.940/0.941 for neighborhood_3 and 0.964/0.937 for office_loop_5.

These are single-seed checks, not the 10-seed Monte Carlo.  VINS-Mono is sensitive to the
initialization window and to the feature/keyframe sequence; repeated seeds can change the ATE
materially, so the table is a port/stability check, not a conclusion about ALI.  In this seed
ALI reduced the ATE slightly on both sequences (about 5% and 2%); one seed is not enough to
claim a significant effect.

## Licences

VINS-Mono is GPL-3.0; see `LICENSE`.  The vendored camodocal sources are part of VINS-Mono.  OpenCV,
Ceres, Eigen and Boost are used as external dependencies; see `THIRD_PARTY_NOTICES.md`.
