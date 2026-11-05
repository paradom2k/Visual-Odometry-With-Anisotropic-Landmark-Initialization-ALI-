# VINS-Mono + ALI (ROS-free Windows port)

This directory is a ROS-free MSVC port of **VINS-Mono**, with anisotropic landmark initialization
(ALI) added as a per-feature reprojection weight.  It uses the same MSVC/CMake toolchain as
`ALI-VINS` and the same Win32 front end (`vins_app.exe`).

## What was ported

* `camera_model` (camodocal), `feature_tracker` and `vins_estimator` are VINS-Mono's own sources.
* The ROS node (`estimator_node.cpp`), publishers (`visualization.cpp`,
  `CameraPoseVisualization.cpp`) and the ROS parameter reader are replaced by `src/vins_run.cpp`
  and a file-based parameter reader.
* `marginalization_factor.cpp` originally used pthreads; it now sums the four accumulators serially,
  which builds with MSVC.
* The remaining Linux-only pieces (`execinfo.h`, `pthread.h`, `dirent.h`, `clock_gettime`) were
  removed or replaced.

## ALI integration

At a feature's first observation the driver builds an `ali::Frame` from the current grayscale image.
For every new feature it computes the structure tensor

```text
M_i = sum_{p in W_i} g(p) g(p)^T = U_i diag(lambda_1,i, lambda_2,i) U_i^T
```

updates the scene reference

```text
lambda2_bar <- 0.995 lambda2_bar + 0.005 lambda_2,i
```

and forms

```text
C_i = U_i diag(
        clip(lambda2_bar / lambda_1,i, 0.25, 4),
        clip(lambda2_bar / lambda_2,i, 0.25, 4)
      ) U_i^T
```

The feature's whitening is

```text
sqrt_info_i = (f / sigma_pix) * C_i^(-1/2)
```

and is used for the **first reprojection after the feature's start frame**; later observations keep
VINS-Mono's original `f/1.5 I`.  With `ali: 0` every feature uses `f/1.5 I`, i.e. stock VINS-Mono
apart from the ROS-free driver.

Files:

- `src/ali/ali.h`, `src/ali/ali.cpp` — structure tensor, reference update, whitening.
- `thirdparty/vins_estimator/src/factor/projection_factor.*` — per-instance `sqrt_info_i`.
- `thirdparty/vins_estimator/src/feature_manager.*` — per-feature `sqrt_info`.
- `thirdparty/vins_estimator/src/estimator.cpp` — passes the matrix to the factor.

## Configurations

`config/` contains 12 configurations: the six paper sequences, each with `ali: 0` and `ali: 1`.

| Sequence | Baseline | ALI | Frames |
|---|---|---|---|
| 4Seasons neighborhood_3 | `neighborhood_3.conf` | `neighborhood_3_ali.conf` | 6000 |
| 4Seasons office_loop_5 | `office_loop_5.conf` | `office_loop_5_ali.conf` | 6000 |
| GICI 1.1 | `gici_1_1.conf` | `gici_1_1_ali.conf` | full |
| GICI 2.1 | `gici_2_1.conf` | `gici_2_1_ali.conf` | full |
| GREAT campus03 | `campus03.conf` | `campus03_ali.conf` | 6000 |
| GREAT campus01 | `campus01.conf` | `campus01_ali.conf` | 6000 |

The GREAT campus manifests point at the rectified `campus01_rect/cam0` and `campus03_rect/cam0`
images generated for the stereo experiments; the camera intrinsics are the rectified P0 values and
distortion is zero.  GICI uses the raw camera images with the dataset's radtan distortion.

## Build and run

```bat
build_msvc.bat
```

Output: `bin/vins_run.exe` (headless estimator) and `bin/vins_app.exe` (Win32 UI).  Headless example:

```bat
set OV_NOISE=2
set OV_SEED=1
bin\vins_run.exe --config config\vins_campus01_ali.yaml ^
  --manifest "C:\Users\fengx\Desktop\texfile\IEEE-JSEN-LaTeX-template-202405\great_mc\inputs\campus01_rect\manifest.txt" ^
  --imu "C:\Users\fengx\Desktop\texfile\IEEE-JSEN-LaTeX-template-202405\great_mc\inputs\campus01\imu.csv" ^
  --out sol\run.txt --frames 6000
```

## 10-seed Monte Carlo

`系统实验/run_vins_mc.py` runs both arms with `OV_NOISE=2` and seeds 1-10.  The table reports the
mean ATE over the common non-divergent seeds:

| Sequence | VINS-Mono | VINS-Mono + ALI | p | n |
|---|---:|---:|---:|---:|
| 4Seasons neighborhood_3 | 10.81 m | 11.59 m | 0.232 | 10 |
| 4Seasons office_loop_5 | 23.12 m | 22.39 m | 1.000 | 10 |
| GICI 1.1 | — | — | — | 3 |
| GICI 2.1 | — | — | — | 0 |
| GREAT campus03 | 2.76 m | 2.58 m | 0.432 | 10 |
| GREAT campus01 | 16.56 m | 14.17 m | 0.375 | 10 |

The four 4Seasons/campus sequences are stable for all ten seeds.  VINS-Mono's GICI runs are much
less stable (GICI 1.1: 5/10 and 4/10 divergent; GICI 2.1: 9/10 in each arm); only three paired seeds
survive on GICI 1.1 and none on GICI 2.1, so both GICI rows are omitted from the comparison.  On the
stable sequences ALI does not change the mean significantly; the largest change is campus01
(16.56 to 14.17 m, p=0.375).  Runs are not bit-reproducible: repeat one seed with the same binary
before drawing conclusions from a single trajectory.

## Licence

GPL-3.0, as a derivative of VINS-Mono.  See `LICENSE` and the third-party notices.
