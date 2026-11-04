# VINS-Mono + ALI (ROS-free Windows port)

This directory is a ROS-free MSVC port of **VINS-Mono**, with anisotropic landmark initialization (ALI)
added as a per-feature reprojection weight.  It uses the same MSVC/CMake toolchain as `ALI-VINS` and the
same Win32 front end as `ALI-VINS` (`vins\\\_app.exe`).

## What was ported

* `camera\\\_model` (camodocal), `feature\\\_tracker` and `vins\\\_estimator` are VINS-Mono's own sources.
* The ROS node (`estimator\\\_node.cpp`), publishers (`visualization.cpp`, `CameraPoseVisualization.cpp`)
and the ROS parameter reader are replaced by `src/vins\\\_run.cpp` and a file-based parameter reader.
* `marginalization\\\_factor.cpp` originally used pthreads; it now sums the four accumulators serially,
which is deterministic and builds with MSVC.
* The remaining Linux-only pieces (`execinfo.h`, `pthread.h`, `dirent.h`, `clock\\\_gettime`) were removed
or replaced.

## ALI integration

At each feature's first observation the driver builds an `ali::Frame` from the current grayscale image.
For every new feature it computes

\[
M\_i=\\sum\_{\\mathbf p\\in W\_i}\\mathbf g(\\mathbf p)\\mathbf g(\\mathbf p)^\\top,\\qquad
C\_i=U\_i\\operatorname{diag}!\\left(\\operatorname{clip}!\\left(\\frac{\\bar\\lambda\_2}{\\lambda\_{1,i}},0.25,4\\right),
\\operatorname{clip}!\\left(\\frac{\\bar\\lambda\_2}{\\lambda\_{2,i}},0.25,4\\right)\\right)U\_i^\\top,
]

updates (\\bar\\lambda\_2\\leftarrow0.995\\bar\\lambda\_2+0.005\\lambda\_{2,i}), and passes

\[
\\text{sqrt\_info}*i=\\frac{f}{\\sigma*{\\text{pix}}}C\_i^{-1/2}
]

to the feature.  The estimator stores it in `FeaturePerId::sqrt\\\_info` and uses it for the **first
observation after the feature's start frame**; later observations use VINS-Mono's original
(f/1.5,I).  With `ali: 0`, every feature uses (f/1.5,I), i.e. stock VINS-Mono.

Files:

* `src/ali/ali.h`, `src/ali/ali.cpp` — structure tensor, reference update, whitening.
* `thirdparty/vins\\\_estimator/src/factor/projection\\\_factor.\\\*` — per-instance `sqrt\\\_info\\\_i`.
* `thirdparty/vins\\\_estimator/src/feature\\\_manager.\\\*` — per-feature `sqrt\\\_info`.
* `thirdparty/vins\\\_estimator/src/estimator.cpp` — passes the matrix to the factor.

## Build

```bat
build\\\_msvc.bat
```

It uses the same MSVC 14.44, CMake, Ninja, OpenCV, Eigen, Ceres and Boost roots as `ALI-VINS`.  Output:

* `bin/vins\\\_run.exe` — headless estimator;
* `bin/vins\\\_app.exe` — the same Win32 UI as ALI-VINS.

## Run

```bat
bin\\\\vins\\\_run.exe --config config\\\\vins\\\_neighborhood\\\_3.yaml ^
  --manifest config\\\\data\\\\neighborhood\\\_3\\\_manifest.txt ^
  --imu "E:\\\\jo\\\_gvil\\\\jo\\\_gvil 1.0\\\\build\\\\inputs\\\\fs\\\_full\\\\imu.csv" ^
  --out sol\\\\run.txt --frames 6000
```

Set `OV\\\_NOISE=2` and `OV\\\_SEED=1` to match the experiment protocol.  The UI is
`bin\\\\vins\\\_app.exe`: it lists the four configurations (`neighborhood\\\_3`, `neighborhood\\\_3\\\_ali`,
`office\\\_loop\\\_5`, `office\\\_loop\\\_5\\\_ali`), runs the selected one and scores it with
`tools/gauge\\\_free\\\_score.py`.

## Validation (4Seasons, seed 1, `OV\\\_NOISE=2`, 6000 frames)

|Sequence|VINS-Mono|VINS-Mono + ALI|
|-|-:|-:|
|neighborhood\_3|11.66 m|11.09 m|
|office\_loop\_5|25.96 m|25.42 m|

ATE is the gauge-free score (yaw + translation, scale fixed at 1) from `tools/gauge\\\_free\\\_score.py`.
The path ratios were 0.940/0.941 for neighborhood\_3 and 0.964/0.937 for office\_loop\_5.

These are single-seed checks, not the 10-seed Monte Carlo.  VINS-Mono is sensitive to the
initialization window and to the feature/keyframe sequence; repeated seeds can change the ATE
materially, so the table is a port/stability check, not a conclusion about ALI.  In this seed
ALI reduced the ATE slightly on both sequences (about 5% and 2%); one seed is not enough to
claim a significant effect.

## Licences

VINS-Mono is GPL-3.0; see `LICENSE`.  The vendored camodocal sources are part of VINS-Mono.  OpenCV,
Ceres, Eigen and Boost are used as external dependencies; see `THIRD\\\_PARTY\\\_NOTICES.md`.

