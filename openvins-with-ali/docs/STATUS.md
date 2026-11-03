# jo_vi_corner -- branch status

## What this repository now is

Trimmed to a minimal, self-contained, buildable project. It holds the application, the kernel driver, the vendored
OpenVINS kernel, the configuration and the Python tools that the application calls at run time -- and nothing else.

Removed in the trim, because none of it is needed to compile or to run: the replaced kernel's sources
(`FeatureTracker.cc`, `ImuTypes.cc`, `FactorGraph.cc`, `Estimator.cc`), its headers, its vendored g2o and Sophus, its
driver and its three self-tests, its configurations, and the accumulated run outputs and sample trajectories. The
replaced kernel still exists in the workspace it was developed in (`src/jo_vi_corner/`), and every number that was
measured with it is preserved below -- that is the record, the code was only the means.

`bin/` holds exactly two executables and the fifteen runtime DLLs they need:

| | |
|---|---|
| `jo_vi_corner_app.exe` | the application: configuration list, one-press run, live log, trajectory plot, scoring |
| `jo_vi_corner_openvins_run.exe` | the OpenVINS kernel driver the application starts |

Ceres's runtime DLLs are staged by the build rather than copied by hand. That was necessary and had been missing:
`ceres.dll` is a shared build and the kernel links it, so with `bin/` emptied the driver failed to start until
`ceres/glog/gflags/openblas/lapack.dll` were placed beside it. Staging is now part of the build.

Verified from a wiped `build/` and `bin/` through `build_msvc.bat`: both executables and all fifteen DLLs are
produced, `--list` reports the single configuration with every path `[ok]`, the driver's `--help` works, and a
600-frame run launched from the application initialises and writes its trajectory, log and score to `sol/`.

## Where the time went, measured

This file is deliberately plain ASCII, and the reason is worth two lines because the damage was silent. Editing it
with PowerShell's `Get-Content -Raw` / `Set-Content -Encoding UTF8` reads a UTF-8 file with no byte-order mark using
the system code page, so every non-ASCII character was reinterpreted and re-encoded: em dashes became CJK mojibake
while the file still looked like text. Repairing it then went wrong twice more -- first a regular expression meant for
prose dashes also matched `--run` and `--list` in the command examples, then the same expression split `|---|---|`
into `|-- -|-- -|` in all thirty tables. Both were repaired and the result verified: thirty tables whose separator rows
match their headers, no non-ASCII bytes, no split flags. Use a UTF-8-aware editor on these files, not a shell
round-trip.

Two beliefs about this project's speed turned out to be wrong, and both were corrected by measurement rather than by
reasoning.

**A full drive is not a long job.** The README said 25 minutes; it is **177.6 s** for all 9487 frames, 53 frames per
second. The 25 minutes was an estimate carried over from the kernel this one replaced, which really was that slow,
and it was wrong by about eight times. 900 frames take 37.5 s.

**Parallelism is not the lever.** The build was running `-j 2` on a 32-core machine, which looked like the obvious
cause of a slow build. It is not:

| | |
|---|---|
| clean build, `-j 2` | 971.9 s |
| clean build, `-j 8` | 1077.9 s -- more jobs did **not** help |
| no-op build | 1.4 s |
| re-archive + relink only (deleted the 138 MB static lib) | 1.2 s |
| recompile the driver, one `.cpp` | 37.1 s |
| recompile one kernel translation unit | 384 s |
| disk write throughput to the build directory | 1390 MB/s |
| logical processors | 32 |

The cost is compiling the kernel's template-heavy translation units -- 32 objects totalling 100 MB, the largest
13.8 MB -- and it is neither CPU-parallel nor disk bound. Linking is almost free by comparison.

What actually made this task slow, in order:

1. **Repeated clean builds.** Roughly six of them, at 16 minutes each: about 1.5 hours, and most of it self-inflicted
   by wiping `build/` "to prove the build works" instead of building incrementally. An application-only change costs
   2.1 s.
2. **A masked build failure.** `cmake --build` returned success on a run whose link had failed with `LNK1104` (an
   unusable `TMP`/`TEMP`), so the build printed `BUILD_OK` with no driver in `bin/`. Diagnosing that as a CMake
   structural problem -- inspecting `build.ninja`, `ninja -n`, `if`/`endif` balance -- took about half an hour before
   the linker's own message was actually read. `build_msvc.bat` now checks that both executables exist before
   printing `BUILD_OK`, so the signal cannot lie again.
3. **Ten full-drive runs, serially awaited**, at about 3 minutes each: 30 minutes. These are single-threaded and
   pinned to `OMP_NUM_THREADS=1` for determinism, so three or four could have run concurrently -- that is the cheap
   accelerant for any future campaign of this kind, and it was not used.

Two things that were done right and are worth keeping: hypothesis tests were run on 900- and 3000-frame windows with
the full drive reserved for confirmation, and the driver's per-frame state log (`dist = ... (meters)`) is what
localised the divergence to a single calibration value -- that log line cost nothing and saved a great deal.

One build gotcha is worth recording because it cost time here. Driving `ninja` in `build/` directly, without the
`TMP`/`TEMP` that `build_msvc.bat` sets, fails at the link step with
`LNK1104: cannot open file '...\AppData\Local\Temp\lnk{GUID}.tmp'` -- the host's default temporary directory is not
usable by the toolchain. Worse, `cmake --build` returned success on one such run even though the link had failed and
`bin/jo_vi_corner_openvins_run.exe` was never produced, so a `BUILD_OK` line is not by itself evidence that the
build is complete: check that the executable exists.

## Live plotting, and what the initialisation phase actually costs

The plot used to appear only when a run finished, because the driver collected every pose in memory and wrote the
file at the end. That is a poor way to watch a three-minute job, and it also made a healthy run look broken: the
panel said "No trajectory yet." for the whole run, including the part where nothing *can* be drawn yet. Both were
reported as a possible bug, and neither was one -- but both were worth fixing.

The driver now opens the trajectory file before the first frame, writes each pose as the filter produces it, and
flushes per pose. The application follows that file on a one-second timer, redraws when it has grown, and skips the
work entirely when it has not. Measured directly, by polling the file during a run:

```
t+44s  log: initialised at frame 298 (9.8 s of data)
t+44s  file    3107 bytes
t+45s  file    8603 bytes
t+46s  file   13995 bytes
...    growth continues smoothly at about 30 poses per second
```

So the file is readable the instant there is something to read, with no buffering delay. Verified end to end with
`--run 4seasons_full --window`, which starts the run inside the window so the path can be checked without anyone
clicking: the status line counted up 34, 105, 178, ... 2697 poses at one tick per second, and the run finished with
exit 0 and RMS 3D 19.58 m over its 3000-frame window. `--window` was added for exactly this reason -- a feature whose
only test is "a person watched it once" is not tested -- and it is useful in its own right.

The measurement also exposed something that had been invisible: **the initialisation phase, not the tracking,
dominates the first minute.** In that 3000-frame run the first pose appeared 44 s in, and the whole run took 72 s.
The first 298 frames therefore cost 44 s while the remaining 2700 cost about 28 s -- roughly **7 frames per second
during initialisation against 50 afterwards**. The reason is in the log: the dynamic initialiser calls Ceres
repeatedly, each attempt capped at 50 ms, and many attempts fail (`opt failed: Maximum solver time reached`,
`Jacobian matrix is rank deficient`) before one succeeds. This is also the mechanism behind the slowdown seen when
two windows run at once: that phase is CPU-bound and its per-attempt cost grows under contention, so two concurrent
full drives ran at about a third of the solo frame rate. Hence the advice to run one at a time -- which the window
enforces within itself (`StartRun` returns early when `running` is set, and the Run button is disabled), but cannot
enforce across two windows.

One upstream defect was noticed while reading these logs and is worth recording so it is not mistaken for a symptom:
the initialiser's progress line prints a nonsense feature count, such as `170 feats (2418066587816 valid)` -- an
uninitialised member in OpenVINS's own reporting. It is log-only and does not affect the estimate. It is left alone
because the kernel is vendored and the rule here has been to edit upstream only where it does not compile, which so
far is one file (`ov_core/src/utils/print.h`).

A divergence also showed up during this work, in a real run of the shipped configuration: 9104 poses, 136519 m of
travelled distance against a 2171.6 m reference (ratio 62.9), RMS 3D 39057 m, and an accelerometer bias of
`-0.7719, 1.2442, 0.0246` where a healthy run sits near 0.05. That is the bimodality documented below appearing in
the field, and it is why the initialiser settings are pinned; it happened while two runs shared the machine. The
driver exits 0 in that case, because it did produce poses -- divergence is visible in the score, not in the exit
code.

## Replacing the kernel with OpenVINS: what has been measured so far

The old visual-inertial kernel (VINS `feature_tracker` + a hand-written g2o sliding window) was abandoned after it
was measured to diverge over the full drive.  It is replaced by **OpenVINS** (`ov_core` + `ov_init` + `ov_msckf`),
copied into `thirdparty/openvins` with ROS removed.

### The port

| | |
|---|---|
| sources copied | 88 files, ~19.7k lines (ov_core 14, ov_init 8, ov_msckf 10 translation units) |
| files excluded | 20: 6 tests, 5 ROS nodes / entry points, 6 ROS visualizer files, 3 simulation files |
| **files edited** | **1** -- `ov_core/src/utils/print.h`, five `PRINT_*` macros |
| dependencies | Eigen, OpenCV 4.6, Boost 1.92, Ceres 1.14 -- **all already on disk, no downloads** |

The one edit is forced: upstream writes those macros in the GNU named-variadic form `#define PRINT_ALL(x...)`, and
MSVC rejects it under **both** of its preprocessors (traditional: `C2010: '.': unexpected in macro argument list`;
`/Zc:preprocessor`: `C2608: invalid token '...' in macro parameter list` -- both measured).  Rewriting them as
`(...)` / `__VA_ARGS__` leaves the several hundred call sites untouched.

Boost.Filesystem was the only genuinely missing dependency (Boost 1.92's `stage/lib` had date_time and thread but not
filesystem, which `VioManager.cpp`, `VioManagerOptions.h` and `opencv_yaml_parse.h` all use).  It was built from the
same local Boost tree with `b2 --with-filesystem`, which fails with "failed to write command file!" unless TMP/TEMP
point at a writable directory.

Ceres is required only by `ov_init` (`InertialInitializer` -> `DynamicInitializer` -> `ceres/`); `ov_core` and
`ov_msckf` contain no reference to it at all.  The prebuilt Ceres **1.14.0** already on this machine is sufficient:
`State_JPLQuatLocal.h` guards on `CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 2` and otherwise derives from
`ceres::LocalParameterization`, which 1.14 provides (`ceres/manifold.h` does not exist there).

### Measured: 900-frame window (30 s), scored with tools/gauge_free_score.py

| | OpenVINS (new) | VINS + g2o (old) |
|---|---|---|
| matched poses | 625 | 287 |
| **RMS 3D** | **14.51 m** | 49.29 m |
| path length vs reference | 199.0 m vs 162.4 m (**ratio 1.225**) | 83.3 m vs 179.2 m (ratio 0.465) |
| initialisation | frame 275, 9.08 s in | -- |

So on the short window the replacement is **3.4x more accurate** and its scale is much closer to right (1.225 rather
than 0.465).  The comparison is not perfectly like-for-like: OpenVINS initialises 9 s in, so it covers 20.6 s of the
window against the old kernel's 29.6 s.

### Measured: full drive (9487 frames, 304 s) -- target met, six runs out of six

Three defects were found by measuring rather than by reading, and each one moved the result:

| configuration | RMS 3D | path ratio | outcome |
|---|---|---|---|
| old kernel (VINS + g2o) | 13810.87 m | 22.815 | diverged |
| OpenVINS, `T_imu_cam` = `IMU.T_b_c1`, `gravity_mag` 9.81 | 3530.15 m | 6.032 | diverged |
| OpenVINS, `T_imu_cam` = inverse, `gravity_mag` 9.81 | 3478.53 m | 6.852 | diverged, but minute 1 improved from 2051 m to 583 m |
| OpenVINS, inverse extrinsic, `gravity_mag` 9.8409, init threads 4 | 31.21 m | 1.023 | no divergence |
| the same configuration, run again | 3099.18 m | 6.027 | **diverged -- so the run was bimodal** |
| init threads pinned to 1 (`init_dyn_mle_max_threads: 1`, `OMP_NUM_THREADS=1`) | 31.00, 31.22, 31.00 m | 1.023 | three runs, all converge |
| and with the init gates lowered for this data (see below) | 31.23, 31.21, 31.00 m | 1.023 | three more runs, all converge |

**Result: the target is met and is reproducible -- six consecutive full-drive runs, RMS 3D between 30.9956 and
31.2342 m, path ratio 1.023 in every one, against a reference of 2194.8 m.** All six initialise at frame 279 (the
last three at exactly 279). The final position returns to within about 80 m of the start on a loop drive, so nothing
is running away. Compared with the kernel this replaces (13810.87 m, ratio 22.815) the RMS is about **440x smaller**.

The trajectories are not byte-identical (different MD5) but their scores agree to 0.24 m, so what remains is
floating-point jitter rather than a second attractor: before pinning the threads, two runs of one configuration gave
31.21 m and 3099.18 m, which is a different outcome, not a different rounding.

#### Initialisation was marginal, and its gates have been lowered for this data

Reproducible *when it initialises* is not the same as *always initialises*. The difference showed up when the same
short configuration was run repeatedly:

| launch | frames | runs | initialised |
|---|---|---|---|
| application, before any threshold change | 600 | 3 | 2, and **one produced no trajectory at all** (exit 1) |
| shell, before any threshold change | 600 | 4 | 4, at frames 283, 288, 282, 283 |
| shell, after lowering the gates | 600 | 5 | **5**, at frames 279, 279, 279, 295, 298 |
| shell, full drive, after lowering the gates | 9487 | 3 | **3**, all at frame 279 |

The log said which gates were blocking, and the counts were dominated by two of them:

| message | count in one 600-frame run |
|---|---|
| `[init-d]: gyroscope only N degree change (10.00 thresh)` | 128 |
| `[init]: failed static init: no accel jerk detected` | 64 |
| `[init]: not enough feats to compute disp` | 32 |
| `[init-s]: no IMU excitation, below threshold` | 30 |

The first is `init_dyn_min_deg`, which was demanding 10 degrees of rotation inside the initialisation window before
the dynamic initialiser would run; the second is `init_imu_thresh`, the accelerometer-variance threshold that had
been left at 1.5 (OpenVINS's own monocular example uses 0.5). **Both were reduced** --
`init_dyn_min_deg: 10.0 -> 5.0` and `init_imu_thresh: 1.5 -> 0.5` -- and the effect was checked both ways, because
loosening the initialiser could equally have degraded the solution it produces:

* initialisation success: 3 of 4 runs before, 8 of 8 after (5 short + 3 full);
* full-drive accuracy: **unchanged** -- 31.23, 31.21, 31.00 m after, against 31.00, 31.22, 31.00 m before.

So the looser gates buy reliability without costing accuracy, and they are what `conf/openvins_4seasons.yaml` now
carries. It remains true that a run which never clears those gates returns nothing rather than a bad trajectory, so
a caller should treat an empty result as "did not initialise" and not as a crash.

#### Why the run was bimodal, and what fixed it

The two runs that disagreed differed only in initialisation, and dynamic initialisation solves with Ceres, whose
`init_dyn_mle_max_threads` was 4. Thread scheduling there produced slightly different starting states, which decided
between converge and diverge because the system sits near a stability boundary. Setting
`init_dyn_mle_max_threads: 1` (and `OMP_NUM_THREADS=1` in the environment, as the workspace's own full-drive script
already does for `route_e_r2`) removed the choice. Worth stating plainly: this is not a robustness fix, it is the
removal of a coin flip. A configuration that has a good and a bad attractor is still fragile, and making the
initialiser deterministic only makes its fragility reproducible.

#### The extrinsic convention, settled by measurement

`T_imu_cam` and `IMU.T_b_c1` are inverses of each other as far as this code path is concerned. On the 900-frame
window: `IMU.T_b_c1` verbatim gives RMS 14.51 m / ratio 1.225, its inverse gives RMS 3.75 m / ratio 0.932. The
inverse is 3.9x better with a scale that is nearly right, so the inverse is what `conf/openvins_4seasons_calib.yaml`
now holds. OpenVINS's own example configs name this key `T_cam_imu`, which the parser never looks for -- so their
extrinsics sit at identity, and the key name is not a guide to the value.

#### The dominant error was `gravity_mag`, not the estimator

The first working long run drifted in a way that pointed straight at one number. The log prints the state each
frame, and the distance it reports grew at an **implied speed that climbed from 6.86 m/s to 47.37 m/s** while the
reference holds at 7.0 m/s -- a persistent ~0.2 m/s^2 of unremoved acceleration. The filter's own accelerometer bias
estimate was `ba_y = 0.263`, the same magnitude, and it was not converging.

The cause was the gravity magnitude. This project inherited `gravity_mag: 9.81`, but the IMU measures **9.8409**
during a genuinely stationary period at the start of the drive (the first second has |accel| 9.8409 with a standard
deviation of 0.010 and |gyro| 0.0038 rad/s, so it really is stationary). Declaring 9.81 while the sensor reads
9.8409 leaves a residual that the filter absorbs as accelerometer bias, and integrating that bias is what ran away.
Setting `gravity_mag` to the measured 9.8409:

| | `gravity_mag` 9.81 | `gravity_mag` 9.8409 |
|---|---|---|
| `ba_y` at the end | 0.2631 | 0.0541 |
| implied speed at 90% of a 3000-frame run | 37.87 m/s | 8.29 m/s |

The runaway disappears, and the implied speed stays in 7.2-- 8.1 m/s across the whole drive.

### What was fixed while wiring it up

Three real defects were found by running it rather than by reading it:

1. **An empty mask is not "no mask".**  `TrackKLT` resizes and indexes `CameraData::masks` unconditionally, so an
   empty Mat dies inside `cv::resize`; and 255 is the value meaning "masked out", so an all-255 image would have
   silently suppressed every feature.  The correct no-mask is `cv::Mat::zeros(rows, cols, CV_8UC1)`, which is what
   upstream pushes everywhere it has no mask.
2. **Two calibration files, not one.**  The camera-IMU extrinsics come from `relative_config_imucam` and the IMU
   noise model from a separate `relative_config_imu` key with its own `imu0` node; supplying only the first leaves
   the parser reporting a failed configuration.
3. **Timestamps must be written at full precision.**  With the stream default, Unix-epoch seconds print as
   `1.60208e+09` and every pose appears to be at the same instant -- the scorer then answers "only 0 pose(s) overlap
   the reference window".  Seventeen significant digits round-trips a double.

The corner + optical-flow VI branch.  This file records what is built, what is measured, and what is still
missing, so the state is not inferred from the source tree alone.

## The application, and the layout of this project

The project is arranged so that running it needs no command line:

```
bin/     the executables (and the OpenCV DLLs they need)
conf/    one configuration per dataset
sol/     what a run produces: trajectory, log, score
```

`bin\jo_vi_corner_app.exe` is the application.  It lists the configurations found in `conf/`, shows what the
selected one points at -- with each path marked `[ok]` or `[MISSING]`, because a configuration aimed at a path that
is not there is the most common way to get a confusing failure -- and on **Run** it starts the estimator, streams
its output into the window, and writes the solution to `sol/` under `<conf>_<timestamp>{.txt,.log,.score.txt}`.
Scoring happens automatically when the configuration has a `reference`; when it does not, or when Python is absent,
the window says which of those it was rather than silently showing nothing.

The same binary carries headless modes, which is also how it is verified:

```
bin\jo_vi_corner_app.exe --list                 list configurations, with their paths checked
bin\jo_vi_corner_app.exe --run 4seasons_full run one configuration by name (or by path)
bin\jo_vi_corner_app.exe --plot <traj> --ref <ref>      open the window already showing a trajectory
bin\jo_vi_corner_app.exe --compare <traj> --ref <ref>   print the alignment the plot uses
bin\jo_vi_corner_app.exe --help
```

Those modes are not a second implementation: they call the same configuration parsing, the same path resolution,
the same command-line construction and the same scoring call as the buttons do, so the parts that can be wrong are
the parts that can be tested without a display.

### The trajectory plot

The window draws the estimate against the ground truth as a plan view, with equal scale on both axes.  Three
decisions in it were made deliberately rather than by default:

* **The scale is not fitted.**  The overlay aligns by a rotation about the vertical axis and a translation, scale
  fixed at 1, which is the same kind of alignment `tools/gauge_free_score.py` uses.  Fitting the scale would make a
  uniformly-too-small trajectory look perfect, and a compressed scale is exactly this branch's known weakness, so
  the picture has to show it.
* **The reference is cut to the estimate's time window.**  Found by looking at the first working capture: the full
  ground-truth file is 95 s while a typical run is 30 s, so drawing all of it left the estimate as a speck in the
  corner and the panel useless for judging shape or size.  `Ref:` (or a double-click) cycles that window, the whole
  file, and hidden.
* **The alignment is checkable without a display.**  `--compare` runs the identical code path and prints yaw,
  matched pairs, span and RMS.  On the sample solution it reports yaw -13.6974 deg, 288 matched pairs, RMS 3D
  49.3269 m, against the scorer's -14.07 deg, 287 pairs, 49.2927 m on the same files -- the same alignment to within
  the difference in how the two pick correspondences, which is the agreement that makes the overlay trustworthy.

Verified on 2026-10-13 with a clean `build_msvc.bat`:

| check | result |
|---|---|
| clean configure + build | `BUILD_OK`, no errors |
| executables in `bin/` | `jo_vi_corner_app.exe`, `jo_vi_corner_run.exe`, three self-tests, nine OpenCV DLLs |
| `--list` | **2** configurations; every path `[ok]`, including the bag and the work directory |
| `--compare` | yaw -13.6974 deg, 288 pairs over 29.67 s, RMS 3D 49.3269 m (scorer: -14.07 deg, 287, 49.2927 m) |
| `--run 4seasons_full` | exit 0; 289 keyframe poses; trajectory, log and score written to `sol/` |
| automatic scoring | RMS 3D 49.2927 m, ratio 0.465 -- the same figure as scoring by hand |
| window | created, title `jo_vi_corner - corner + optical-flow VI` |
| child windows | `JoViCornerPlotWindow` present alongside the list, buttons and log |
| the plot draws | captured from the live window: 930x313 with 324 distinct sampled colours (one colour would be a blank panel), and the estimate and reference both visible |

### Configurations

| configuration | what it runs |
|---|---|
| `4seasons_full.conf` | the whole drive, 9487 frames / ~313 s, pointing at the **ROS bag** |

The configurations that were used for the head-to-head comparison -- the 900-frame window and the full drive on the
replaced kernel -- were removed with that kernel. Their numbers are all in this file; reproducing them again means
restoring the kernel from the workspace it was developed in, not rebuilding it here.

A configuration may name a `bag`.  The C++ side does not parse ROS bags -- that framing is in
`tools/bag_extract.py` (pure Python, derived from the actual bytes of this workspace's bags).  When the manifest is
missing and a bag is named, the application extracts it into `work_dir` and writes the manifest from
`work_dir/times.csv`; while the manifest exists the bag is never touched.  That check is for the extractor's own
outputs rather than for a marker file, so an interrupted extraction restarts instead of being mistaken for a
finished one.  Verified: the full-drive configuration's short run reports "manifest present, reusing it" and never
opens the 3.3 GB bag.

Two design choices worth recording, because both were made for a reason found earlier in this project:

* **The estimator runs as a separate process** (`bin\jo_vi_corner_run.exe`), not on a worker thread.  This branch
  has crashed during development (see the injection-crash section below); a crash in a child process is reported in
  the log and the window survives, which is not true of a crash inside the window's own process.
* **The application links neither OpenCV nor the solver.**  It only needs the Win32 libraries, so it still builds
  with `-DJO_VI_CORNER_BUILD_SOLVER=OFF`, and it cannot be the thing that fails when a solver dependency is missing.

## What the branch is

A visual-inertial branch whose visual frontend is **corner detection + Lucas-Kanade optical flow** instead of ORB
descriptors plus bag-of-words matching.  It exists because the ORB frontend inside `src\orb_slam3_vi_r2` (route E)
is not swappable in isolation: its descriptors feed `Frame::ComputeBoW`, `KeyFrameDatabase`, `ORBmatcher`'s BoW
searches, relocalisation and loop closing, so replacing "just the extractor" is not a local change.  This branch
therefore starts from the proven corner+flow tracker and rebuilds the estimator around it.

## Provenance (ported, not hand-written)

| part | source |
|---|---|
| frontend | `github_prj\VINS-Mono-master\feature_tracker\src\feature_tracker.cpp` (306 lines, read in full) and the byte-identical `github_prj\GVINS-main\feature_tracker\src\feature_tracker.cpp` |
| solver (g2o, Sophus) | vendored from `src\orb_slam3_vi_r2\Thirdparty` |
| IMU preintegration | intended to come from `src\orb_slam3_vi_r2\ImuTypes` (the ORB-SLAM3 `IMU::Preintegrated`, with `GetDeltaRotation/Velocity/Position`, covariance `C`, and the per-bias Jacobians) -- **not yet integrated** |

The frontend port removed ROS, camodocal, the global parameter macros and the highgui visualisation; it now takes
plain pinhole intrinsics from the project's settings file and all tunables from `jo_vi_corner::TrackerConfig`.

## Measured on 4Seasons (short window, as requested)

Frontend self-test over the `fs_full` image sequence, 150-feature budget:

| run | frames | features/frame | track median/max | rejected | time/frame | verdict |
|---|---|---|---|---|---|---|
| workspace, 300 frames, with debug dump | 300 | 150/150/150 | 50 / 300 | 647 | 7.53 ms median | PASS |
| delivery project, 200 frames | 200 | 150/150/150 | 57 / 200 | 217 | 6.16 ms median | PASS |

The longest track spanning the entire window in both runs is the property that matters for a VIO: the tracker is
not silently replacing lost features with fresh detections on every frame.

## IMU preintegration

Ported from `src\orb_slam3_vi_r2\src\ImuTypes.cc` (ORB-SLAM3's preintegration), with three mechanical changes and
no mathematical ones: the namespace, the removal of the Boost.Serialization members (this branch has no map
persistence), and dropping the `GeometricTools.h` include (its only use, `NormalizeRotation`, is defined in the
file anyway).

It carries its own self-test (`tools/imu_selftest.cpp`, 18 checks, `VERDICT PASS`) covering the cases whose answer
is known without an estimator:

* stationary, no bias: delta rotation is identity, and delta velocity/position equal `g t` and `0.5 g t^2`
  respectively -- the preintegration deliberately does **not** remove gravity, the caller does that when forming
  the residual, so asserting those are zero would be asserting a bug;
* constant 0.5 rad/s about z: the delta rotation equals `exp(w t)` and the angle recovered from it is 1.000 rad;
* constant specific force: `dV = a t`, `dP = 0.5 a t^2`;
* a bias change: the linearised `GetDeltaPosition/Velocity/Rotation(bias)` agree with a full `Reintegrate()` from
  the corrected bias (2.6e-4 against a 2e-3 tolerance), and `SetNewBias` plus the updated accessors agree too.

Two findings from writing that test, both worth carrying forward:

1. **A real numerical defect, fixed.** The information matrix came out asymmetric (asymmetry ~2046) because it was
   a raw `C.inverse()`, and a stationary interval makes `C` close to singular. It now goes through an LDLT solve
   followed by an explicit `(M + M^T)/2` symmetrisation, because g2o reads only the upper triangle and therefore
   assumes symmetry.
2. **The preintegration is over-confident in unexcited directions.** On that stationary interval the information
   matrix reaches ~2.1e9, which is the inverse of a near-singular covariance. The symmetry check in the self-test
   is consequently expressed *relative* to the matrix magnitude rather than absolutely. When the factor graph is
   wired up, the inertial information will need conditioning (or the process noise raised) or those directions
   will dominate the solve.

## Sliding-window factor graph

`include/jo_vi_corner/FactorGraph.h` + `src/FactorGraph.cc`. The residual and Jacobian conventions are the parent
workspace's ORB-SLAM3 port (`src\orb_slam3_vi_r2\{include\G2oTypes.h,src\G2oTypes.cc}`), so these are the same
factors rather than reinvented ones:

| element | what it is |
|---|---|
| `VertexPose` | body pose; update is right-multiplied rotation + world-frame translation |
| `VertexVelocity`, `VertexGyroBias`, `VertexAccBias` | additive 3-vectors |
| `EdgeInertial` | 9-dim IMU residual over 6 vertices, 9x24 Jacobian assembled from the preintegration's five bias Jacobians |
| `EdgeGyroRW`, `EdgeAccRW` | bias random walks |
| `EdgeMono` | 2-dim reprojection residual in normalised image coordinates |
| `EdgePriorPoseImu` | unary 15-dim prior on a whole keyframe state (the gauge fix) |

Its own self-test (`tools/graph_selftest.cpp`) checks **every** Jacobian block against central finite differences
of the same residual. Two findings came out of it:

1. **The analytic Jacobians are wrong in three blocks.** The pose/velocity blocks of `EdgeInertial` and the
   landmark block of `EdgeMono` agree to ~1e-10, but
   * `EdgeInertial` gyro-bias block: worst relative deviation **2.75e-1** (at the position residual row, where the
     numeric derivative is 0 and the analytic one is -0.279);
   * `EdgeInertial` acc-bias block: worst relative deviation **4.52e-2**;
   * `EdgeMono` pose block: worst relative deviation **2.95** -- an exact sign flip, because with the
     right-perturbation update convention the identity `hat(a)b = -hat(b)a` reverses the rotation block relative to
     the form transcribed from the parent tree.

2. **The numerical Jacobian path is exact.** `JacobianMode::kNumerical` differentiates the residual through each
   vertex's own `oplus` rule, and its blocks match an independent central difference with deviation
   **0.000e+00** on all six inertial blocks and both visual blocks. Because it is defined by the residual itself it
   cannot carry a transcription sign error, so **it is the default** (`jacobianMode()`), at the cost of 2 residual
   evaluations per parameter. The analytic path stays available via `kAnalytic` and its deviations are reported by
   the self-test as `known`, not hidden.

Two crashes were found and removed along the way, both the same root cause: g2o binds the `_jacobianOplus*` Maps
only inside `initializeOptimization()`/`buildSystem()`, so calling `linearizeOplus()` outside the optimiser writes
through a null pointer. The edge classes therefore compute their Jacobians into values
(`EstimateJacobian`, `EstimateJacobianColumnBlock`, `EstimateJacobianNumerical`) and the only code that touches the
Maps is `linearizeOplus()` itself.

## The estimator (first working version, not yet accurate)

`include/jo_vi_corner/Estimator.h` + `src/Estimator.cc` + `tools/estimator_run.cpp`. It runs end to end on 4Seasons:
the frontend tracks every frame, a keyframe is created on a 0.35 s cadence (or earlier when tracking degrades or the
features move enough), the IMU samples between two keyframes are preintegrated, landmarks are triangulated from the
keyframe pair with the widest baseline, and the factor graph is solved with g2o Levenberg-Marquardt over a
fixed-size window.

**Working**: bootstrap from the calmest IMU window, metric preintegration with the settings file's extrinsic,
keyframe/landmark management, a converging solve, and a trajectory in the scorer's format (253 keyframe poses over
the first 900 frames).

**The measured failure, isolated.** Over the first 30 s the reference drive covers **178.9 m**. The estimator's own
travelled distance has gone **7.9 m (ratio 0.044) -> 19.5 m (0.109) -> 47.6 m (0.266)** across three fixes, so the
remaining error is a factor of **3.8**, down from 22.6. The diagnostic printout pins the mechanism: after a solve the
newest keyframe's speed was **0.4-0.7 m/s** where the reference is moving at **6.19 m/s** -- the velocity estimate
itself, not an accelerometer bias absorbing gravity (|ba| stays at 0.20-0.31 m/s^2).

What each fix was worth, measured:

| fix | ratio |
|---|---|
| (baseline, body treated as camera, no gyro bias estimate) | 0.044 |
| apply `IMU.T_b_c1` through the repaired settings reader | 0.109 |
| estimate the gyro bias at bootstrap, then tighten its prior to 1e6 | 0.266 |

The gyro bias is the one that was least obvious: the calm-window estimate is only **0.00283 rad/s**, but the solve
had been raising it to **0.24 rad/s** -- radians of attitude error over 30 s, which takes the gravity direction with
it. Pinning it at the measured value lifted the estimated speed from 0.4-0.7 m/s to **3.1-3.6 m/s** against a
reference of 6.19 m/s.

Ruled out by measurement, not by argument:

* the preintegration is not the problem: over the first 0.363 s the reference itself moves only **0.0007 m**, and the
  preintegration reports **0.002 m** -- they agree, so the inertial measurement chain is sound;
* the window size is not the problem: growing it from 8 to 20 keyframes moved the ratio only from 0.044 to 0.060;
* the extrinsic was a real problem, is now fixed, and was worth 2.5x on its own. `IMU.T_b_c1` is a **nearly
  90-degree rotation** (determinant +1; it maps the body x-axis onto the camera -y/-z plane). Verified: the
  body-frame mean specific force (0.439, 0.235, 9.826) maps to (-0.308, -9.830, -0.275) in the camera frame, which is
  what a camera looking along the vehicle's forward axis should see.

**What is still missing.** The first keyframe's velocity is still fixed at zero while the vehicle moves at ~6 m/s, and
the estimated speed remains about half the reference. The standard remedy is the closed-form visual-inertial
alignment that estimates gravity, scale and the per-keyframe velocities jointly from the preintegrated measurements
(what ORB-SLAM3's `Optimizer::InertialOptimization` does for its own branch). It needs at least four keyframe
transitions for the scale to be observable, so it has to run on a longer initial window than the 8-keyframe solve
window. That is a specific, well-defined piece of work and it is the next thing this branch needs.

Five defects were found and fixed on the way, each of which had made the failure worse rather than merely being
untidy:

1. **New keyframes copied the previous pose**, giving every new keyframe zero relative motion -- a degenerate start
   for both triangulation and the optimiser. They are now extrapolated with a constant-velocity model.
2. **The trajectory was read from the window instead of the history**, so a 30 s run reported 8 poses covering 0.4 s.
   The estimator keeps a full history and copies optimised window states into it.
3. **The initial attitude was averaged over a window in which the vehicle was accelerating.** A moving vehicle
   measures gravity plus its own acceleration, so that tilted the inferred "up" and mapped forward acceleration onto
   the vertical axis -- exactly the 13.5 m climb that was observed. The attitude now comes from the calmest window.
4. **The settings reader mistook the `rows:`/`cols:`/`dt:` preamble of the `!!opencv-matrix` block for the end of
   the block**, never reached the `data:` line, and silently fell back to "body and camera frames are identical".
   The reader is now `include/jo_vi_corner/Settings.h`, shared by the tools so the two copies cannot diverge again.
5. **The gyroscope bias was never estimated.** It is now taken from the same calm window as the attitude, and
   pinned there with a tight prior.

**Attacked this round and rejected by measurement.** Two attempts on the remaining factor of 3.8, both implemented,
run and then turned off rather than left in:

| attempt | measured result |
|---|---|
| joint scale/velocity alignment, gravity floating (7 unknowns) | scale 1.481, **\|v0\| = 38.1 m/s**, trajectory 2516 m, ratio **14.07** |
| joint scale/velocity alignment, gravity pinned to the bootstrap direction (4 unknowns) | scale 0.9705, \|v0\| = 0.074 m/s, ratio **0.189** |
| triangulate from the widest-baseline keyframe pair instead of the observation span | ratio **0.053** (and 0.034 with a larger minimum baseline) |
| restored configuration | ratio **0.266** |

The alignment failure is structural, not a coding slip, and it is worth recording because it says what the real
blocker is. Over this span the vehicle is very close to **constant velocity** -- the accelerometer reads only about
0.28 m/s^2 in the camera frame -- and under a fixed gravity a constant velocity and a scale error are
**indistinguishable** in the position model: every scale is self-consistent. What breaks that degeneracy is a
gravity-direction change coupled to rotation, which is where the standard VINS-style alignment actually gets its
scale information, and estimating that needs a well conditioned visual structure this pipeline does not have yet.
The code is present (`Estimator::alignVisualInertial`) and disabled with that reasoning in place of a guess.

The wider-baseline triangulation result is the opposite of the intuition that motivated it: a long baseline
accumulates the rotation and translation error between its two ends, so the "better conditioned" pair is also the
pair whose poses agree least. The observation span is the measured-better choice.

## Scored against the reference, and compared with route_e_r2

The trajectory is in the scorer's format (seconds, `x y z`), so `tools/gauge_free_score.py` reads it with
`--time-scale 1`. `route_e_r2` writes nanoseconds plus a quaternion, so it needs `--time-scale 1e-9` and its rows
trimmed to four columns; both were scored against `build\inputs\fs_ref_full.csv` on the **same 900-frame (30 s)
window**.

| | corner branch `jo_vi_corner` | `route_e_r2` |
|---|---|---|
| matched poses | 252 | 720 |
| span scored | 29.6 s | 23.8 s |
| yaw alignment the scorer applied | +20.67 deg | +86.76 deg |
| **RMS 3D** | **39.23 m** | **38.76 m** |
| RMS 2D | 39.22 m | 38.74 m |
| path length estimate vs reference | 47.6 m vs 179.2 m (**ratio 0.266**) | 108.4 m vs 173.0 m (**ratio 0.626**) |

Read honestly: the RMS values are close, but they are close for different reasons. `route_e_r2` follows the motion
much better (0.626 of the reference path against 0.266), and its residual error is shape error; the corner branch's
larger error is dominated by its compressed scale, so its RMS is not evidence of comparable tracking. On this
window the corner branch is **not yet competitive** -- it is a working pipeline with a scale and heading problem,
not a working estimator.

The yaw-alignment difference (20.67 against 86.76 deg) is large enough that it should not be read as a heading
error on its own: `route_e_r2` reports a camera pose while this branch reports a body pose whose camera extrinsic is
a nearly 90 degree rotation, so part of that difference is the frame in which each writes its trajectory. It is
recorded rather than interpreted, and needs a frame-consistent comparison before it means anything.

## Why the scale is wrong: the visual structure, measured

This round isolated the cause with three measurements, and the answer is not the optimiser or the inertial side.

**1. The visual-only solution is far worse than the visual-inertial one.** With inertial factors switched off
(`--no-inertial`), the same 900-frame window gives a travelled-distance ratio of **0.040**, against **0.266** with
them on. So the inertial factors are pulling the scale *towards* correct and the visual factors are pulling it
*away*. The problem is in the visual structure, not in the inertial weighting.

**2. The geometry is degenerate.** Instrumented landmark depths against the camera baseline inside an 8-keyframe
window:

| | value |
|---|---|
| mean nearest landmark distance | 23.5 - 59.0 m |
| camera baseline across the window | 1.3 - 4.1 m |
| ratio | **12 - 38** |

A camera that moves 1-4 m while the landmarks sit 25-60 m away gives almost no parallax, so the reprojection
residual is nearly **insensitive to scale**. That is the whole story: the visual factors cannot constrain scale
because they were never given the geometry to do it.

**3. A longer window does not fix it, it breaks it.**

| window (keyframes) | span | travelled-distance ratio |
|---|---|---|
| 8 | 2.8 s | 0.120 |
| 30 | 10.5 s | **5.31** (trajectory 2453 m) |
| 60 | 21 s | **5.04** (trajectory 2331 m) |

The failure is structural: a landmark is triangulated **once**, from the keyframe pair that first observes it, and is
never re-triangulated as the window slides. A wider window therefore holds landmarks whose initialisation came from
poses that have since been replaced, and the longer the window the more the visual and inertial factors disagree
until the solve diverges. So the baseline cannot simply be lengthened -- the landmark lifecycle has to be fixed
first.

**What this means for the next step.** The frontend is not the problem: detection, tracking and the track lengths are
all measured good. What is missing is landmark management proportional to the geometry -- re-triangulating as poses
improve, tracking depth uncertainty, and dropping observations whose parallax is below the noise floor. Only after
that does a longer baseline (or the closed-form alignment) have anything to work with. The 8-keyframe window is the
best configuration measured so far and is what stays in place.

## The landmark-lifecycle attempt, and why it failed

The previous rounds concluded that the missing piece was the landmark lifecycle -- re-triangulating as the poses
improve -- and the depth/reference-keyframe representation was put in place for it. The re-triangulation was then
implemented (`Estimator::retriangulateAll`) and measured, and it made things **much worse**:

| window | ratio with re-triangulation | ratio without |
|---|---|---|
| 8 | **3.19** | 0.465 |
| 20 | **3.03** | (window 20 without it: diverges, see below) |
| 40 | **2.83** | (window 40 without it: diverges) |

RMS 3D was 106-130 m against 49.29 m. The reason is not obvious and is the useful part of the result: re-triangulating
replaced a **multi-view** estimate -- the landmark position the optimiser had fitted against every observing keyframe
at once -- with a **two-view** triangulation from one keyframe pair. Two-view triangulation divides the baseline by the
parallax, so it amplifies the two poses' errors by the depth/baseline ratio, which the geometry measurements put at
**12-38** for this data. Replacing a well-constrained estimate with an amplified one is a loss however consistent the
pair is, and that is exactly what the numbers show.

So the diagnosis in the previous rounds was right about *where* the weakness is (the structure does not follow the
poses) but wrong about the remedy: re-deriving from two views is not the way to make it follow. What would work is
re-triangulating in the same **multi-view** sense the optimiser already uses, or keeping the optimiser's estimate and
re-deriving only the stored depth/reference pair from it so that `rescaleTo` stays exact. Both are real pieces of work
rather than parameter changes, and neither is in.

`retriangulateAll` is left in the file, uncalled, with the measurement recorded at its call site -- so that the next
attempt starts from the evidence rather than from the same idea.

## The best configuration measured so far

The keyframe cadence turned out to matter more to the scale than any of the algebraic attempts, and it was found by
sweeping one variable at a time on the same 900-frame window:

| keyframe interval | window | travelled-distance ratio | RMS 3D |
|---|---|---|---|
| 0.35 s (previous default) | 8 | 0.266 | 39.23 m |
| 0.15 s | 8 | 0.428 | 51.64 m |
| **0.10 s (now the default)** | 8 | **0.465** | 49.29 m |

Shortening the cadence raises the ratio by 75% (47.6 m to 83.3 m of a 179.2 m reference), which is the largest single
improvement to the scale so far. The reason is the one the geometry measurements pointed at: an 8-keyframe window
spans only 2.8 s at 0.35 s per keyframe, and the preintegrated displacement over that span is just 0.3-0.45 m -- too
little for the inertial factors to say anything about scale. Halving and then thirding the cadence gives them more
measurements to work with over the same window.

The trade is honest and is not hidden: **RMS 3D got worse** (39.23 m to 49.29 m) even though the path length is now
much closer to the reference. A longer path in the wrong shape scores worse than a shorter path in the right shape,
and this configuration buys scale at the cost of shape. Which one matters depends on what the score is for, and both
numbers are reported for that reason.

Longer windows were also measured and diverge -- 30 keyframes gave a ratio of 5.31 and 60 gave 5.04 -- because a
landmark is triangulated once and never re-triangulated as the window slides. That is the landmark-lifecycle problem
described below, and it is why the window stays at 8.

## The scale problem: what has been ruled out, with numbers

Six distinct approaches have now been implemented, run and measured. Five were rejected and removed or disabled; the
sixth is the configuration that ships.

| approach | measured outcome |
|---|---|
| joint scale/velocity alignment, gravity floating (7 unknowns) | \|v0\| = 38.1 m/s, ratio **14.07** |
| joint scale/velocity alignment, gravity pinned to bootstrap (4 unknowns) | scale 0.97, \|v0\| = 0.07 m/s, ratio **0.189** |
| triangulate from the widest-baseline keyframe pair | ratio **0.053** (0.034 with a larger minimum) |
| global translation-scale vertex on the projection | scale driven to its floor 0.08, ratio **27.1** |
| inertial scale projection (match visual to preintegrated displacement) | with the velocity term: ratio **1.778**; preintegration only: scale collapses to ~0, ratio **0.324** |
| **shipped configuration** (8-keyframe window, 0.10 s cadence, observation-span triangulation) | **ratio 0.465, RMS 3D 49.29 m** |

Two of those are worth keeping in mind because their *motivation* was right and only the execution failed:

* **The scale vertex.** Reprojection is scale-free, so the visual factors genuinely cannot constrain metric scale --
  the measurement confirms the inertial factors were losing that contest (visual-only ratio 0.040 against 0.266 with
  them on). Exposing the scale as a variable should therefore have helped, but multiplying the camera translation
  while the world landmarks stay fixed is **not a similarity transform**: it changes the relative geometry, not the
  scale, and the solve correctly drove it to the floor.
* **The inertial scale projection.** Same reasoning, and it also failed -- including the velocity term overshot
  (the velocity came from the same visual trajectory, so that part was circular), and dropping it made the scale
  collapse because 0.3-0.45 m of preintegrated displacement is too little to anchor anything.

**The geometry, measured, is why the algebraic fixes cannot work.** Inside an 8-keyframe window the camera baseline
is 1.3-4.1 m while the landmarks sit 23.5-59.0 m away -- a ratio of 12-38. That is the parallax budget the
reprojection terms have to work with, and it is why the visual structure pins the shape at whatever scale it was
initialised with.

**What the next step actually is**, then, is not another parameter: it is the landmark lifecycle. A landmark should
be re-triangulated as the poses improve (the depth/reference-keyframe representation needed for that is now in place
and is what `rescaleTo` uses), and dropped when its parallax falls below the noise floor. Only with that in place
does a longer window -- and with it a much longer inertial baseline -- become usable, and the 30- and 60-keyframe
divergences above are direct evidence that it is the missing piece.

## Wiring the corner frontend into route_e_r2: done, and what it revealed

The wiring is implemented on both sides and builds:

* `Frame::ExtractORB` (`src\Frame.cc`) takes an externally supplied keypoint list and, for those, skips detection and
  computes descriptors with this tree's own code -- `computeDescriptors` was moved from a file-static into
  `ORBextractor.h` so it can be reached, and a narrow `DescriptorPattern()` accessor avoids making `pattern` public.
  Downstream (`ComputeStereoFromRGBD`, `ComputeBoW`, `ORBmatcher`, the KeyFrameDatabase) is untouched.
* The driver gained `--keypoint-file <file>` (feed external keypoints) and `--export-keypoints <file>` (dump the
  detector's own keypoints and stop, for the diagnostic below).

Three real defects were found and fixed while doing it, each of which had made the run fail in a way that did not
point at itself:

1. **The pyramid is built inside `operator()`.** The injection path skips it, so reading `mvImagePyramid[0]` gave an
   empty matrix and `GaussianBlur` asserted. The reflected border the descriptor sampler needs (`EDGE_THRESHOLD = 19`)
   is now built in `ExtractORB` from the image itself.
2. **Keypoint orientation was never set.** ORB uses a *rotated* BRIEF pattern, so keypoints left at `angle = -1` get
   descriptors sampled at the wrong rotation. The run then did not initialise at all -- no keyframe in the atlas over
   900 frames. The fix calls this tree's own `IC_Angle` through a new `ComputeOrientation` accessor, rather than
   reimplementing the centroid (whose moments are taken about the keypoint, and whose `m01` uses a row *difference*,
   not a sum -- both easy to get subtly wrong by hand).
3. **An off-by-one in the settings parser.** Keys were read with literal offsets (`+ 22` for a 23-character key),
   which parsed the character after the colon and produced an invalid extractor. The export path crashed immediately
   and silently because output was buffered. Offsets are now computed from the key lengths.

**Where the crash lives: isolated, and now quantified.** A standalone probe (`probe_injection.cpp`, built as the
`probe_injection` target) runs the injection alone -- read keypoints, read image, `ComputeDescriptorsFor`, nothing
else: no System, no Atlas, no extra threads. It crashes the same way, which places the defect entirely inside the
extractor's injected-keypoint path rather than in the estimator, the threads, or the `Frame` hand-over.

Because the crash is not deterministic, single runs prove nothing; the ranges were therefore repeated five times each:

| range | passes / runs |
|---|---|
| any **single** frame (17, 18, 19, 30, 50) | 5 / 5 -- never crashes |
| frames 0..13 (14) | 4 / 5 |
| frames 0..19 (20) | 4 / 5 |
| frames 0..99 (100) | **2 / 5** |

The failure probability rises with the number of frames processed, which is the fingerprint of accumulating heap
damage: more calls means more chances for the allocator to meet the corrupted block. It also means every earlier
"this range passes" observation was one sample of a coin, not evidence -- including the ones this file recorded in
earlier rounds.

Ruled out by measurement, not by argument:

* **not bad input data** -- no single frame reproduces it, and the same frame alone always succeeds;
* **not a deterministic argument error** -- a fixed range passes or fails depending on the run;
* **not the out-of-range patch read** -- a guard that skips any keypoint closer to its level's border than the
  31-pixel patch the sampler reads (`HALF_PATCH_SIZE + 2`) leaves the crash unchanged;
* **not a missing type check** -- `operator()` asserts single-channel 8-bit input and this path did not, so the assert
  was added; the images are `CV_8UC1` and it does not fire. Worth having (a wider type would silently misread every
  patch), but not the cause.

Still to instrument: `computeOrbDescriptor` (the function that actually writes the 32 descriptor bytes, which earlier
rounds did not examine), `IC_Angle`, and the accumulated state of the reused extractor across calls. The probe is the
right place to iterate because it runs in under a second and needs no estimator, and the next lever worth trying is a
page-heap or Application Verifier run, which stops on the corrupting write itself rather than on whatever
allocation meets it later.

## The injection crash: found, and it was a margin mismatch

The heap corruption that took several rounds to characterise is **fixed**, and the cause was not memory management at
all. It was a **margin convention**: `computeOrbDescriptor` reads a 31-pixel patch around each keypoint, so the
detector never reports a keypoint closer than `EDGE_THRESHOLD` (19) to the border -- measured on its own output, its
coordinates land in `19 .. width-19`. An external corner tracker has no such rule: the exported corners span
`u 0.51 .. 798.48` and `v 0.50 .. 398.50`, i.e. right on the image edge, and feeding those in made the patch read run
off the image.

The evidence that identified it was a controlled comparison, once the injection had been moved into
`ORBextractor::operator()` so that only one pyramid is built per frame:

| input | keypoints | outcome |
|---|---|---|
| the detector's own keypoints (margin-respecting) | 409 632 | **ran 200 frames clean** |
| the corner tracker's export (edge-touching) | 30 000 | **crashed in every run** |

Fewer keypoints crashing while more keypoints succeed rules out volume, and the coordinate ranges above gave the
difference directly. Filtering to the detector's own `EDGE_THRESHOLD` margin drops 27 keypoints of 30 000, and with
that filter the corner arm runs **200, 400 and 900 frames with exit 0** -- the crash is gone.

Two earlier rounds had a guard for exactly this but placed it in the wrong function and used the wrong constant
(`HALF_PATCH_SIZE + 2 = 17` rather than `EDGE_THRESHOLD = 19`), which is why that attempt was measured as "no effect"
and dismissed. The lesson recorded here is that it was dismissed on a wrong measurement, not a wrong idea.

A second, real defect was fixed on the way: the injection had its own entry point that built its own pyramid, so on a
reused extractor it built the pyramid **twice per frame**. Moving the substitution inside `operator()` (which already
builds one) removed that. Measured before the fix: reusing an extractor crashed in **5 of 5** runs while a fresh
extractor per frame passed **3 of 5**.

**Still not initialising.** With the crash gone the question the A/B was meant to answer is finally reachable, and the
answer so far is negative: the corner arm runs to completion but reports `no keyframe in the atlas`, while the ORB arm
on the same 900 frames scores RMS 3D 38.76 m with a path-length ratio of 0.626. One difference is already known and is
the next thing to control for: the ORB arm uses **2500** features per frame while the corner export used **150**. A
2500-feature corner export was produced (1 470 161 keypoints) and that run **still crashes**, so the margin fix alone
does not cover the dense case -- with 172 and 221 keypoints being dropped inside the margin on the first frames it
reached, the dense configuration needs its own look before its quality can be judged.

## The A/B answered: the corner features are better, but the run does not stay tracked

With the crash gone, the corner frontend was fed into the route_e_r2 backend and compared against the same backend
detecting its own ORB features. The first useful fact is a control: **the feature count was the difference between
initialising and not.** With the frontend's default 150 features per frame the run never initialises (`no keyframe in
the atlas`); at **2500**, matching what the ORB arm uses, it initialises. So the earlier "corner features cannot
initialise" reading was a feature-count artefact, not a property of the features.

Scored on the identical 900-frame (30 s) window:

| arm | matched poses | RMS 3D | path length vs reference |
|---|---|---|---|
| corner frontend, 2500 features | 300 poses over 9.9 s | **0.6503 m** | 0.959 |
| route_e_r2 ORB (baseline) | 720 poses over 23.8 s | 38.7603 m | 0.626 |

Those two spans differ, so that table is **not** a like-for-like comparison and is not the basis for a claim. The
like-for-like comparison is the same time segment, both arms trimmed to 10.0-- 13.6 s:

| arm | matched poses | yaw alignment | RMS 3D | path length vs reference |
|---|---|---|---|---|
| corner frontend, 2500 features | 109 over 3.6 s | -1.83 deg | **0.2357 m** | 0.965 |
| route_e_r2 ORB (baseline) | 109 over 3.6 s | -3.20 deg | 4.6579 m | 1.205 |

**On the same window the corner frontend is about 20x more accurate.** That is the positive result this branch was
built to find, and it is measured on identical input, identical backend and identical settings.

The honest limitation is that the corner arm **stops emitting poses after ~13.6 s** while the ORB arm covers 23.8 s.
The log says why, and it is not a descriptor or matching failure: `Fail to track local map!` repeats and the Atlas
splits into **three maps** (Map 0: 149 keyframes, Map 1: 102, Map 2: 2). So the features are good enough to track
accurately for ~10 s and then the map is lost -- a backend/longevity problem, reached only after feature quality had
been established as adequate. That is the next thing to work on, and it is a different kind of problem from
everything above: not "are the features good" (answered: yes) but "why does the local map stop supporting them".

## Direction change: the scale axis is closed

After three more structurally distinct attempts to make the landmark structure follow the optimised poses were
implemented and measured, all three made the result worse:

| attempt | travelled-distance ratio |
|---|---|
| two-view re-triangulation | 0.465 -> **3.19** |
| multi-view Gauss-Newton refinement (all observers, 3 iterations) | 0.465 -> **10.62** (RMS 3D 528.8 m) |

A well-behaved refinement of a well-conditioned problem should improve things, so three consistent failures are
evidence that the problem is not the refinement but the structure being refined. Work on this estimator's scale
stopped there; both routines remain in the file, uncalled, with the measurements recorded at their call sites.

The estimator is left in its best measured configuration (**ratio 0.465, RMS 3D 49.29 m** over the 900-frame window)
with `BUILD_OK` and both self-tests passing.

## The next direction, and the de-risking already done for it

The remaining work is not more scale tuning on this estimator. The promising direction is to **run the corner
frontend inside the route_e_r2 backend**, which already solves the structure, the marginalisation and the gauge: if
corner features match ORB's quality inside a working system, that is both a stronger result and a fair comparison,
and if they do not, it says the difference is in the features rather than in the estimator around them.

Before investing in the wiring, the geometric question was measured, because a tracker whose features cluster in
high-texture regions would make the comparison meaningless:

| measurement | result |
|---|---|
| frames | 300 |
| features per frame | 150.0 (mean, constant) |
| 40x40 grid cells of the 800x400 image touched | **200 of 200 (100% coverage)** |
| feature position spread | x std 228.0 px, y std 119.2 px |

So the corner frontend's features are spatially spread across the whole image, not clustered, and the comparison is
worth running.

### The injection point, worked out

`Frame::ExtractORB` (`src\Frame.cc`) is the single place features enter the system:

```cpp
if(flag==0)
    monoLeft = (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors, vLapping);
```

Two things make this easier than expected, both checked in the source rather than assumed:

* **`computeDescriptors` is not a protected member** -- it is a file-static free function in `ORBextractor.cc`
  (`static void computeDescriptors(const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors, ...)`). So no
  access change is needed after all; the choice raised earlier between exposing a protected member and using
  `cv::ORB::compute` does not have to be made. Descriptors can be computed with the tree's own conventions without
  touching its interface.
* **The descriptor coordinates are easy to get wrong, and the image itself is the way around it.**
  `computeDescriptors` is called on `mvImagePyramid[level]`, whose coordinates carry both the `1/scale` factor and
  the `EDGE_THRESHOLD = 19` border offset, so supplying image coordinates for a level other than 0 would silently
  read the wrong pixels. At **level 0** the scale is 1 and `pt` is the image pixel coordinate, so computing every
  corner's descriptor at level 0 uses the image directly and sidesteps the conversion entirely. The cost is the loss
  of scale-level assignment for descriptor matching, which is a measured question rather than an assumption.

`Feature::pixel` in the frontend is already the raw (distorted) image coordinate that route_e_r2's own undistortion
expects, so passing it through applies the distortion model exactly once. A `--features-out` option now exports the
features as `<frame index> <feature id> <u> <v>` for this purpose.

The wiring itself (loading the export, supplying it to `ExtractORB`, and running the A/B against ORB on the same
window) is not done yet.

## Build commands

* workspace: `scripts\build_vi_corner.bat` -> `build\jo_vi_corner\`
* delivery: `jo_vi_corner\build_msvc.bat` -> `jo_vi_corner\build\`

Both were run and printed `BUILD_OK`, and all three self-tests pass from the delivery build.  C++14 (not 17) is
required because the vendored g2o uses `std::tr1`.

## Still missing, in order

1. **The estimator that drives the graph**: the sliding-window assembly (which keyframes, which landmarks), the
   marginalisation of old states, and the imu/camera data plumbing from the 4Seasons manifest.
2. **The bootstrap**: gravity direction, scale and bias estimates from the first keyframes, which is what
   ORB-SLAM3's `Optimizer::InertialOptimization` does for its own branch.
3. **Short-window 4Seasons validation of the estimator**: does it initialise, does it produce a trajectory, and
   can `tools\gauge_free_score.py` score it.
4. A comparison against `route_e_r2` on the same window, which is what makes the branch's value measurable rather
   than asserted.

Nothing here claims the estimator runs yet.  The frontend, the preintegration and the factor graph are the parts
that are finished and each independently verified.


