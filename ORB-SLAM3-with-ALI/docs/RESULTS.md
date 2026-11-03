# Measured results

Two 4Seasons recordings, **monocular-inertial** (single cam0 + IMU), full length, ORB-SLAM3 1.0 with
this project's ALI switch.  Every number is `tools/gauge_free_score.py`: ATE after yaw + translation
alignment with the scale fixed at 1, matched by nearest timestamp.

The stereo-inertial results that predate the switch to monocular are kept in the last section, because
they are what that mode measured.

## What was run

```
bin\ali_run.exe --config conf\4seasons_neighborhood_3.conf      --out sol\<file>.txt
bin\ali_run.exe --config conf\4seasons_neighborhood_3_ali.conf  --out sol\<file>.txt
bin\ali_run.exe --config conf\4seasons_office_loop_5.conf       --out sol\<file>.txt
bin\ali_run.exe --config conf\4seasons_office_loop_5_ali.conf   --out sol\<file>.txt
python tools\gauge_free_score.py --est sol\<file>.txt --ref conf\data\<reference>.csv --time-scale 1
```

`neighborhood_3`: 9487 frames, 309.8 s, reference path 2211.9 m.
`office_loop_5`: 13746 frames, 447.5 s, reference path 3910.6 m.

## Monocular-inertial results

| Sequence | Arm | RMS 3D (m) | RMS 2D (m) | Path ratio | Poses | Maps / lost frames | ALI landmarks |
| --- | --- | --- | --- | --- | --- | --- | --- |
| neighborhood_3 | baseline | 14.955 | 14.898 | 0.953 | 9390 | 1 / 0 | — |
| neighborhood_3 | ALI | **13.992** | 13.927 | 0.962 | 9390 | 1 / 0 | 125 534 |
| office_loop_5 | baseline | — | — | — | — | 17 / 2434 by frame 7200 | — |
| office_loop_5 | ALI | not measured | — | — | — | — | — |

`neighborhood_3` is a clean paired run: both arms hold one map, lose no frame, and produce the same 9390
poses, so the 6.4 % difference is not a tracking difference.  It is still **one run per arm**; ORB-SLAM3
is not bit-reproducible, and the stereo experiments below showed baseline runs of one configuration
differing by 2.2 m.  Treat it as a first measurement.

`office_loop_5` is not stable with this recipe: the baseline re-created the map 17 times and reported
2434 lost frames by frame 7200, so it was stopped.  The same configuration is stable on
`neighborhood_3`; the difference between the recordings was not diagnosed.  Candidate causes, none
tested: the office recording's IMU stream (only the 2000 Hz CSV is available, the bag is not), scene
texture, or the longer loop geometry.

## What made monocular-inertial work at all

Each of these was measured on the neighborhood drive, not assumed:

| Setting | Value | Without it |
| --- | --- | --- |
| `imu_frequency` | 2000 | the CSVs are 2000 Hz; writing 200 (the earlier value) is a factor of ten in the inertial model |
| `features` | 2000 | at 1000 the monocular map holds ~19 points per keyframe and the inlier count sits just above the 30 `TrackLocalMap` needs; measured 16 map recreations and 742 local-map failures over the drive |
| `imu_fast_init` | on | a car rarely changes average acceleration by 0.5 m/s² between frames, which the IMU gate demands |
| `imu_reset_map_if_not_initialized` | off | upstream discards the map on the first failed local-map track while the IMU is still initialising, so the 2 s / 10 keyframes it needs are never accumulated |
| `relocalize_before_imu_init` | on | while the IMU is not initialised, try `Relocalization()` before declaring the map dead |
| `scale_refinement` | off | the periodic scale re-estimation has a catastrophic mode on these recordings |
| `prefetch_frames` | off | with the decoder on its own thread the tracker outruns LocalMapping and the monocular map starves: measured 38 map recreations and 5371 local-map failures on the full drive, against one map and none with it off |
| `Frame::ComputeKeypointCovariances` | called from the monocular constructor | it was only called from the stereo and RGB-D constructors, so ALI had no covariance to use in monocular mode and reported 0 landmarks |

## Why the effect is small in ORB-SLAM3

ALI in the OpenVINS port changes the landmark's **initial covariance in an EKF**, which weights the first
updates of a state that is otherwise initialized from the same two observations.  ORB-SLAM3 has no
per-landmark covariance and no EKF: a triangulated point is immediately re-estimated by local bundle
adjustment over many observations with robust kernels.  The triangulation estimate is therefore an
initial value that the optimizer discards quickly.

In stereo-inertial most landmarks never come from triangulation at all (stereo disparity provides their
depth), which is why ALI was neutral in that mode.  In monocular-inertial every landmark is triangulated
— 125 534 of them on the neighborhood drive — and that is where the 6.4 % was measured.

The model is also a weaker fit here than in the KLT system ALI came from: the covariance is the inverse
structure tensor of the patch, i.e. the localisation uncertainty of a correlation matcher, whereas ORB
keypoints are FAST detections whose position error is closer to isotropic.

A port that kept the stock triangulation and used the anisotropic covariance as the **information matrix
of the creating observations in local BA** would be closer to what ALI does in OpenVINS.  It is a larger
change (the covariance has to travel from `Frame` to `MapPoint` to the g2o edges) and is not implemented.

## Earlier stereo-inertial results (kept for reference)

| Sequence | Arm | RMS 3D (m) | RMS 2D (m) | Path ratio | Poses | ALI landmarks | Note |
| --- | --- | --- | --- | --- | --- | --- | --- |
| neighborhood_3 | baseline | 7.151 | 6.805 | 0.990 | 9244 | — | first binary |
| neighborhood_3 | baseline | 4.960 | 4.918 | 0.988 | 9244 | — | final binary |
| neighborhood_3 | ALI, ungated | 6.007 | 5.927 | 0.992 | 9244 | 98 144 | |
| neighborhood_3 | ALI, gated | 5.724 | 5.516 | 0.988 | 9243 | 48 316 | |
| office_loop_5 | baseline | 4.214 | 2.072 | 1.002 | 13458 | — | first binary |
| office_loop_5 | baseline | 6.545 | 3.515 | 1.005 | 13458 | — | final binary |
| office_loop_5 | ALI, ungated | 216.596 | 216.482 | 1.476 | 13112 | 188 318 | lost tracking at frame 9000 |
| office_loop_5 | ALI, ungated | 9863.115 | 9806.808 | 41.775 | 13298 | 240 718 | lost tracking at frame 13200 |
| office_loop_5 | ALI, gated | 6.563 | 2.602 | 1.002 | 13458 | 69 506 | stable |

"Gated" is the delivered ALI: applied only when the pair has at least ~2.5° of ray parallax
(cos < 0.999), and the refined point is kept only if it lowers the whitened cost and moves less than 10 %
of its depth.  Without the gates, office lost tracking in 2 of 2 runs.

The two baseline runs of the same configuration differ by 2.19 m (neighborhood) and 2.33 m (office);
loop closure succeeding or not is what moves the number by metres.  Against that spread the gated ALI
effect in stereo mode was not measurable.

## Reproducing

`tools/run_mono_full.sh` runs the four full monocular configurations sequentially and scores them.
Runs take 5-10 minutes each on the development machine, and they are **not** bit-reproducible: ORB-SLAM3's
local-mapping and loop-closing threads interleave differently between runs.  Compare arms with the same
binary, and average over repeats before drawing a conclusion.

The trajectories behind the monocular table are `docs/runs/mono_n3_base.txt` and
`docs/runs/mono_n3_ali.txt`; the stereo table's are the other files in that directory.
