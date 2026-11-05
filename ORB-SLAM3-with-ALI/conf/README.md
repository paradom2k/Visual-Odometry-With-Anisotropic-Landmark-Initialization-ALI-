# Configurations

One file per run.  `ali_run --check-only --config <file>` prints every resolved path and refuses a file
that is missing a required key; `ali_ui` lists the directory.

| File | Sequence | Mode | ALI |
| --- | --- | --- | --- |
| `4seasons_neighborhood_3.conf` | neighborhood_3, 9487 frames | mono-inertial | off (stock ORB-SLAM3) |
| `4seasons_neighborhood_3_ali.conf` | the same | mono-inertial | **on** |
| `4seasons_office_loop_5.conf` | office_loop_5, 13746 frames | mono-inertial | off |
| `4seasons_office_loop_5_ali.conf` | the same | mono-inertial | **on** |
| `campus03_bag_stereo.conf` | GREAT campus03, stereo_bags | stereo-inertial | off |
| `campus03_bag_stereo_ali.conf` | the same | stereo-inertial | **on** |
| `campus01_bag_stereo.conf` | GREAT campus01, stereo_bags | stereo-inertial | off |
| `campus01_bag_stereo_ali.conf` | the same | stereo-inertial | **on** |

**Edit the two input paths before running.**  They are absolute because the recordings are not part of
this repository:

```
images_dir        = .../undistorted_images/cam0
imu               = .../imu.csv
```

`imu` is a text file of `t_seconds,gx,gy,gz,ax,ay,az` (the reader also accepts a ROS1 `.bag`).  The
vocabulary is `ORBvoc.txt` from ORB-SLAM3; it is 145 MB and therefore not vendored.

Only `cam0` is read: these are monocular-inertial configurations (one camera plus the IMU), matching the
single-camera setup of the OpenVINS experiments.  `images_right_dir` and the `camera_right_*` keys are
stereo-only and are absent here; add them and set `mode = stereo-inertial` to run the stereo variant.

These keys are what make a monocular-inertial run survive a driving sequence; each one was measured rather
than guessed, and each is inert where it does not apply:

| Key | Value | Why |
| --- | --- | --- |
| `imu_frequency` | 2000 | the recording's native rate. Writing 200 (a factor of ten) is the single most damaging mistake here: it is the rate the noise densities are converted at |
| `features`, `ini_th_fast`, `min_th_fast` | 2000, 12, 3 | at 1000 features a monocular map holds ~19 points per keyframe and the inlier count sits just above the 30 that `TrackLocalMap` demands, so any perturbation destroys the map; 2000 gives one map and no local-map failures over the full drives |
| `imu_fast_init` | on | the IMU gate waits for a change in acceleration/rotation that a car on a road rarely provides |
| `imu_reset_map_if_not_initialized` | off | upstream discards the active map on the first failed local-map track while the IMU is still initialising, so the two seconds and ten keyframes it needs are never accumulated |
| `relocalize_before_imu_init` | on | while the IMU is not initialised, try `Relocalization()` before declaring the map dead |
| `scale_refinement` | off | the periodic scale re-estimation has a catastrophic mode on these recordings (a small scale at the first refinement destroys the map) |
| `min_initial_parallax` | 8.0 px, mean | the monocular initialiser's disparity gate |
| `prefetch_frames` | off | decoding on its own thread lets the tracker outrun LocalMapping; the monocular local map then starves (measured: with it on, 38 map recreations and 5371 "fail to track local map" over the full neighborhood drive; off, one map and none) |

`conf/data/` holds the ground-truth trajectories (ENU, `t,x,y,z,qx,qy,qz,qw`) that
`tools/gauge_free_score.py` compares against.

## Keys this project reads

`name`, `description`, `mode`, `input`, `images_dir`, `images_right_dir`, `bag`, `topic`,
`right_topic`, camera keys (`camera_model`, `camera_fx`, … , `camera_right_*`, `baseline`, `r_bc`,
`t_bc`), IMU keys (`imu`, `imu_frequency`, `imu_gyro_unit`, `imu_accel_unit`, `imu_noise_gyro`,
`imu_noise_acc`, `imu_gyro_walk`, `imu_acc_walk`, `imu_time_shift`), `vocabulary`, `features`,
`scale_factor`, `levels`, `ini_th_fast`, `min_th_fast`, `start_frame`, `duration`, `debug_every`,
`prefetch_frames`, `viewer`, `solution`, and **`ali`**.

`ali = on` turns on the anisotropic landmark initialization; `ali = off` is stock ORB-SLAM3.  The run
prints which one it used (`[ALI] landmark initialisation: ...`) and, for `on`, the number of landmarks
initialised through the anisotropic path.

The configuration file is the whole interface: there is no compiled default for any input, and an
unrecognised key is reported rather than ignored.
