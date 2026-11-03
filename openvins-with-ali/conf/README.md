# 4Seasons run configurations

Two sequence configurations are included:

- `4seasons_full.conf` — the full 4Seasons drive.
- `4seasons_office.conf` — the 4Seasons office-loop recording.

The `settings =` file for each configuration names the corresponding OpenVINS
YAML, camera calibration, and IMU noise model in this directory.  The dataset
paths (`dataset_root`, `manifest`, `imu`, `reference`, and optionally `bag`)
are absolute examples from the development machine.  Update them for your
local extraction before running.

For ALI, set the frontend environment before launching:

```bat
set OV_WEIGHT=l2cov
set OV_WALPHA=1
set OV_WCLAMP=0.5
set OV_WGROUP=slam
set OV_WINIT=1
```

Without `OV_WEIGHT`, the configuration runs the OpenVINS baseline.  The
datasets are not included in this repository.
