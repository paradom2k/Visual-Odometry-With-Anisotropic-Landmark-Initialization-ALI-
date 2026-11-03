# OpenVINS with Anisotropic Landmark Initialization (ALI)

This is a clean, distributable build of a visual-inertial odometry branch based on
**OpenVINS** (`ov_core`, `ov_init`, `ov_msckf`) with **Anisotropic Landmark
Initialization (ALI)**.  The frontend remains FAST + pyramidal Lucas–Kanade;
ALI changes only the covariance used when a SLAM landmark is initialized.

The repository contains:

- the patched OpenVINS source under `thirdparty/openvins/`;
- the Windows application/driver in `src/`;
- a prebuilt Windows x64 runtime in `bin/`;
- two 4Seasons run configurations in `conf/`;
- the trajectory scorer and bag helpers in `tools/`.

The datasets are not included.

## ALI in one paragraph

For a candidate landmark \(i\), the frontend computes the structure tensor
\(M_i\) over its local window and its eigendecomposition
\(M_i=U_i\operatorname{diag}(\lambda_{1,i},\lambda_{2,i})U_i^\top\).  ALI
stores an anisotropic \(2\times2\) covariance multiplier in
`Feature::init_cov`:

\[
C_i=U_i\operatorname{diag}\left(
\operatorname{clip}\left(\frac{\bar\lambda_2}{\lambda_{1,i}},0.25,4\right),
\operatorname{clip}\left(\frac{\bar\lambda_2}{\lambda_{2,i}},0.25,4\right)
\right)U_i^\top,
\qquad
R_i^{\mathrm{init}}=\sigma_{\mathrm{pix}}^2 C_i .
\]

The delayed SLAM initialization consumes this matrix instead of the nominal
isotropic covariance.  The anisotropic system is Cholesky-whitened before
`StateHelper::initialize`, because that routine assumes isotropic noise.

## Quick start on Windows

The prebuilt executables are in `bin/`.  Run from the package root:

```bat
bin\jo_vi_corner_app.exe --list
bin\jo_vi_corner_app.exe --run conf\4seasons_full.conf
bin\jo_vi_corner_app.exe --run conf\4seasons_office.conf
```

The two configurations point at the 4Seasons files on the development machine.
Edit `dataset_root`, `manifest`, `imu`, and `reference` in the relevant
`conf/*.conf` before running elsewhere.  The app checks these paths and prints
`[ok]` or `[MISSING]`.

To enable ALI for a run, set the frontend environment before starting the app:

```bat
set OV_WEIGHT=l2cov
set OV_WALPHA=1
set OV_WCLAMP=0.5
set OV_WGROUP=slam
set OV_WINIT=1
bin\jo_vi_corner_app.exe --run conf\4seasons_full.conf
```

Leave `OV_WEIGHT` unset for the baseline.  `OV_NOISE=<grey-level sigma>` and
`OV_SEED=<integer>` optionally add reproducible per-frame Gaussian image noise.
The Python helpers need Python 3 with `numpy`; `bag_extract.py` additionally
needs `Pillow`.

## Building from source

Windows/MSVC dependencies: OpenCV, Eigen, Ceres (built install), and Boost.
Then:

```bat
build_msvc.bat [OpenCV root] [Eigen root] [Ceres root] [Boost root] [jobs]
```

or configure CMake/Ninja manually; see `build_msvc.bat` and
`cmake/openvins.cmake`.  Build products are written to `bin/`.  Do not use a
clean build to test a one-file change: the OpenVINS kernel translation units
dominate the build time.

## What changed

See [`CODE_CHANGES.md`](CODE_CHANGES.md) for the complete change list and
[`patches/ali-source.patch`](patches/ali-source.patch) for the source diff
against commit `b78fa4c`.  The main ALI changes are in:

- `thirdparty/openvins/ov_core/src/feat/Feature.h`
- `thirdparty/openvins/ov_core/src/track/TrackKLT.{h,cpp}`
- `thirdparty/openvins/ov_msckf/src/update/UpdaterSLAM.cpp`

The same tree also contains the experiment hooks used for selection,
forward–backward, displacement, and per-feature-weight studies.  They are
inactive unless the corresponding `OV_*` environment variables are set.

## License

The combined work is distributed under **GPL-3.0** because the vendored
OpenVINS kernel is GPL-3.0.  See [`LICENSE`](LICENSE) and
[`THIRD_PARTY.md`](THIRD_PARTY.md).  The prebuilt `bin/` contains third-party
runtime DLLs; their notices are listed in `THIRD_PARTY.md`.

## Data and provenance

The 4Seasons data, image manifests, IMU files, and groundtruth are not part of
this repository.  The two `conf/` files document the expected manifest/IMU/
reference layout and can be pointed at a local 4Seasons extraction.  The
`bag_extract.py` helper can build those files from a ROS bag when needed.
