# OpenVINS with Anisotropic Landmark Initialization (ALI)

This is a clean, distributable build of a visual-inertial odometry branch based on
**OpenVINS** (`ov\_core`, `ov\_init`, `ov\_msckf`) with **Anisotropic Landmark
Initialization (ALI)**.  The frontend remains FAST + pyramidal Lucas–Kanade;
ALI changes only the covariance used when a SLAM landmark is initialized.

The repository contains:

* the patched OpenVINS source under `thirdparty/openvins/`;
* the Windows application/driver in `src/`;
* a prebuilt Windows x64 runtime in `bin/`;
* two 4Seasons run configurations in `conf/`;
* the trajectory scorer and bag helpers in `tools/`.

The datasets are not included.

## ALI in one paragraph

For a candidate landmark (i), the frontend computes the structure tensor
(M\_i) over its local window and its eigendecomposition
(M\_i=U\_i\\operatorname{diag}(\\lambda\_{1,i},\\lambda\_{2,i})U\_i^\\top).  ALI
stores an anisotropic (2\\times2) covariance multiplier in
`Feature::init\_cov`:

\[
C\_i=U\_i\\operatorname{diag}\\left(
\\operatorname{clip}\\left(\\frac{\\bar\\lambda\_2}{\\lambda\_{1,i}},0.25,4\\right),
\\operatorname{clip}\\left(\\frac{\\bar\\lambda\_2}{\\lambda\_{2,i}},0.25,4\\right)
\\right)U\_i^\\top,
\\qquad
R\_i^{\\mathrm{init}}=\\sigma\_{\\mathrm{pix}}^2 C\_i .
]

The delayed SLAM initialization consumes this matrix instead of the nominal
isotropic covariance.  The anisotropic system is Cholesky-whitened before
`StateHelper::initialize`, because that routine assumes isotropic noise.

## Quick start on Windows

The prebuilt executables are in `bin/`.  Run from the package root:

```bat
bin\\openvins\_ali\_app.exe --list
bin\\openvins\_ali\_app.exe --run conf\\4seasons\_full.conf
bin\\openvins\_ali\_app.exe --run conf\\4seasons\_office.conf
```

The two configurations point at the 4Seasons files on the development machine.
Edit `dataset\_root`, `manifest`, `imu`, and `reference` in the relevant
`conf/\*.conf` before running elsewhere.  The app checks these paths and prints
`\[ok]` or `\[MISSING]`.

To enable ALI for a run, set the frontend environment before starting the app:

```bat
set OV\_WEIGHT=l2cov
set OV\_WALPHA=1
set OV\_WCLAMP=0.5
set OV\_WGROUP=slam
set OV\_WINIT=1
bin\\openvins\_ali\_app.exe --run conf\\4seasons\_full.conf
```

Leave `OV\_WEIGHT` unset for the baseline.  `OV\_NOISE=<grey-level sigma>` and
`OV\_SEED=<integer>` optionally add reproducible per-frame Gaussian image noise.
The Python helpers need Python 3 with `numpy`; `bag\_extract.py` additionally
needs `Pillow`.

## Building from source

Windows/MSVC dependencies: OpenCV, Eigen, Ceres (built install), and Boost.
Then:

```bat
build\_msvc.bat \[OpenCV root] \[Eigen root] \[Ceres root] \[Boost root] \[jobs]
```

or configure CMake/Ninja manually; see `build\_msvc.bat` and
`cmake/openvins.cmake`.  Build products are written to `bin/`.  Do not use a
clean build to test a one-file change: the OpenVINS kernel translation units
dominate the build time.

## What changed

See [`CODE\_CHANGES.md`](CODE_CHANGES.md) for the complete change list and
[`patches/ali-source.patch`](patches/ali-source.patch) for the source diff
against commit `b78fa4c`.  The main ALI changes are in:

* `thirdparty/openvins/ov\_core/src/feat/Feature.h`
* `thirdparty/openvins/ov\_core/src/track/TrackKLT.{h,cpp}`
* `thirdparty/openvins/ov\_msckf/src/update/UpdaterSLAM.cpp`

The same tree also contains the experiment hooks used for selection,
forward–backward, displacement, and per-feature-weight studies.  They are
inactive unless the corresponding `OV\_\*` environment variables are set.

## License

The combined work is distributed under **GPL-3.0** because the vendored
OpenVINS kernel is GPL-3.0.  See [`LICENSE`](LICENSE) and
[`THIRD\_PARTY.md`](THIRD_PARTY.md).  The prebuilt `bin/` contains third-party
runtime DLLs; their notices are listed in `THIRD\_PARTY.md`.

## Data and provenance

The 4Seasons dataset is not included in this repository.  Download it from the
official page:

https://cvg.cit.tum.de/data/datasets/4seasons-dataset/download

After downloading, prepare or update the manifest, IMU, and reference files
according to the layout shown in the two `conf/` configurations.  The
`bag\_extract.py` helper can build those files from a ROS bag when needed.

