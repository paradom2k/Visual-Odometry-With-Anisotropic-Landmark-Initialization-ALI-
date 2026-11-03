# Code changes

Base: the vendored OpenVINS tree in this repository, commit `b78fa4c`.
The source tree is **already patched**; `patches/ali-source.patch` is provided
for provenance and must not be applied on top of this tree.

## ALI / landmark covariance

| File | Change |
|---|---|
| `thirdparty/openvins/ov_core/src/feat/Feature.h` | Added per-feature `sigma_mult`, init-only `sigma_mult_init`, and anisotropic `init_cov` (`Eigen::Matrix2d`). |
| `thirdparty/openvins/ov_core/src/track/TrackKLT.{h,cpp}` | Added structure-tensor helpers (`eig2`, `tensor_eig`, `tensor_cov`), the running `l2_ref_`, and the `OV_WEIGHT=l2cov` branch. `tensor_cov` forms \(U\operatorname{diag}(\mathrm{clip}(\bar\lambda_2/\lambda_1,0.25,4),\mathrm{clip}(\bar\lambda_2/\lambda_2,0.25,4))U^\top\) and stores it in `Feature::init_cov`. The scalar λ2, λ2-init, FB, d1 and selection branches are also present for the experiment comparisons. |
| `thirdparty/openvins/ov_msckf/src/update/UpdaterSLAM.cpp` | Per-feature noise multiplier at delayed initialization; the anisotropic `init_cov` is copied into each 2×2 residual block; the local system is Cholesky-whitened before `StateHelper::initialize`, which requires isotropic noise. The SLAM update path also applies the scalar multiplier. |
| `thirdparty/openvins/ov_msckf/src/update/UpdaterMSCKF.{h,cpp}` | Per-feature sigma support for the MSCKF update: rows are whitened by their own variance before measurement compression, and the compressed covariance is set to identity. Diagnostics for offered/accepted/χ²-rejected features were added. |
| `thirdparty/openvins/ov_msckf/src/core/VioManager.cpp` | Optional `OV_SLAMQ` ordering for SLAM anchor slots and the `[UPD]` acceptance/χ² diagnostic line. |
| `conf/openvins_4seasons.yaml` | Clone window raised from 11 to 25; dynamic-initializer iteration/time caps raised for deterministic convergence. |

The ALI path is selected at runtime:

```bat
set OV_WEIGHT=l2cov
set OV_WALPHA=1
set OV_WCLAMP=0.5
set OV_WGROUP=slam
set OV_WINIT=1
```

With `OV_WEIGHT` unset, the estimator uses the nominal isotropic covariance.
The other `OV_WEIGHT` values (`l2`, `l2init`, `fb`, `fb_rel`, `p1`, `p1init`,
`p3`, etc.) are experiment controls and are not required for ALI.

### Init-only d1 note

`OV_WEIGHT=p1init` originally did not compute `d_med`, so that branch was
inert. The condition now covers both `p1` and `p1init`; `p1init` sets only
`sigma_mult_init`. This correction affects the d1 experiment, not ALI.

## Driver / application / tools

| File | Change |
|---|---|
| `src/openvins_run.cpp` | Manifest paths may contain spaces; optional reproducible image noise (`OV_NOISE`, `OV_SEED`); fail early when the estimator configuration is missing. |
| `src/app_main.cpp` | Failed estimator runs no longer print or attempt to score a non-existent trajectory. |
| `tools/gauge_free_score.py` | Missing estimate/reference files are reported directly instead of producing a Python traceback. |
| `bin/openvins_ali_run.exe` | Rebuilt from the source above, including the ALI and d1 changes. |
| `bin/openvins_ali_app.exe` | Application/driver launcher (no estimator changes). |

## Vendored OpenVINS

The kernel is upstream OpenVINS with ROS nodes, stand-alone entry points and
tests removed.  The original vendoring also rewrote five `PRINT_*` macros in
`ov_core/src/utils/print.h` from GNU named-variadic form to MSVC-compatible
`(...)`/`__VA_ARGS__` form.  That edit is part of the baseline tree and is not
counted in the ALI diff.

## Reproduction scope

The two checked-in configurations are the 4Seasons full and office runs.
Dataset paths in those files are absolute examples from the development
machine; edit them for a local extraction.  The datasets themselves are not
distributed here.
