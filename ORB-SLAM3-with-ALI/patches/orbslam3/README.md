# The ORB-SLAM3 overlay

```
thirdparty/orbslam3/                 pristine upstream ORB-SLAM3 1.0 source subset
        +
patches/orbslam3/overlay/            every file this project changes or adds (same relative layout)
        =
<build>/thirdparty/orbslam3/         the tree that is compiled
```

CMake copies `thirdparty/orbslam3/` into the build directory and then copies `overlay/` on top, so the
checked-in upstream tree stays byte-identical and `git diff thirdparty/` is always empty.  The copy uses
`cmake -E copy_directory`, not `file(COPY)`: `file(COPY)` was measured to leave some destination files at
their upstream content when both copies happen in the same configure pass, which silently built the
unpatched header.

`overlay/` is a full-file mirror, not a diff series.  That is deliberate: a small change to
`Optimizer.cc` would otherwise sit in the repository as a patch against a 220 KB file that cannot be
applied with `patch -p1` and cannot be reviewed without a diff anyway.  With the mirror, the reviewable
artefact is `git diff --no-index thirdparty/orbslam3 build/thirdparty/orbslam3`, and every changed file
keeps its GPL notice plus a modification notice at the top.

What the overlay contains, and why:

| Part | Files | Detail |
| --- | --- | --- |
| **ALI** — this project's contribution | `Frame`, `KeyFrame`, `GeometricTools`, `LocalMapping`, `Settings`, `System` | anisotropic landmark initialisation; see `CHANGES.md` §1 |
| **Windows / MSVC port** | `MapPoint.cc`, `Thirdparty/g2o/…`, plus everything in `src/compat/orbslam3/` | no behaviour change; see `CHANGES.md` §2 |
| **Optional switches** inherited from the sibling GIO_GVIL project | `Optimizer`, `MarginalizationInfo`, `MarginalizationPrior`, `GnssAnchor`, `Tracking`, `LoopClosing`, `ORBextractor`, `TwoViewReconstruction` | all default to upstream and are not enabled by the delivered configurations; see `CHANGES.md` §3 |

## Licensing

Every file in `overlay/` is derived from ORB-SLAM3 and is distributed under **GPLv3**, like the rest of
the combined work.  See the repository's `LICENSE` and `THIRD_PARTY_NOTICES.md`.  The upstream files they
replace remain under their original ORB-SLAM3 copyright and license in `thirdparty/orbslam3/`.

When changing an overlay file, keep the existing GPL notice and add a short comment saying that this
project modified it and when.  GPLv3 section 5(a) requires modified files to carry prominent change
notices.
