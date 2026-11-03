# Upstream source

This directory is the source subset of **ORB-SLAM3 v1.0** used by ali-orbslam3:

- Upstream: https://github.com/UZ-SLAMLab/ORB_SLAM3
- Snapshot: `ORB_SLAM3-master`, source files dated 2022-02-10
- Included: `include/`, `src/`, `Thirdparty/`, `LICENSE`, `Changelog.md`, `Dependencies.md`, and the upstream README
- Not included: `Examples/`, `Examples_old/`, `Vocabulary/`, `evaluation/` (large data, not needed to build)

The upstream source files here are kept byte-identical to upstream except for the added
`Thirdparty/DBoW2/LICENSE.txt` (the upstream DBoW2 repository's license text, which ORB-SLAM3's own copy
omits).  `UPSTREAM.md`, `README.upstream.md` and the license files are documentation additions; they do not
change the compiled upstream source.

This project's modifications are not made in this directory.  They live in `patches/orbslam3/overlay/`
and are applied to a build-tree copy by CMake.  See `patches/orbslam3/README.md` and `CHANGES.md`.
