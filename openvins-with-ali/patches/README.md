# Patch provenance

`ali-source.patch` is the source diff against base commit `b78fa4c`
(OpenVINS integration before the ALI/experiment changes).  It covers
`src/`, `thirdparty/openvins/`, `tools/gauge_free_score.py`, and the modified
4Seasons estimator YAML; it deliberately excludes the prebuilt `bin/`.

The repository tree already contains these changes.  Do not apply the patch a
second time; use it only to review or port the changes to another baseline.
