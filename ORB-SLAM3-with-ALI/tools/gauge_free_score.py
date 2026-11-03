"""Score a route E trajectory against the 4Seasons reference, gauge-free (yaw + translation, no scale).

Same ruler as the earlier work: the estimate is compared after removing the degrees of freedom a monocular
system cannot observe -- a single yaw rotation and a translation -- with the scale left at 1 so that a
scale-drifting run cannot hide behind the alignment.  The EuRoC trajectories ORB-SLAM3 writes carry the
timestamp in **nanoseconds** (a seconds value times 1e9), hence the `--time-scale` default of 1e-9.

    python tools\\gauge_free_score.py --est build\\route_e_runs\\fs_w1_puremono.txt \\
        --ref build\\inputs\\fs_ref_w1.csv
"""
import argparse
import os
import sys

import numpy as np


def load_estimate(path, time_scale):
    stamps, positions = [], []
    with open(path, encoding="utf-8", errors="replace") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            stamps.append(float(parts[0]) * time_scale)
            positions.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.array(stamps), np.array(positions)


def load_reference(path):
    stamps, positions = [], []
    with open(path, encoding="utf-8", errors="replace") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 4:
                continue
            stamps.append(float(parts[0]))
            positions.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.array(stamps), np.array(positions)


def yaw_translation_align(estimate, reference):
    """Rotation about z plus translation, scale fixed at 1 (the gauge a monocular VI system has)."""
    est_xy = estimate[:, :2]
    ref_xy = reference[:, :2]
    est_centred = est_xy - est_xy.mean(axis=0)
    ref_centred = ref_xy - ref_xy.mean(axis=0)
    covariance = est_centred.T @ ref_centred
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
    rotated = np.empty_like(estimate)
    rotated[:, 0] = cos_yaw * estimate[:, 0] - sin_yaw * estimate[:, 1]
    rotated[:, 1] = sin_yaw * estimate[:, 0] + cos_yaw * estimate[:, 1]
    rotated[:, 2] = estimate[:, 2]
    rotated -= rotated.mean(axis=0) - reference.mean(axis=0)
    return rotated, np.degrees(yaw)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--est", required=True)
    parser.add_argument("--ref", required=True)
    parser.add_argument("--time-scale", type=float, default=1e-9)
    parser.add_argument("--max-gap", type=float, default=0.2, help="largest allowed reference time gap (s)")
    args = parser.parse_args(argv)

    # Check the two inputs before reading them.  Without this, a run whose estimator failed (no
    # trajectory was written) reached the scorer as a traceback from load_estimate, which reads like a
    # bug here rather than an estimator that stopped; saying which file is missing is the difference
    # between a minute and a diagnostic session.
    for label, path in (("estimate", args.est), ("reference", args.ref)):
        if not os.path.exists(path):
            print(f"  the {label} file does not exist: {path}")
            return 1

    est_t, est_p = load_estimate(args.est, args.time_scale)
    ref_t, ref_p = load_reference(args.ref)
    if len(est_t) == 0 or len(ref_t) == 0:
        print("  empty estimate or reference")
        return 1

    low = max(est_t[0], ref_t[0])
    high = min(est_t[-1], ref_t[-1])
    interpolated = np.vstack([np.interp(est_t, ref_t, ref_p[:, axis]) for axis in range(3)]).T
    nearest = np.abs(est_t[:, None] - ref_t[None, :]).min(axis=1) if len(ref_t) < 20000 else None
    keep = (est_t >= low) & (est_t <= high)
    if nearest is not None:
        keep &= nearest <= args.max_gap
    if keep.sum() < 10:
        print(f"  only {int(keep.sum())} pose(s) overlap the reference window -- nothing to score")
        return 1

    aligned, yaw = yaw_translation_align(est_p[keep], interpolated[keep])
    error = np.linalg.norm(aligned - interpolated[keep], axis=1)
    error_2d = np.linalg.norm(aligned[:, :2] - interpolated[keep][:, :2], axis=1)
    est_path = np.linalg.norm(np.diff(est_p[keep], axis=0), axis=1).sum()
    ref_path = np.linalg.norm(np.diff(interpolated[keep], axis=0), axis=1).sum()

    print(f"  {args.est}")
    print(f"  matched {int(keep.sum())} pose(s) over {high - low:.1f} s, yaw alignment {yaw:+.2f} deg")
    print(f"  RMS 3D {np.sqrt((error ** 2).mean()):.4f} m   (median {np.median(error):.4f}, "
          f"median {np.median(error):.4f}, max {error.max():.4f})")
    print(f"  RMS 2D {np.sqrt((error_2d ** 2).mean()):.4f} m")
    print(f"  path length estimate {est_path:.1f} m vs reference {ref_path:.1f} m "
          f"(ratio {est_path / max(ref_path, 1e-9):.3f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
