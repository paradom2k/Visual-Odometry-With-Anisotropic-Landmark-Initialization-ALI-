// GIO_GVIL modification notice (2026-10-21).
// Derived from ORB-SLAM3 (GPLv3) and modified by GIO_GVIL; see patches/orbslam3/README.md.
// MarginalizationPrior -- the value type of the sliding-window prior (goal item (F), step 3).
//
// Why this file is separate from MarginalizationInfo.h: G2oTypes.h needs the prior type to declare the dense prior
// edge (`EdgeMarginalizationPrior`), while MarginalizationInfo.h needs the edge type to feed the prior back into
// the marginalization system.  Keeping the value type in its own header that includes neither of them breaks that
// cycle.
//
// What the prior is, exactly:
//   * `info`   -- an information (Hessian) block over `states`, in the *local* parameterisation the vertices use:
//                 pose = (rotation 3, translation 3) with the update  Rwb <- Rwb*Exp(ur), twb += Rwb*ut
//                 (ImuCamPose::Update, src/G2oTypes.cc:192), the other three states are plain 3-vectors;
//   * `residual`-- the first-order term b of the quadratic form 0.5*x'*info*x - residual'*x.  It is what keeps the
//                 prior consistent with the measurements that were eliminated: without it the marginalization
//                 would silently re-linearise the eliminated factors at the new point;
//   * `reference`-- the values of the states at the linearisation point, i.e. where `info`/`residual` were
//                 evaluated.  The residual applied at a later point x is  residual - delta(reference, x).
//
// The support of the prior is *several* keyframes, not one, and that is the reason the plan's original
// "re-apply it through EdgePriorPoseImu" turns out to be impossible: Schur-complementing one keyframe out of a
// window that also contains the random-walk (bias-to-bias) factors leaves the surviving keyframes coupled, so the
// prior is dense over 15*(number of states) dimensions.  `EdgePriorPoseImu` is a unary edge on exactly one
// keyframe (15x15), so it can carry only the block-diagonal part; `EdgeMarginalizationPrior` carries all of it.

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>

namespace ORB_SLAM3 {

/// Which of a keyframe's four states a block belongs to.  The order of the enumerators is the order the four
/// states are addressed throughout (pose, velocity, gyro bias, accel bias).
enum class WindowStateKind { Pose = 0, Velocity = 1, GyroBias = 2, AccBias = 3 };

/// One state of one keyframe, addressed independently of the g2o vertex id.
///
/// This matters: LocalInertialBA derives vertex ids from `maxKFid` (`maxKFid + 3*mnId + {1,2,3}` for
/// velocity/gyro/accel), and `maxKFid` is the *current* keyframe's id, so the vertex id of a given keyframe
/// changes from one BA call to the next.  A prior that outlives one call must therefore be addressed by
/// (keyframe id, state kind) and re-resolved against the new optimisation.
struct PriorStateHeader {
  int kf_id = -1;                                                        ///< KeyFrame::mnId
  WindowStateKind kind = WindowStateKind::Pose;                          ///< which of the four states
  int global_size = 0;                                                   ///< 6 for the pose, 3 otherwise
  int local_size = 0;                                                    ///< == global_size here (no manifold wrapper)
};

/// The frozen prior: information, gradient, minimiser, and linearisation point, over an ordered list of states.
struct MarginalizationPrior {
  Eigen::MatrixXd info;       ///< (D x D), D = sum of the local sizes of `states`
  Eigen::VectorXd residual;   ///< (D) the gradient b of the eliminated window at the linearisation point
  Eigen::VectorXd shift;      ///< (D) +info^+ * residual, the prior edge's centre relative to `reference`
  Eigen::VectorXd reference;  ///< (D) state values at the linearisation point, same layout as `residual`
  std::vector<PriorStateHeader> states;
  double min_eigenvalue = 0.0;  ///< recorded, never assumed positive (see MarginalizationInfo::minEigenvalue)

  /// Number of local coordinates covered by the prior.
  int dimension() const { return static_cast<int>(residual.size()); }

  /// True when the prior is structurally usable: a square information matrix of the right size, a residual,
  /// a shift and a reference of the same size, and at least one state.
  bool usable() const {
    const int d = dimension();
    return d > 0 && info.rows() == d && info.cols() == d && reference.size() == d && shift.size() == d &&
           !states.empty();
  }
};

}  // namespace ORB_SLAM3
