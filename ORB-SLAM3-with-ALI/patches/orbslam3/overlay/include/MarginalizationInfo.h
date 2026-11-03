// GIO_GVIL modification notice (2026-10-21).
// Derived from ORB-SLAM3 (GPLv3) and modified by GIO_GVIL; see patches/orbslam3/README.md.
// MarginalizationInfo -- goal item (F): a persistent sliding-window prior for ORB-SLAM3's inertial BA.
//
// Why this file exists: ORB-SLAM3 has the two halves of a marginalization prior but never keeps one --
// `Optimizer::Marginalize` (src/Optimizer.cc:2960) is a dense Schur-complement helper, and `EdgePriorPoseImu`
// (include/G2oTypes.h:732) is a 15-dimensional prior on pose+velocity+biases, which is exactly the per-keyframe
// state vector this backend uses.  What was missing is the bookkeeping that VINS-Mono keeps in its
// `MarginalizationInfo` (github_prj/VINS-Mono-master/vins_estimator/src/factor/marginalization_factor.*): 
// evaluate every factor of the window at one linearisation point, assemble the dense system, Schur-complement
// out the states that leave the window, and hand the remaining information back as a prior in the next
// optimisation.
//
// Design decisions, and where they differ from the earlier sketch in docs\MARGINALIZATION_PLAN.md:
//   * g2o instead of Ceres.  g2o edges expose computeError()/linearizeOplus(), so `preMarginalize` is a loop
//     over the window's edges rather than a Ceres Evaluate() with raw jacobian pointers.
//   * the Schur step is written out inline (the four-block formula).  `Optimizer::Marginalize` is the same
//     algebra but returns only the matrix, so it cannot carry the gradient term b; and b is what keeps the
//     prior consistent with the eliminated measurements instead of re-linearising them (see `b_` below).
//   * the prior is expressed as an information matrix plus a frozen linearisation point, and it is *re-applied
//     through the g2o graph* by `EdgeMarginalizationPrior` (include/G2oTypes.h), not by EdgePriorPoseImu: the
//     Schur complement of one keyframe out of a window whose random-walk factors couple consecutive keyframes
//     leaves the surviving keyframes coupled, so the prior is dense over several keyframes.  A unary 15x15 edge
//     could only carry its block-diagonal part.
//   * states are addressed by (keyframe id, state kind), never by g2o vertex id: LocalInertialBA derives vertex
//     ids from the *current* keyframe's id (`maxKFid + 3*mnId + {1,2,3}`), so those ids change from call to
//     call and a prior that outlives one call cannot depend on them.
//   * `clear()` must be called whenever the map is reset, otherwise the prior refers to states that no longer
//     exist (the same class of bug as the two reset fixes in LocalMapping/Tracking).
//
// Two guards are built in, because the classic failure mode of marginalisation is silently locking a stale
// linearisation point into the problem:
//   * `minEigenvalue()` reports the smallest eigenvalue of the prior information matrix (never assumed
//     positive; the prior is projected to PSD and the raw value is recorded for the run log);
//   * `clear()` on every map reset.

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

// g2o's nested types cannot be forward-declared; the full definition is needed for Edge/Vertex anyway.
#include <g2o/core/optimizable_graph.h>

#include "MarginalizationPrior.h"

namespace ORB_SLAM3 {

/// One state of one keyframe that lives in the marginalisation window.
struct WindowState {
  int kf_id = -1;
  WindowStateKind kind = WindowStateKind::Pose;

  WindowState() = default;
  WindowState(int id, WindowStateKind k) : kf_id(id), kind(k) {}

  bool operator==(const WindowState& other) const { return kf_id == other.kf_id && kind == other.kind; }
};

/// One factor of the window, plus which of its states are about to leave it.
struct WindowFactor {
  g2o::OptimizableGraph::Edge* edge = nullptr;  ///< not owned; the edge belongs to the current optimisation
  std::vector<int> drop;                        ///< indices into edge->vertices() that must leave the window
};

/// What the caller knows about one state of one keyframe in the current optimisation.
struct MarginalizationState {
  WindowState key;
  g2o::OptimizableGraph::Vertex* vertex = nullptr;  ///< not owned
};

/// Hash for the (keyframe id, state kind) address, so states can be looked up in constant time.
struct WindowStateHash {
  std::size_t operator()(const WindowState& key) const {
    return static_cast<std::size_t>(key.kf_id) * 4u + static_cast<std::size_t>(key.kind);
  }
};

/// One addressable state, in the order it occupies in the dense system.
struct MarginalizationStateAddress {
  WindowState key;
  g2o::OptimizableGraph::Vertex* vertex = nullptr;  ///< nullptr for a prior block that has no vertex this call
  int global_size = 0;
  int local_size = 0;  ///< 6 for a pose (SE3), 3 for velocity/biases
  /// The vertex is fixed in the optimisation (the window's gauge).  Its columns are removed from H; it is
  /// neither kept nor dropped, so the prior is conditioned on it and never re-anchors it.
  bool fixed = false;
};

class MarginalizationInfo {
 public:
  MarginalizationInfo() = default;
  ~MarginalizationInfo();

  /// Collect a factor of the window.  `drop` lists indices into the edge's own vertex list.
  void addEdge(g2o::OptimizableGraph::Edge* edge, const std::vector<int>& drop);

  /// Mark a state for elimination independently of any factor.  A sliding window drops the oldest frame even
  /// when its only remaining information is the input prior (the frame's own factors left with it); the drop
  /// lists of the current factors cannot express that.
  void addDropState(const WindowState& key) { drop_states_.push_back(key); }

  /// Optional provider for edge types the built-in EdgeInformation() does not know (this project's
  /// EdgePointLine).  It returns true when it handled the edge, fills the factor's J'*Omega*J block, its
  /// per-vertex Jacobians, and its current error.  Returning false falls back to the built-in types.
  using EdgeBlockProvider = std::function<bool(const g2o::OptimizableGraph::Edge*, Eigen::MatrixXd*,
                                               std::vector<Eigen::MatrixXd>*, Eigen::VectorXd*)>;
  void setEdgeBlockProvider(EdgeBlockProvider provider) { edge_provider_ = std::move(provider); }

  /// Feed the prior produced in the previous call back into this window.  `present` maps each previously
  /// marginalised state to its vertex in *this* optimisation; a state whose keyframe has left the map is passed
  /// as a null vertex, and `requireAll` (used by the caller) decides whether a partial prior is acceptable.
  /// Returns false when the prior is unusable or no block of it survives.
  bool addPriorFactor(const MarginalizationPrior& prior, const std::vector<MarginalizationState>& present,
                      bool requireAll);

  /// Evaluate every collected factor once, at the current estimate, and assemble the dense H/b of the window.
  ///
  /// `window` is the caller's declaration of which vertex is which state: every keyframe state the window owns,
  /// with its vertex in this optimisation.  It is passed in rather than derived from the vertices because the
  /// g2o vertex types carry no keyframe back-pointer (VertexPose stores only the estimate), and because the
  /// (keyframe id, state kind) address -- not the g2o vertex id -- is what survives from one call to the next.
  ///
  /// Returns false (and leaves the object empty) if a factor uses a state that is not in `window`, or if a
  /// factor cannot be linearised.
  bool preMarginalize(const std::vector<MarginalizationState>& window);

  /// Schur-complement the states marked for dropping and freeze the resulting prior.  The output is
  /// `priorOut`; it covers the surviving states, in the order of `keptStates()`.  Returns false if the system
  /// is empty or the Schur step fails -- the caller must then carry on with no prior.
  bool marginalize(MarginalizationPrior& priorOut);

  /// True once marginalize() has produced a usable prior.
  bool hasPrior() const { return has_prior_; }

  /// Smallest eigenvalue of the prior information matrix.  Logged, not assumed positive: a strongly negative
  /// value means the prior is unusable and the caller should discard it rather than locking it in.
  double minEigenvalue() const { return min_eigenvalue_; }

  /// States that survive the window, in the order used by the prior produced by marginalize().
  const std::vector<MarginalizationStateAddress>& keptStates() const { return kept_; }

  /// Dimension of the prior produced by the last successful marginalize() (0 when there is none).
  int priorDimension() const { return prior_dimension_; }

  /// Residual term b of the assembled window system (first-order correction carried by the eliminated
  /// measurements).  Exposed for the run log.
  const Eigen::VectorXd& gradient() const { return b_; }

  /// Dense Hessian of the assembled window system.  Exposed for the run log.
  const Eigen::MatrixXd& hessian() const { return H_; }

  /// Drop everything.  MUST be called on every map reset: a prior that outlives its states is the same class of
  /// dangling reference that the two reset fixes addressed.
  void clear();

  /// One-line description for the run log (factor count, kept/dropped sizes, smallest eigenvalue).
  std::string summary() const;

 private:
  std::vector<WindowFactor> factors_;
  std::vector<MarginalizationStateAddress> states_;         ///< all states touched by factors_
  std::vector<MarginalizationStateAddress> kept_;           ///< states that survive the window
  std::unordered_map<WindowState, int, WindowStateHash> state_index_;  ///< address -> index in states_
  std::unordered_map<const g2o::OptimizableGraph::Vertex*, int> vertex_index_;  ///< vertex -> index in states_

  /// The prior handed in by the previous call, as a fourth kind of factor.
  bool has_input_prior_ = false;
  MarginalizationPrior input_prior_;
  /// index into states_ of each prior block, -1 when that block has no vertex in this optimisation
  std::vector<int> input_prior_index_;
  std::vector<char> input_prior_present_;

  Eigen::MatrixXd H_;   ///< dense Hessian of the window at the linearisation point
  Eigen::VectorXd b_;   ///< dense gradient of the window (J'*Omega*(-e) over every factor)

  bool pre_marginalized_ = false;
  bool has_prior_ = false;
  double min_eigenvalue_ = 0.0;
  int prior_dimension_ = 0;

  EdgeBlockProvider edge_provider_;  ///< handles the caller's own edge types (EdgePointLine)
  std::vector<WindowState> drop_states_;  ///< states eliminated even if no current factor names them
};

}  // namespace ORB_SLAM3
