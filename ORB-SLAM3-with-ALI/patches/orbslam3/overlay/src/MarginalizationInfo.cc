// GIO_GVIL modification notice (2026-10-21).
// Derived from ORB-SLAM3 (GPLv3) and modified by GIO_GVIL; see patches/orbslam3/README.md.
// MarginalizationInfo -- implementation (goal item (F), step 3).
//
// The whole file is one pipeline:
//   1. collect the states the window's factors touch, addressing each as (keyframe id, state kind);
//   2. scatter each factor's information block into one dense H, and each factor's gradient into one dense b.
//      b used to be left at zero ("no correction requested"); that is the difference between a prior that keeps
//      the eliminated measurements' first-order term and one that silently re-linearises them, so it is now
//      filled from each factor's own error;
//   3. Schur-complement the states marked for dropping out of (H, b) and freeze the result as the prior.
//
// Verified layout facts about the edges this handles (all in include/G2oTypes.h):
//   EdgeInertial      24x24 = pose(6)+vel(3)+bg(3)+ba(3) of KF i, then pose(6)+vel(3) of KF i+1
//   EdgeGyroRW/AccRW   6x6  = the same bias at KF i and KF i+1 (random walk between them)
//   EdgePriorPoseImu  15x15 = pose+vel+bg+ba of one KF
//   EdgePriorAcc/Gyro   3x3 = one bias
//   EdgeMarginalizationPrior  (15*N)x(15*N), dense over several keyframes (the previous call's prior)

#include "MarginalizationInfo.h"

// The edge types whose ready-made information blocks are scattered in preMarginalize().
#include "G2oTypes.h"

#include <algorithm>
#include <cstdio>
#include <sstream>

namespace ORB_SLAM3 {

namespace {

/// The factor's information block (J'*Omega*J), its Jacobians and its residual, for the edge types this file
/// understands.
///
/// The block is assembled from each edge's own Jacobian() accessors and its own information matrix.  It is
/// deliberately **not** taken from GetHessian(): that helper calls linearizeOplus(), which writes into g2o's
/// `_jacobianOplus*` Maps, and those Maps are only bound inside the optimizer's own buildSystem() loop.  The
/// marginalization reads the factors before that, so going through them dereferences null pointers (measured:
/// an access violation inside EdgeGyroRW::linearizeOplus and inside EdgeInertial::linearizeOplus).
bool EdgeInformation(const g2o::OptimizableGraph::Edge* edge, Eigen::MatrixXd* block,
                     std::vector<Eigen::MatrixXd>* jacobians, Eigen::VectorXd* error) {
  const int dimension = edge->dimension();
  if (dimension <= 0) return false;
  *error = Eigen::VectorXd::Map(edge->errorData(), dimension);
  // g2o's Edge base class exposes the information matrix only as a raw column-major pointer.
  const Eigen::MatrixXd omega = Eigen::MatrixXd::Map(edge->informationData(), dimension, dimension);

  if (const EdgeInertial* inertial = dynamic_cast<const EdgeInertial*>(edge)) {
    const Eigen::Matrix<double, 9, 24> J = inertial->EstimateJacobian();
    jacobians->resize(6);
    for (int v = 0; v < 6; ++v) {
      (*jacobians)[v] = J.block(0, EdgeInertial::kJacobianColumnOffset[v], 9,
                                EdgeInertial::kJacobianBlockWidth[v]);
    }
    *block = J.transpose() * omega * J;
    return true;
  }
  if (const EdgePriorVelocity* priorVelocity = dynamic_cast<const EdgePriorVelocity*>(edge)) {
    jacobians->resize(1);
    (*jacobians)[0] = priorVelocity->Jacobian(0);
  } else if (const EdgePriorPose* priorPose = dynamic_cast<const EdgePriorPose*>(edge)) {
    jacobians->resize(1);
    (*jacobians)[0] = priorPose->computeJacobian();
  } else if (const EdgeGyroRW* gyro = dynamic_cast<const EdgeGyroRW*>(edge)) {
    jacobians->resize(2);
    (*jacobians)[0] = gyro->Jacobian(0);
    (*jacobians)[1] = gyro->Jacobian(1);
  } else if (const EdgeAccRW* acc = dynamic_cast<const EdgeAccRW*>(edge)) {
    jacobians->resize(2);
    (*jacobians)[0] = acc->Jacobian(0);
    (*jacobians)[1] = acc->Jacobian(1);
  } else if (const EdgePriorPoseImu* pose = dynamic_cast<const EdgePriorPoseImu*>(edge)) {
    jacobians->resize(4);
    for (int i = 0; i < 4; ++i) (*jacobians)[i] = pose->Jacobian(i);
  } else if (const EdgePriorAcc* priorAcc = dynamic_cast<const EdgePriorAcc*>(edge)) {
    jacobians->resize(1);
    (*jacobians)[0] = priorAcc->Jacobian(0);
  } else if (const EdgePriorGyro* priorGyro = dynamic_cast<const EdgePriorGyro*>(edge)) {
    jacobians->resize(1);
    (*jacobians)[0] = priorGyro->Jacobian(0);
  } else {
    return false;
  }

  // J is (dimension x sum of the vertex local sizes), the same layout the edges' own GetHessian() uses.
  int width = 0;
  for (std::size_t v = 0; v < jacobians->size(); ++v) {
    if ((*jacobians)[v].rows() != dimension) return false;
    width += static_cast<int>((*jacobians)[v].cols());
  }
  if (width <= 0) return false;

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(dimension, width);
  int offset = 0;
  for (std::size_t v = 0; v < jacobians->size(); ++v) {
    const Eigen::MatrixXd& block_v = (*jacobians)[v];
    J.block(0, offset, dimension, block_v.cols()) = block_v;
    offset += static_cast<int>(block_v.cols());
  }
  *block = J.transpose() * omega * J;
  return true;
}

/// The dense-system offset of the s-th state (states are laid out in first-appearance order).
int DenseOffset(const std::vector<MarginalizationStateAddress>& states, std::size_t index) {
  int offset = 0;
  for (std::size_t i = 0; i < index && i < states.size(); ++i) offset += states[i].global_size;
  return offset;
}

/// delta = local(current) - local(reference) for every state of the input prior, in the prior's layout.
/// The pose uses the same manifold convention as EdgeMarginalizationPrior::computeError.
bool StateDelta(const MarginalizationPrior& prior,
                const std::vector<MarginalizationStateAddress>& states,
                const std::vector<int>& index, Eigen::VectorXd* delta) {
  int offset = 0;
  for (std::size_t a = 0; a < prior.states.size(); ++a) {
    const int size = prior.states[a].local_size;
    const int state_index = (a < index.size()) ? index[a] : -1;
    if (state_index >= 0 && state_index < static_cast<int>(states.size())) {
      const g2o::OptimizableGraph::Vertex* vertex = states[static_cast<std::size_t>(state_index)].vertex;
      if (vertex == nullptr) return false;
      if (size == 6 && vertex->dimension() == 6) {
        const auto* pose = static_cast<const VertexPose*>(vertex);
        const Eigen::Vector3d r_ref = prior.reference.segment<3>(offset);
        const Eigen::Vector3d t_ref = prior.reference.segment<3>(offset + 3);
        delta->segment<3>(offset) = LogSO3(ExpSO3(r_ref).transpose() * pose->estimate().Rwb);
        delta->segment<3>(offset + 3) =
            ExpSO3(r_ref).transpose() * (pose->estimate().twb - t_ref);
      } else if (size == 3) {
        Eigen::Vector3d estimate;
        if (const auto* v = dynamic_cast<const VertexVelocity*>(vertex)) {
          estimate = v->estimate();
        } else if (const auto* g = dynamic_cast<const VertexGyroBias*>(vertex)) {
          estimate = g->estimate();
        } else if (const auto* ac = dynamic_cast<const VertexAccBias*>(vertex)) {
          estimate = ac->estimate();
        } else {
          return false;
        }
        delta->segment<3>(offset) = estimate - prior.reference.segment<3>(offset);
      } else {
        return false;
      }
    }
    offset += size;
  }
  return true;
}

}  // namespace

MarginalizationInfo::~MarginalizationInfo() { clear(); }

void MarginalizationInfo::addEdge(g2o::OptimizableGraph::Edge* edge, const std::vector<int>& drop) {
  if (edge == nullptr) return;
  WindowFactor factor;
  factor.edge = edge;
  factor.drop = drop;
  factors_.push_back(factor);
}

bool MarginalizationInfo::addPriorFactor(const MarginalizationPrior& prior,
                                         const std::vector<MarginalizationState>& present,
                                         bool requireAll) {
  if (!prior.usable()) return false;
  if (prior.dimension() > kMaxPriorDim) {
    std::printf("[F] marg prior rejected: dimension %d exceeds the compiled maximum %d\n", prior.dimension(),
                kMaxPriorDim);
    return false;
  }

  // The blocks are resolved against the declared window later, in preMarginalize(): at this point the window may
  // not be complete yet, and the only question that matters here is whether the prior can be anchored at all.
  int resolved = 0;
  for (std::size_t i = 0; i < prior.states.size(); ++i) {
    for (std::size_t j = 0; j < present.size(); ++j) {
      if (present[j].vertex == nullptr) continue;
      if (present[j].key == WindowState(prior.states[i].kf_id, prior.states[i].kind)) {
        ++resolved;
        break;
      }
    }
  }
  if (resolved == 0) {
    std::printf("[F] marg prior discarded: none of its %d state(s) is in this window\n",
                static_cast<int>(prior.states.size()));
    has_input_prior_ = false;
    return false;
  }
  if (requireAll && resolved != static_cast<int>(prior.states.size())) {
    std::printf("[F] marg prior discarded: %d of %d states missing from this window\n",
                static_cast<int>(prior.states.size()) - resolved, static_cast<int>(prior.states.size()));
    has_input_prior_ = false;
    return false;
  }

  input_prior_ = prior;
  input_prior_index_.assign(prior.states.size(), -1);
  input_prior_present_.assign(prior.states.size(), 0);
  has_input_prior_ = true;
  return true;
}

bool MarginalizationInfo::preMarginalize(const std::vector<MarginalizationState>& window) {
  // Scatter every factor's information block into one dense system over the window's states, and every factor's
  // gradient into b.  A window state is 15 wide (pose 6 + velocity 3 + gyro bias 3 + accel bias 3), which is the
  // layout EdgeMarginalizationPrior and EdgePriorPoseImu both use.
  states_.clear();
  state_index_.clear();
  kept_.clear();
  pre_marginalized_ = false;
  has_prior_ = false;
  vertex_index_.clear();

  // The caller's declaration is the authority on which vertex is which state, and fixes the order in which the
  // states enter the dense system (poses first within each keyframe, as the caller declares them).
  for (std::size_t i = 0; i < window.size(); ++i) {
    g2o::OptimizableGraph::Vertex* vertex = window[i].vertex;
    if (vertex == nullptr) continue;
    if (vertex_index_.count(vertex) != 0) continue;
    const auto existing = state_index_.find(window[i].key);
    if (existing != state_index_.end()) {
      vertex_index_[vertex] = existing->second;
      continue;
    }
    MarginalizationStateAddress address;
    address.key = window[i].key;
    address.vertex = vertex;
    address.local_size = vertex->dimension();
    address.global_size = address.local_size;
    address.fixed = vertex->fixed();
    const int index = static_cast<int>(states_.size());
    state_index_[address.key] = index;
    vertex_index_[vertex] = index;
    states_.push_back(address);
  }
  if (states_.empty()) {
    return false;
  }

  // 1b) the previous prior's blocks, resolved against the declared window by (keyframe id, state kind).  A block
  //     whose keyframe has left the window is reported as missing, which the caller's requireAll decides about.
  if (has_input_prior_) {
    for (std::size_t i = 0; i < input_prior_.states.size(); ++i) {
      const PriorStateHeader& header = input_prior_.states[i];
      const auto found = state_index_.find(WindowState(header.kf_id, header.kind));
      // A fixed state is conditioned on, not re-anchored by the prior; treat that block as absent.
      const bool present = found != state_index_.end() && !states_[found->second].fixed;
      input_prior_present_[i] = present ? 1 : 0;
      input_prior_index_[i] = present ? found->second : -1;
    }
  }

  // Every factor vertex must be a declared state: an undeclared one would silently drop information.
  for (std::size_t f = 0; f < factors_.size(); ++f) {
    const WindowFactor& factor = factors_[f];
    if (factor.edge == nullptr) continue;
    const int vertex_count = static_cast<int>(factor.edge->vertices().size());
    for (int v = 0; v < vertex_count; ++v) {
      g2o::OptimizableGraph::Vertex* vertex =
          dynamic_cast<g2o::OptimizableGraph::Vertex*>(factor.edge->vertex(v));
      if (vertex == nullptr || vertex_index_.count(vertex) == 0) {
        std::printf("[F] marg: a window factor uses a state that was not declared, no prior this call\n");
        return false;
      }
    }
  }
  int total = 0;
  for (std::size_t s = 0; s < states_.size(); ++s) total += states_[s].global_size;
  if (total <= 0) return false;
  H_ = Eigen::MatrixXd::Zero(total, total);
  b_ = Eigen::VectorXd::Zero(total);

  // 2) scatter each factor's Jacobian and gradient.
  //
  // The factor's own J is (error dimension x sum of its vertices' local sizes); every vertex block shares
  // the same rows.  The first version wrote each vertex's block into its own row band and then multiplied
  // by the local Hessian again, which is only correct for a single-vertex factor.  The inertial, random-walk
  // and point-line factors are all multi-vertex, so the assembly must use the true information matrix Omega
  // and the true stacked J.
  for (std::size_t f = 0; f < factors_.size(); ++f) {
    const WindowFactor& factor = factors_[f];
    if (factor.edge == nullptr) continue;
    Eigen::MatrixXd ignored_local_block;
    std::vector<Eigen::MatrixXd> jacobians;
    Eigen::VectorXd error;
    // The marginalisation must see the same error the solver did; the g2o Jacobian workspace is not used.
    factor.edge->computeError();
    bool got = false;
    if (edge_provider_) {
      got = edge_provider_(factor.edge, &ignored_local_block, &jacobians, &error);
    }
    if (!got) got = EdgeInformation(factor.edge, &ignored_local_block, &jacobians, &error);
    if (!got) continue;

    const int dimension = factor.edge->dimension();
    if (dimension <= 0 || error.size() != dimension) continue;
    const int vertex_count = static_cast<int>(factor.edge->vertices().size());
    if (static_cast<int>(jacobians.size()) < vertex_count) continue;

    std::vector<int> offset(vertex_count, -1);
    std::vector<int> width(vertex_count, 0);
    std::vector<char> fixed(vertex_count, 0);
    for (int v = 0; v < vertex_count; ++v) {
      g2o::OptimizableGraph::Vertex* vertex =
          dynamic_cast<g2o::OptimizableGraph::Vertex*>(factor.edge->vertex(v));
      if (vertex == nullptr) return false;
      const auto found = vertex_index_.find(vertex);
      if (found == vertex_index_.end()) return false;
      offset[v] = DenseOffset(states_, found->second);
      // The Jacobian's column count is the state's local size; the vertex is the authority on that.
      width[v] = static_cast<int>(jacobians[v].cols());
      if (width[v] <= 0 || width[v] != vertex->dimension()) return false;
      if (jacobians[v].rows() != dimension) return false;
      fixed[v] = states_[found->second].fixed ? 1 : 0;
    }

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(dimension, total);
    for (int v = 0; v < vertex_count; ++v) {
      // A fixed gauge state is conditioned on: its columns do not enter the Schur system.
      if (fixed[v] == 0) {
        J.block(0, offset[v], dimension, width[v]) = jacobians[v];
      }
    }
    const Eigen::MatrixXd omega =
        Eigen::MatrixXd::Map(factor.edge->informationData(), dimension, dimension);

    // IRLS weight at the current error, the same factor the solver's robust kernel applies.  Applying the raw
    // information here would let the prior disagree with the solve it is supposed to summarise.
    double robust_weight = 1.0;
    if (factor.edge->robustKernel() != nullptr) {
      Eigen::Vector3d rho;
      factor.edge->robustKernel()->robustify(factor.edge->chi2(), rho);
      robust_weight = rho[1];
    }

    H_ += robust_weight * J.transpose() * omega * J;
    // b = sum over factors of J'*Omega*(-e): the negative gradient of the window system at the current
    // estimate, which is what carries the eliminated measurements' first-order term into the prior.
    b_ += -robust_weight * J.transpose() * (omega * error);
  }

  // 2b) the previous prior.  Its cost is 0.5*delta'*P*delta + b'*delta about the prior's reference.
  // The retained states may have moved since that reference, so the gradient at the current estimate is
  // P*delta + b, not b alone.  Without the P*delta term the prior silently re-linearises itself at the
  // wrong point and fights the factors it is meant to summarise.
  if (has_input_prior_) {
    const Eigen::MatrixXd& P = input_prior_.info;
    Eigen::VectorXd delta_prior = Eigen::VectorXd::Zero(input_prior_.dimension());
    if (!StateDelta(input_prior_, states_, input_prior_index_, &delta_prior)) {
      std::printf("[F] marg: the input prior's states could not be evaluated at the current estimate\n");
      return false;
    }
    const Eigen::VectorXd Pdelta = P * delta_prior;

    for (std::size_t a = 0; a < input_prior_.states.size(); ++a) {
      if (input_prior_index_[a] < 0) continue;
      int prior_offset_a = 0;
      for (std::size_t i = 0; i < a; ++i) prior_offset_a += input_prior_.states[i].local_size;
      const int dense_a = DenseOffset(states_, input_prior_index_[a]);
      const int size_a = input_prior_.states[a].local_size;

      // b_ is the NEGATIVE gradient.  The prior's gradient at the current delta is g = -b + P*delta, so its
      // negative gradient is b - P*delta.
      b_.segment(dense_a, size_a) +=
          input_prior_.residual.segment(prior_offset_a, size_a) - Pdelta.segment(prior_offset_a, size_a);

      for (std::size_t c = 0; c < input_prior_.states.size(); ++c) {
        if (input_prior_index_[c] < 0) continue;
        int prior_offset_c = 0;
        for (std::size_t i = 0; i < c; ++i) prior_offset_c += input_prior_.states[i].local_size;
        const int dense_c = DenseOffset(states_, input_prior_index_[c]);
        const int size_c = input_prior_.states[c].local_size;
        H_.block(dense_a, dense_c, size_a, size_c) += P.block(prior_offset_a, prior_offset_c, size_a, size_c);
      }
    }
  }

  pre_marginalized_ = true;
  return true;
}

bool MarginalizationInfo::marginalize(MarginalizationPrior& priorOut) {
  priorOut = MarginalizationPrior();
  has_prior_ = false;
  prior_dimension_ = 0;
  if (!pre_marginalized_ || H_.rows() == 0) return false;

  // States listed in the factors' drop sets leave the window; everything else is kept and carries the prior.
  std::vector<char> dropped(states_.size(), 0);
  for (std::size_t f = 0; f < factors_.size(); ++f) {
    const WindowFactor& factor = factors_[f];
    if (factor.edge == nullptr) continue;
    for (std::size_t d = 0; d < factor.drop.size(); ++d) {
      const int index = factor.drop[d];
      if (index < 0 || index >= static_cast<int>(factor.edge->vertices().size())) continue;
      g2o::OptimizableGraph::Vertex* vertex =
          dynamic_cast<g2o::OptimizableGraph::Vertex*>(factor.edge->vertex(index));
      if (vertex == nullptr) continue;
      const auto found = vertex_index_.find(vertex);
      if (found != vertex_index_.end()) dropped[found->second] = 1;
    }
  }
  // States the caller drops even though no current factor names them: the oldest frame's remaining
  // information lives only in the input prior.
  for (std::size_t d = 0; d < drop_states_.size(); ++d) {
    const auto found = state_index_.find(drop_states_[d]);
    if (found != state_index_.end()) dropped[found->second] = 1;
  }

  // Everything not in a drop set is kept, in the order the factors introduced it: kept first, dropped last.
  kept_.clear();
  std::vector<int> kept_offset(states_.size(), -1);
  std::vector<int> dropped_offset(states_.size(), -1);
  int kept_size = 0;
  int dropped_size = 0;
  for (std::size_t s = 0; s < states_.size(); ++s) {
    if (states_[s].global_size <= 0) continue;
    // The gauge is conditioned on, not eliminated and not carried: its columns are zero in H, and a
    // prior that re-anchored it would fight the next window's fixed pose.
    if (states_[s].fixed) continue;
    if (dropped[s]) {
      dropped_offset[s] = dropped_size;
      dropped_size += states_[s].global_size;
    } else {
      kept_offset[s] = kept_size;
      kept_.push_back(states_[s]);
      kept_size += states_[s].global_size;
    }
  }
  if (kept_.empty()) {
    std::printf("[F] marg: every state of the window is in the drop set, no prior produced\n");
    return false;
  }

  // Marginalise by the full-system pseudo-inverse, not by the four-block Schur formula.
  //
  // The dropped block can be singular in this window (the oldest frame's velocity/bias are observed mainly
  // through the kept frame), and Hkk - Hkr Hrr^+ Hrk then loses the cross-coupling through the null space
  // and comes out non-PSD by a few percent.  For a PSD H the same marginal is obtained from the covariance
  // H^+ : the kept block C_kk is the kept states' marginal covariance, and its inverse is the kept marginal
  // information.  This construction is PSD by definition, and a Jacobi scaling of the full system keeps the
  // 10^12 dynamic range from destroying the pseudo-inverse.
  const int system_size = kept_size + dropped_size;
  std::vector<int> compact(states_.size(), -1);
  for (std::size_t s = 0; s < states_.size(); ++s) {
    if (states_[s].global_size <= 0 || states_[s].fixed) continue;
    compact[s] = (kept_offset[s] >= 0) ? kept_offset[s] : kept_size + dropped_offset[s];
  }

  std::vector<double> sys_scale(static_cast<std::size_t>(system_size), 1.0);
  double max_diag = 0.0;
  for (std::size_t s = 0; s < states_.size(); ++s) {
    if (compact[s] < 0) continue;
    const int dense = DenseOffset(states_, s);
    for (int i = 0; i < states_[s].global_size; ++i) {
      max_diag = std::max(max_diag, H_(dense + i, dense + i));
    }
  }
  const double diag_floor = std::max(max_diag * 1e-12, 1e-12);
  for (std::size_t s = 0; s < states_.size(); ++s) {
    if (compact[s] < 0) continue;
    const int dense = DenseOffset(states_, s);
    for (int i = 0; i < states_[s].global_size; ++i) {
      const double diag = std::max(H_(dense + i, dense + i), diag_floor);
      sys_scale[static_cast<std::size_t>(compact[s] + i)] = 1.0 / std::sqrt(diag);
    }
  }

  Eigen::MatrixXd H_sys = Eigen::MatrixXd::Zero(system_size, system_size);
  Eigen::VectorXd b_sys = Eigen::VectorXd::Zero(system_size);
  for (std::size_t a = 0; a < states_.size(); ++a) {
    if (compact[a] < 0) continue;
    const int size_a = states_[a].global_size;
    const int dense_a = DenseOffset(states_, a);
    const int ca = compact[a];
    for (int i = 0; i < size_a; ++i) {
      b_sys(ca + i) = b_(dense_a + i) * sys_scale[static_cast<std::size_t>(ca + i)];
    }
    for (std::size_t c = 0; c < states_.size(); ++c) {
      if (compact[c] < 0) continue;
      const int size_c = states_[c].global_size;
      const int dense_c = DenseOffset(states_, c);
      const int cc = compact[c];
      for (int i = 0; i < size_a; ++i) {
        for (int j = 0; j < size_c; ++j) {
          H_sys(ca + i, cc + j) = H_(dense_a + i, dense_c + j) *
                                  sys_scale[static_cast<std::size_t>(ca + i)] *
                                  sys_scale[static_cast<std::size_t>(cc + j)];
        }
      }
    }
  }
  H_sys = (H_sys + H_sys.transpose()) * 0.5;

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> sys_solver(H_sys);
  if (sys_solver.info() != Eigen::Success) {
    std::printf("[F] marg: the window Hessian could not be decomposed, no prior produced\n");
    return false;
  }
  const Eigen::VectorXd sys_values = sys_solver.eigenvalues();
  const double sys_largest = sys_values.cwiseAbs().maxCoeff();
  if (!(sys_largest > 0.0)) {
    std::printf("[F] marg: the window Hessian is identically zero, no prior produced\n");
    return false;
  }
  Eigen::VectorXd sys_inverse = Eigen::VectorXd::Zero(sys_values.size());
  const double sys_floor = sys_largest * 1e-14;
  for (int k = 0; k < sys_values.size(); ++k) {
    if (sys_values[k] > sys_floor) sys_inverse[k] = 1.0 / sys_values[k];
  }
  const Eigen::MatrixXd covariance = sys_solver.eigenvectors() * sys_inverse.asDiagonal() *
                                     sys_solver.eigenvectors().transpose();
  const Eigen::MatrixXd kept_covariance = covariance.topLeftCorner(kept_size, kept_size);
  const Eigen::VectorXd minimiser_scaled =
      covariance.topLeftCorner(system_size, kept_size).transpose() * b_sys;
  // minimiser_scaled is the kept part of H^+ b (the kept states come first), in scaled coordinates.

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> cov_solver(
      (kept_covariance + kept_covariance.transpose()) * 0.5);
  if (cov_solver.info() != Eigen::Success) {
    std::printf("[F] marg: the kept covariance block could not be decomposed, no prior produced\n");
    return false;
  }
  const Eigen::VectorXd cov_values = cov_solver.eigenvalues();
  const double cov_largest = cov_values.cwiseAbs().maxCoeff();
  if (!(cov_largest > 0.0)) {
    std::printf("[F] marg: the kept covariance block is identically zero, no prior produced\n");
    return false;
  }
  Eigen::VectorXd info_values = Eigen::VectorXd::Zero(cov_values.size());
  const double cov_floor = cov_largest * 1e-14;
  for (int k = 0; k < cov_values.size(); ++k) {
    if (cov_values[k] > cov_floor) info_values[k] = 1.0 / cov_values[k];
  }
  const Eigen::MatrixXd info_scaled = cov_solver.eigenvectors() * info_values.asDiagonal() *
                                      cov_solver.eigenvectors().transpose();

  Eigen::MatrixXd H_out = Eigen::MatrixXd::Zero(kept_size, kept_size);
  Eigen::VectorXd shift = Eigen::VectorXd::Zero(kept_size);
  for (int a = 0; a < kept_size; ++a) {
    const double s_a = sys_scale[static_cast<std::size_t>(a)];
    shift(a) = s_a * minimiser_scaled(a);
    for (int b = 0; b < kept_size; ++b) {
      const double s_b = sys_scale[static_cast<std::size_t>(b)];
      H_out(a, b) = info_scaled(a, b) / (s_a * s_b);
    }
  }
  H_out = (H_out + H_out.transpose()) * 0.5;

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> out_solver(H_out);
  if (out_solver.info() != Eigen::Success) {
    min_eigenvalue_ = -1.0;
    return false;
  }
  Eigen::VectorXd out_values = out_solver.eigenvalues();
  min_eigenvalue_ = out_values.minCoeff();
  const double out_largest = std::max(1.0, out_values.maxCoeff());
  for (int k = 0; k < out_values.size(); ++k) {
    if (out_values[k] < 0.0) out_values[k] = 0.0;  // clip numerical negatives, keep weak positive information
  }
  const Eigen::MatrixXd projected = out_solver.eigenvectors() * out_values.asDiagonal() *
                                    out_solver.eigenvectors().transpose();

  priorOut.info = (projected + projected.transpose()) * 0.5;
  // The negative gradient of the marginal prior at its reference: H * shift.
  priorOut.residual = priorOut.info * shift;
  priorOut.shift = shift;
  priorOut.min_eigenvalue = min_eigenvalue_;
  prior_dimension_ = kept_size;
  priorOut.reference = Eigen::VectorXd::Zero(kept_size);
  priorOut.states.reserve(kept_.size());
  int reference_offset = 0;
  for (std::size_t i = 0; i < kept_.size(); ++i) {
    PriorStateHeader header;
    header.kf_id = kept_[i].key.kf_id;
    header.kind = kept_[i].key.kind;
    header.global_size = kept_[i].global_size;
    header.local_size = kept_[i].local_size;
    priorOut.states.push_back(header);

    // The linearisation point of the new prior is the current estimate: b_out is the gradient at exactly this
    // point, so the reference is what the residual has to be measured against in the next call.
    g2o::OptimizableGraph::Vertex* vertex = kept_[i].vertex;
    const int take = std::min(3, kept_[i].local_size);
    if (vertex != nullptr && vertex->dimension() == 6) {
      const VertexPose* VP = static_cast<const VertexPose*>(vertex);
      priorOut.reference.segment(reference_offset, 3) = LogSO3(VP->estimate().Rwb);
      priorOut.reference.segment(reference_offset + 3, 3) = VP->estimate().twb;
    } else if (const VertexVelocity* VV = dynamic_cast<const VertexVelocity*>(vertex)) {
      priorOut.reference.segment(reference_offset, take) = VV->estimate().head(take);
    } else if (const VertexGyroBias* VG = dynamic_cast<const VertexGyroBias*>(vertex)) {
      priorOut.reference.segment(reference_offset, take) = VG->estimate().head(take);
    } else if (const VertexAccBias* VA = dynamic_cast<const VertexAccBias*>(vertex)) {
      priorOut.reference.segment(reference_offset, take) = VA->estimate().head(take);
    }
    reference_offset += kept_[i].local_size;
  }

  has_prior_ = true;
  return true;
}

void MarginalizationInfo::clear() {
  factors_.clear();
  states_.clear();
  kept_.clear();
  state_index_.clear();
  vertex_index_.clear();
  has_input_prior_ = false;
  input_prior_ = MarginalizationPrior();
  input_prior_index_.clear();
  input_prior_present_.clear();
  drop_states_.clear();
  H_.resize(0, 0);
  b_.resize(0);
  pre_marginalized_ = false;
  has_prior_ = false;
  min_eigenvalue_ = 0.0;
  prior_dimension_ = 0;
}

std::string MarginalizationInfo::summary() const {
  std::ostringstream text;
  text << "marginalization: " << factors_.size() << " factor(s), " << states_.size() << " state(s), "
       << kept_.size() << " kept";
  if (has_prior_) {
    char buffer[96];
    std::snprintf(buffer, sizeof(buffer), ", prior %dx%d, minEig %.3e", prior_dimension_, prior_dimension_,
                  min_eigenvalue_);
    text << buffer;
  } else {
    text << ", no prior";
  }
  return text.str();
}

}  // namespace ORB_SLAM3
