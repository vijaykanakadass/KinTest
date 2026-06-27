#include "core/ik/dls.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include "core/jacobian.h"

namespace kp {

Vec6 pose_error(const Transform& current, const Transform& target) {
  Vec6 e;
  e.head<3>() = target.translation() - current.translation();
  const Mat3 R_err = target.linear() * current.linear().transpose();
  const Eigen::AngleAxisd aa(R_err);
  e.tail<3>() = aa.axis() * aa.angle();  // axis-angle, gimbal-lock free (§10.4)
  return e;
}

namespace {

constexpr double k_sigma_eps   = 1e-2;   // singular value below which we damp up
constexpr double k_ratio_cap   = 1e3;    // cap on the damping multiplier
constexpr double k_limit_band  = 1e-9;   // "at limit" tolerance
constexpr double k_alpha_floor = 1e-3;   // line search won't shrink α past this

// Adaptive λ²: λ₀²·(1 + σ_ratio), where σ_ratio grows as the smallest singular
// value of J shrinks (CLAUDE.md §7). Robust to rank-deficient J (e.g. a planar
// arm chasing a full 6-D pose) where det(J Jᵀ) would be identically zero.
double adaptive_lambda_sq(const MatX& J, double lambda0) {
  const double lam0_sq = lambda0 * lambda0;
  Eigen::JacobiSVD<MatX> svd(J);  // singular values only
  const auto& sv = svd.singularValues();
  if (sv.size() == 0) return lam0_sq;
  const double sigma_min = sv(sv.size() - 1);
  double ratio = k_sigma_eps / std::max(sigma_min, 1e-12) - 1.0;
  ratio = std::clamp(ratio, 0.0, k_ratio_cap);
  return lam0_sq * (1.0 + ratio);
}

// Zero the orientation rows of the error/Jacobian for position-only solves.
void apply_position_only(Vec6& e, MatX& J) {
  e.tail<3>().setZero();
  J.bottomRows<3>().setZero();
}

}  // namespace

IkResult solve_ik_dls(const Mechanism& m, int ee_link, const Transform& target,
                      const VecX& q_init, const IkOptions& opts) {
  const int n = m.dof();
  const std::vector<int> act = m.actuated_joint_indices();

  // Joint limits in q-vector order.
  VecX lo(n), hi(n);
  for (int i = 0; i < n; ++i) {
    const JointLimits& L = m.joints()[act[i]].limits;
    lo[i] = L.lower;
    hi[i] = L.upper;
  }
  auto clamp_to_limits = [&](VecX& q) {
    for (int i = 0; i < n; ++i) {
      if (std::isfinite(lo[i])) q[i] = std::max(q[i], lo[i]);
      if (std::isfinite(hi[i])) q[i] = std::min(q[i], hi[i]);
    }
  };

  VecX q = q_init;
  clamp_to_limits(q);

  auto error_at = [&](const VecX& qq) {
    Vec6 e = pose_error(m.link_pose(qq, ee_link), target);
    if (opts.position_only) e.tail<3>().setZero();
    return e;
  };

  Vec6 e = error_at(q);
  double err = e.norm();
  double alpha = opts.alpha;
  int worse_streak = 0;

  const Eigen::Matrix<double, 6, 6> I6 =
      Eigen::Matrix<double, 6, 6>::Identity();

  int it = 0;
  for (; it < opts.max_iters; ++it) {
    if (err < opts.tol) break;

    MatX J = geometric_jacobian(m, q, ee_link);
    if (opts.position_only) apply_position_only(e, J);

    // Lock joints pressed against a limit whose descent gradient (Jᵀe) would
    // push them further out of range (CLAUDE.md §7).
    const VecX grad = J.transpose() * e;
    for (int i = 0; i < n; ++i) {
      const bool at_hi = std::isfinite(hi[i]) && q[i] >= hi[i] - k_limit_band;
      const bool at_lo = std::isfinite(lo[i]) && q[i] <= lo[i] + k_limit_band;
      if ((at_hi && grad[i] > 0.0) || (at_lo && grad[i] < 0.0)) {
        J.col(i).setZero();
      }
    }

    const double lam_sq = adaptive_lambda_sq(J, opts.damping);
    const Vec6 y = (J * J.transpose() + lam_sq * I6).ldlt().solve(e);
    VecX dq = J.transpose() * y;

    // Step clamp to avoid overshoot/oscillation (§10.6).
    const double dq_norm = dq.norm();
    if (dq_norm > opts.step_clamp) dq *= opts.step_clamp / dq_norm;

    VecX q_new = q + alpha * dq;
    clamp_to_limits(q_new);

    Vec6 e_new = error_at(q_new);
    const double err_new = e_new.norm();
    if (!std::isfinite(err_new)) break;  // never propagate NaN

    if (err_new > err) {
      if (++worse_streak >= 3) {
        alpha = std::max(alpha * 0.5, k_alpha_floor);
        worse_streak = 0;
      }
    } else {
      worse_streak = 0;
    }

    q = std::move(q_new);
    e = e_new;
    err = err_new;
  }

  IkResult res;
  res.q = q;
  res.error_norm = err;
  res.iterations = it;
  res.converged = err < opts.tol;
  return res;
}

IkResult DlsSolver::solve(const Mechanism& m, int ee_link,
                          const Transform& target, const VecX& q_init,
                          const IkOptions& opts) const {
  return solve_ik_dls(m, ee_link, target, q_init, opts);
}

}  // namespace kp
