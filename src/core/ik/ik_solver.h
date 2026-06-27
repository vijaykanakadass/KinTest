#pragma once

#include "core/mechanism.h"
#include "core/types.h"

namespace kp {

/// Tuning knobs shared by all IK strategies. Defaults follow CLAUDE.md §7.
struct IkOptions {
  double tol        = 1e-5;   ///< convergence threshold on ||e||
  int    max_iters  = 200;    ///< iteration cap
  double damping    = 0.01;   ///< base λ₀ for DLS (ignored by Jacobian-free solvers)
  double step_clamp = 0.1;    ///< max ||Δq|| per iteration (rad/m), CLAUDE.md §10.6
  double alpha      = 1.0;    ///< initial line-search step (α ≤ 1)
  bool   position_only = false;  ///< solve 3D position only, ignore orientation
};

/// Outcome of an IK solve.
struct IkResult {
  VecX   q;                   ///< final joint vector (size = mechanism dof)
  double error_norm = 0.0;    ///< final ||e|| (position-only norm if requested)
  int    iterations = 0;      ///< iterations actually run
  bool   converged  = false;  ///< true iff error_norm < tol
};

/// Strategy interface for inverse-kinematics solvers (CLAUDE.md §4).
/// All solvers work in the WORLD/space frame for both the error and the
/// Jacobian (pitfall §10.5).
class IkSolver {
 public:
  virtual ~IkSolver() = default;

  /// Drive `ee_link` toward `target` (a T_world_ee pose) starting from
  /// `q_init`. Implementations must never return NaN: an unreachable target
  /// yields a finite best-effort pose with converged == false.
  virtual IkResult solve(const Mechanism& m, int ee_link,
                         const Transform& target, const VecX& q_init,
                         const IkOptions& opts) const = 0;
};

/// Pose error twist in the world frame, [v; ω] convention (CLAUDE.md §6).
///   linear  = p_target − p_current
///   angular = axis-angle of (R_target · R_currentᵀ)  (NOT Euler — §10.4)
Vec6 pose_error(const Transform& current, const Transform& target);

}  // namespace kp
