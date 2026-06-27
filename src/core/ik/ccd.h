#pragma once

#include "core/ik/ik_solver.h"

namespace kp {

/// Cyclic Coordinate Descent IK (secondary solver, CLAUDE.md §7).
///
/// Position-only and matrix-free. Each sweep walks the actuated joints from the
/// one nearest the end-effector back toward the root; at each joint it makes the
/// single best local move toward the target:
///   - Revolute: rotate about the joint axis to align (p_ee − p_joint) with
///     (p_target − p_joint), via the signed angle between those vectors
///     projected onto the plane normal to the axis.
///   - Prismatic: slide by â·(p_target − p_ee), the distance-minimizing offset
///     along the joint axis.
/// q is clamped to joint limits after every joint update.
///
/// Orientation is ignored (the `target` rotation is unused); `opts.damping` and
/// `opts.step_clamp` are likewise ignored. Convergence and iteration cap follow
/// `opts.tol` / `opts.max_iters`, where one iteration is one full sweep.
class CcdSolver : public IkSolver {
 public:
  IkResult solve(const Mechanism& m, int ee_link, const Transform& target,
                 const VecX& q_init, const IkOptions& opts) const override;
};

/// Convenience free function wrapping CcdSolver.
IkResult solve_ik_ccd(const Mechanism& m, int ee_link, const Transform& target,
                      const VecX& q_init, const IkOptions& opts = {});

}  // namespace kp
