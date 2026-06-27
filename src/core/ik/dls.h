#pragma once

#include "core/ik/ik_solver.h"

namespace kp {

/// Damped Least Squares IK (primary solver, CLAUDE.md §7).
///
///   Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e ,   q ← q + α Δq
///
/// Features:
///   - Adaptive damping: λ² grows as the smallest singular value of J shrinks,
///     so the solver passes cleanly through singularities (Buss 2009).
///   - Joint limits: q is clamped each step; a joint sitting on a limit whose
///     gradient pushes it further out is removed from J for that iteration.
///   - Step clamping: ||Δq|| ≤ step_clamp to avoid overshoot (§10.6).
///   - Line search: if ||e|| rises for 3 consecutive iterations, α is halved.
class DlsSolver : public IkSolver {
 public:
  IkResult solve(const Mechanism& m, int ee_link, const Transform& target,
                 const VecX& q_init, const IkOptions& opts) const override;
};

/// Convenience free function wrapping DlsSolver.
IkResult solve_ik_dls(const Mechanism& m, int ee_link, const Transform& target,
                      const VecX& q_init, const IkOptions& opts = {});

}  // namespace kp
