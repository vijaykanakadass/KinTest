#pragma once

#include <vector>

#include "core/mechanism.h"
#include "core/types.h"

namespace kp {

/// One row of a standard (distal) Denavit–Hartenberg table. DH is an *import
/// helper only* — the native representation is URDF-style (CLAUDE.md §6).
///
/// Standard-DH link transform (see docs/math.md):
///   A_i(q) = Rz(θ_i) · Tz(d_i) · Tx(a_i) · Rx(α_i)
/// For a Revolute joint θ_i = theta + q and d_i = d.
/// For a Prismatic joint d_i = d + q and θ_i = theta.
struct DhRow {
  double a = 0.0;      ///< link length: translation along x_i
  double alpha = 0.0;  ///< link twist: rotation about x_i (rad)
  double d = 0.0;      ///< link offset: translation along z_{i-1} (m)
  double theta = 0.0;  ///< joint angle offset about z_{i-1} (rad)
  JointType type = JointType::Revolute;
};

/// Build a serial mechanism whose end-effector FK equals the DH product
/// A_1(q_1)·…·A_n(q_n). The joint variable in DH sits between fixed
/// sub-transforms, so each row's trailing fixed part is folded into the next
/// joint's origin; FK at the end-effector is exact (intermediate link frames
/// are not the DH frames, which is irrelevant to kinematics).
///
/// Links are named base, link1…linkN, ee. The N non-fixed joints are q1…qN.
Mechanism mechanism_from_dh(const std::vector<DhRow>& rows);

/// Approximate UR5 (CLAUDE.md §8, Test 4), built from standard-DH rows.
Mechanism make_ur5();

}  // namespace kp
