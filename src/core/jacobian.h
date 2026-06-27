#pragma once

#include "core/mechanism.h"
#include "core/types.h"

namespace kp {

/// Geometric Jacobian of an end-effector link, expressed in the WORLD (space)
/// frame. See CLAUDE.md §7 and pitfall §10.5 — error vector and Jacobian must
/// share this frame.
///
/// Returns a 6×dof() matrix. Row blocks follow the twist convention [v; ω]
/// (CLAUDE.md §6): rows 0-2 are linear velocity, rows 3-5 angular velocity.
/// Columns are ordered to match the q-vector (i.e. Mechanism::
/// actuated_joint_indices() order).
///
/// Joint i contributes only if it lies on the kinematic path from the root to
/// `ee_link`; columns of joints not on that path are zero.
///
///   - Revolute joint i: column = [ω_i × (p_ee − p_i); ω_i]
///   - Prismatic joint i: column = [â_i; 0]
///
/// where ω_i / â_i is the joint axis in world coordinates and p_i is the joint
/// origin in world coordinates (both taken in the frame T_world_parent · origin,
/// before the joint's variable motion).
///
/// Throws std::runtime_error if q.size() != dof() or ee_link is out of range.
MatX geometric_jacobian(const Mechanism& m, const VecX& q, int ee_link);

}  // namespace kp
