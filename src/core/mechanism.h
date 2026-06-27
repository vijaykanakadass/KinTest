#pragma once

#include <vector>

#include "core/joint.h"
#include "core/link.h"
#include "core/types.h"

namespace kp {

/// Open-chain (tree, no loops) kinematic mechanism. v1 scope: see CLAUDE.md §1.
///
/// Conventions:
///   - Transforms are T_world_link: world-frame poses of each link's frame.
///   - The actuated joint vector `q` is sized `dof()` and ordered by
///     `actuated_joint_indices()` (joints appear in insertion order).
class Mechanism {
 public:
  int add_link(Link l);
  int add_joint(Joint j);

  const std::vector<Link>&  links()  const { return links_; }
  const std::vector<Joint>& joints() const { return joints_; }
  std::vector<Joint>&       joints()       { return joints_; }

  int  root_link() const         { return root_link_; }
  void set_root_link(int idx)    { root_link_ = idx; }

  /// Number of actuated (non-Fixed) joints.
  int dof() const;

  /// Indices into joints() of the actuated joints, in q-vector order.
  std::vector<int> actuated_joint_indices() const;

  /// Forward kinematics over the whole tree.
  /// Returns T_world_link for every link, indexed by link index.
  /// `q.size()` must equal `dof()`.
  std::vector<Transform> forward_kinematics(const VecX& q) const;

  /// Convenience: T_world for a single link.
  Transform link_pose(const VecX& q, int link_idx) const;

 private:
  std::vector<Link>  links_;
  std::vector<Joint> joints_;
  int root_link_ = -1;
};

/// Built-in planar 2R arm with link lengths L1, L2 (CLAUDE.md §8, Test 1).
///
/// Topology:
///   links:  [base(0), link1(1), link2(2), ee(3)]
///   joints: q1 (Revolute Z, base->link1, origin = I)
///           q2 (Revolute Z, link1->link2, origin = translate(L1, 0, 0))
///           fixed (link2->ee, origin = translate(L2, 0, 0))
///
/// End-effector position in world frame is `link_pose(q, 3).translation()`:
///   x = L1 cos(q1) + L2 cos(q1+q2)
///   y = L1 sin(q1) + L2 sin(q1+q2)
Mechanism make_planar_2r(double L1, double L2);

}  // namespace kp
