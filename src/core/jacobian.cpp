#include "core/jacobian.h"

#include <stdexcept>
#include <vector>

namespace kp {

MatX geometric_jacobian(const Mechanism& m, const VecX& q, int ee_link) {
  const auto& links  = m.links();
  const auto& joints = m.joints();
  if (ee_link < 0 || ee_link >= static_cast<int>(links.size())) {
    throw std::runtime_error("geometric_jacobian: ee_link out of range");
  }

  // forward_kinematics validates q.size() == dof() and root_link, and gives us
  // T_world_link for every link.
  const std::vector<Transform> T = m.forward_kinematics(q);

  const int n = m.dof();
  MatX J = MatX::Zero(6, n);
  const Vec3 p_ee = T[ee_link].translation();

  // Map each link to the joint that drives it (its incoming joint), and each
  // actuated joint to its column in the q-vector.
  std::vector<int> incoming_joint(links.size(), -1);
  std::vector<int> qcol(joints.size(), -1);
  int qi = 0;
  for (int j = 0; j < static_cast<int>(joints.size()); ++j) {
    const Joint& jt = joints[j];
    if (jt.child_link >= 0 && jt.child_link < static_cast<int>(links.size())) {
      incoming_joint[jt.child_link] = j;
    }
    if (jt.type != JointType::Fixed) qcol[j] = qi++;
  }

  // Walk from the end-effector back to the root, filling one column per
  // actuated joint encountered on the path.
  const int root = m.root_link();
  int link = ee_link;
  while (link >= 0 && link != root) {
    const int jidx = incoming_joint[link];
    if (jidx < 0) break;  // disconnected from root
    const Joint& j = joints[jidx];
    if (j.type != JointType::Fixed) {
      const int col = qcol[jidx];
      // Joint frame in world: T_world_parent · origin (axis lives here,
      // before the variable motion — CLAUDE.md §6).
      const Transform T_world_jf = T[j.parent_link] * j.origin;
      const Vec3 axis_w = T_world_jf.linear() * j.axis.normalized();
      const Vec3 p_i    = T_world_jf.translation();
      if (j.type == JointType::Revolute) {
        J.block<3, 1>(0, col) = axis_w.cross(p_ee - p_i);
        J.block<3, 1>(3, col) = axis_w;
      } else {  // Prismatic
        J.block<3, 1>(0, col) = axis_w;
        // angular block stays zero
      }
    }
    link = j.parent_link;
  }
  return J;
}

}  // namespace kp
