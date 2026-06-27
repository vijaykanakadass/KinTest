#include "core/ik/ccd.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace kp {

namespace {

constexpr double k_tiny = 1e-12;  // guard for degenerate (zero-length) vectors

// Actuated joints on the path root→ee, returned nearest-ee first (CCD order),
// paired with their column in the q-vector.
struct PathJoint {
  int joint_idx;
  int qcol;
};

std::vector<PathJoint> ccd_path(const Mechanism& m, int ee_link) {
  const auto& joints = m.joints();
  const auto& links  = m.links();

  std::vector<int> incoming(links.size(), -1);
  std::vector<int> qcol(joints.size(), -1);
  int qi = 0;
  for (int j = 0; j < static_cast<int>(joints.size()); ++j) {
    if (joints[j].child_link >= 0 &&
        joints[j].child_link < static_cast<int>(links.size())) {
      incoming[joints[j].child_link] = j;
    }
    if (joints[j].type != JointType::Fixed) qcol[j] = qi++;
  }

  std::vector<PathJoint> path;  // built ee→root, which is the sweep order
  const int root = m.root_link();
  int link = ee_link;
  while (link >= 0 && link != root) {
    const int j = incoming[link];
    if (j < 0) break;
    if (joints[j].type != JointType::Fixed) path.push_back({j, qcol[j]});
    link = joints[j].parent_link;
  }
  return path;
}

}  // namespace

IkResult solve_ik_ccd(const Mechanism& m, int ee_link, const Transform& target,
                      const VecX& q_init, const IkOptions& opts) {
  const int n = m.dof();
  const std::vector<int> act = m.actuated_joint_indices();
  const std::vector<PathJoint> path = ccd_path(m, ee_link);
  const Vec3 p_tgt = target.translation();  // position-only

  VecX lo(n), hi(n);
  for (int i = 0; i < n; ++i) {
    const JointLimits& L = m.joints()[act[i]].limits;
    lo[i] = L.lower;
    hi[i] = L.upper;
  }
  auto clamp1 = [&](int i, double v) {
    if (std::isfinite(lo[i])) v = std::max(v, lo[i]);
    if (std::isfinite(hi[i])) v = std::min(v, hi[i]);
    return v;
  };

  VecX q = q_init;
  for (int i = 0; i < n; ++i) q[i] = clamp1(i, q[i]);

  auto pos_err = [&](const VecX& qq) {
    return (m.link_pose(qq, ee_link).translation() - p_tgt).norm();
  };

  double err = pos_err(q);
  int it = 0;
  for (; it < opts.max_iters; ++it) {
    if (err < opts.tol) break;

    for (const PathJoint& pj : path) {
      const Joint& j = m.joints()[pj.joint_idx];
      const std::vector<Transform> T = m.forward_kinematics(q);
      const Vec3 p_ee = T[ee_link].translation();
      const Transform jf = T[j.parent_link] * j.origin;
      const Vec3 axis = jf.linear() * j.axis.normalized();

      if (j.type == JointType::Revolute) {
        const Vec3 p_j = jf.translation();
        // Components of the ee→ and target→ vectors in the plane normal to axis.
        const Vec3 vc = (p_ee - p_j) - axis * axis.dot(p_ee - p_j);
        const Vec3 vt = (p_tgt - p_j) - axis * axis.dot(p_tgt - p_j);
        if (vc.norm() < k_tiny || vt.norm() < k_tiny) continue;
        const double angle =
            std::atan2(axis.dot(vc.cross(vt)), vc.dot(vt));  // signed about axis
        q[pj.qcol] = clamp1(pj.qcol, q[pj.qcol] + angle);
      } else {  // Prismatic: slide to minimize distance along the axis.
        const double delta = axis.dot(p_tgt - p_ee);
        q[pj.qcol] = clamp1(pj.qcol, q[pj.qcol] + delta);
      }
    }

    const double err_new = pos_err(q);
    if (!std::isfinite(err_new)) break;
    err = err_new;
  }

  IkResult res;
  res.q = q;
  res.error_norm = err;
  res.iterations = it;
  res.converged = err < opts.tol;
  return res;
}

IkResult CcdSolver::solve(const Mechanism& m, int ee_link,
                          const Transform& target, const VecX& q_init,
                          const IkOptions& opts) const {
  return solve_ik_ccd(m, ee_link, target, q_init, opts);
}

}  // namespace kp
