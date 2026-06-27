#include "core/dh.h"

#include <cmath>
#include <string>

#include <Eigen/Geometry>

namespace kp {

namespace {

constexpr double k_half_pi = 1.5707963267948966;

Transform rot_z(double t) {
  Transform x = Transform::Identity();
  x.linear() = Eigen::AngleAxisd(t, Vec3::UnitZ()).toRotationMatrix();
  return x;
}

// Fixed "tail" of a DH row after the joint variable:
//   revolute  -> Tz(d)·Tx(a)·Rx(α)   (d is fixed)
//   prismatic -> Tx(a)·Rx(α)         (d is the variable, absorbed into origin)
Transform dh_tail(const DhRow& r, bool include_dz) {
  Transform t = Transform::Identity();
  if (include_dz) t = t * Eigen::Translation3d(0.0, 0.0, r.d);
  t = t * Eigen::Translation3d(r.a, 0.0, 0.0);
  t = t * Eigen::AngleAxisd(r.alpha, Vec3::UnitX());
  return t;
}

}  // namespace

Mechanism mechanism_from_dh(const std::vector<DhRow>& rows) {
  Mechanism m;
  const int base = m.add_link({"base", 0.0});
  m.set_root_link(base);

  int parent = base;
  Transform b_prev = Transform::Identity();  // fixed tail of the previous row

  for (size_t i = 0; i < rows.size(); ++i) {
    const DhRow& r = rows[i];
    const int child = m.add_link({"link" + std::to_string(i + 1), r.a});

    Joint j;
    j.name = "q" + std::to_string(i + 1);
    j.type = r.type;
    j.axis = Vec3::UnitZ();
    j.parent_link = parent;
    j.child_link = child;

    if (r.type == JointType::Prismatic) {
      // origin = B_prev · Rz(θ) · Tz(d); variable Tz(q) follows; tail = Tx(a)·Rx(α)
      j.origin = b_prev * rot_z(r.theta) * Eigen::Translation3d(0.0, 0.0, r.d);
      m.add_joint(j);
      b_prev = dh_tail(r, /*include_dz=*/false);
    } else {
      // origin = B_prev · Rz(θ); variable Rz(q) follows; tail = Tz(d)·Tx(a)·Rx(α)
      j.origin = b_prev * rot_z(r.theta);
      m.add_joint(j);
      b_prev = dh_tail(r, /*include_dz=*/true);
    }
    parent = child;
  }

  // Final fixed joint carries the last row's tail out to the end-effector frame.
  const int ee = m.add_link({"ee", 0.0});
  Joint jee;
  jee.name = "ee_fixed";
  jee.type = JointType::Fixed;
  jee.parent_link = parent;
  jee.child_link = ee;
  jee.origin = b_prev;
  m.add_joint(jee);

  return m;
}

Mechanism make_ur5() {
  // Approximate UR5 standard-DH parameters (CLAUDE.md §8, Test 4).
  const std::vector<DhRow> rows = {
      {0.0,     k_half_pi,  0.089, 0.0, JointType::Revolute},
      {-0.425,  0.0,        0.0,   0.0, JointType::Revolute},
      {-0.392,  0.0,        0.0,   0.0, JointType::Revolute},
      {0.0,     k_half_pi,  0.109, 0.0, JointType::Revolute},
      {0.0,    -k_half_pi,  0.095, 0.0, JointType::Revolute},
      {0.0,     0.0,        0.082, 0.0, JointType::Revolute},
  };
  return mechanism_from_dh(rows);
}

}  // namespace kp
