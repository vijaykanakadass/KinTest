// Standard-DH import helper + UR5 6-DOF case (CLAUDE.md §8, Test 4).
//   - mechanism_from_dh FK matches the independent DH product A_1·…·A_n
//   - UR5 has 6 DOF
//   - geometric Jacobian matches finite differences (6-DOF, with twists)
//   - random q -> FK -> DLS -> FK round-trip converges to the same pose

#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "core/dh.h"
#include "core/ik/dls.h"
#include "core/jacobian.h"
#include "core/mechanism.h"

namespace {

using kp::DhRow;
using kp::Mat3;
using kp::MatX;
using kp::Mechanism;
using kp::Transform;
using kp::Vec3;
using kp::VecX;

VecX vec(std::initializer_list<double> xs) {
  VecX v(static_cast<int>(xs.size()));
  int i = 0;
  for (double x : xs) v[i++] = x;
  return v;
}

// Independent reference: standard-DH single-row transform
//   A = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
Transform dh_A(double a, double alpha, double d, double theta) {
  Transform t = Transform::Identity();
  t = Eigen::AngleAxisd(theta, Vec3::UnitZ()) * Eigen::Translation3d(0, 0, d) *
      Eigen::Translation3d(a, 0, 0) * Eigen::AngleAxisd(alpha, Vec3::UnitX());
  return t;
}

// Reference end-effector pose by directly multiplying per-row A_i(q_i).
Transform dh_fk(const std::vector<DhRow>& rows, const VecX& q) {
  Transform T = Transform::Identity();
  for (size_t i = 0; i < rows.size(); ++i) {
    const DhRow& r = rows[i];
    if (r.type == kp::JointType::Prismatic)
      T = T * dh_A(r.a, r.alpha, r.d + q[static_cast<int>(i)], r.theta);
    else
      T = T * dh_A(r.a, r.alpha, r.d, r.theta + q[static_cast<int>(i)]);
  }
  return T;
}

const std::vector<DhRow>& ur5_rows() {
  static const double h = 1.5707963267948966;
  static const std::vector<DhRow> rows = {
      {0.0, h, 0.089, 0.0, kp::JointType::Revolute},
      {-0.425, 0.0, 0.0, 0.0, kp::JointType::Revolute},
      {-0.392, 0.0, 0.0, 0.0, kp::JointType::Revolute},
      {0.0, h, 0.109, 0.0, kp::JointType::Revolute},
      {0.0, -h, 0.095, 0.0, kp::JointType::Revolute},
      {0.0, 0.0, 0.082, 0.0, kp::JointType::Revolute},
  };
  return rows;
}

int ee_of(const Mechanism& m) {
  return static_cast<int>(m.links().size()) - 1;
}

}  // namespace

TEST(Dh, FoldingMatchesDirectDhProduct) {
  // A chain that exercises nonzero a, alpha, d, AND theta offsets, plus a
  // prismatic row — the general case for the folding logic.
  const std::vector<DhRow> rows = {
      {0.3, 1.5707963267948966, 0.2, 0.4, kp::JointType::Revolute},
      {0.5, -0.6, 0.1, -0.2, kp::JointType::Revolute},
      {0.0, 0.9, 0.15, 0.3, kp::JointType::Prismatic},
      {0.25, 0.0, 0.0, 0.1, kp::JointType::Revolute},
  };
  auto m = kp::mechanism_from_dh(rows);
  ASSERT_EQ(m.dof(), 4);

  for (const auto& q : std::vector<VecX>{vec({0, 0, 0, 0}),
                                         vec({0.5, -0.3, 0.2, 0.7}),
                                         vec({-1.1, 0.8, 0.05, -0.4})}) {
    const Transform got = m.link_pose(q, ee_of(m));
    const Transform want = dh_fk(rows, q);
    EXPECT_LT((got.matrix() - want.matrix()).cwiseAbs().maxCoeff(), 1e-12);
  }
}

TEST(Dh, Ur5HasSixDof) {
  auto ur5 = kp::make_ur5();
  EXPECT_EQ(ur5.dof(), 6);
  // FK matches the direct DH product at a few configurations.
  for (const auto& q : std::vector<VecX>{vec({0, 0, 0, 0, 0, 0}),
                                         vec({0.4, -0.7, 1.1, 0.3, -0.9, 0.6})}) {
    const Transform got = ur5.link_pose(q, ee_of(ur5));
    const Transform want = dh_fk(ur5_rows(), q);
    EXPECT_LT((got.matrix() - want.matrix()).cwiseAbs().maxCoeff(), 1e-12);
  }
}

TEST(Dh, Ur5JacobianMatchesFiniteDifference) {
  auto ur5 = kp::make_ur5();
  const int ee = ee_of(ur5);
  const double eps = 1e-6;
  for (const auto& q : std::vector<VecX>{vec({0.2, -0.5, 0.9, 0.3, -0.4, 0.7}),
                                         vec({-1.0, 0.6, -0.8, 1.2, 0.5, -0.3})}) {
    const MatX J = kp::geometric_jacobian(ur5, q, ee);
    MatX Jn(6, ur5.dof());
    for (int i = 0; i < ur5.dof(); ++i) {
      VecX qp = q, qm = q;
      qp[i] += eps;
      qm[i] -= eps;
      const Transform Tp = ur5.link_pose(qp, ee);
      const Transform Tm = ur5.link_pose(qm, ee);
      Jn.block<3, 1>(0, i) = (Tp.translation() - Tm.translation()) / (2 * eps);
      const Eigen::AngleAxisd aa(Tp.linear() * Tm.linear().transpose());
      Jn.block<3, 1>(3, i) = aa.axis() * aa.angle() / (2 * eps);
    }
    EXPECT_LT((J - Jn).cwiseAbs().maxCoeff(), 1e-6);
  }
}

TEST(Dh, Ur5_FullPoseIkRoundTrip) {
  // CLAUDE.md §8 Test-4 method: pick random q, run FK, feed the pose back into
  // IK from a different start, and check FK of the solution matches.
  auto ur5 = kp::make_ur5();
  const int ee = ee_of(ur5);
  const VecX q_true = vec({0.5, -0.8, 1.0, 0.4, -0.6, 0.9});
  const Transform target = ur5.link_pose(q_true, ee);

  kp::IkOptions opts;
  opts.max_iters = 300;
  const kp::IkResult r =
      kp::solve_ik_dls(ur5, ee, target, vec({0.3, -0.6, 0.8, 0.2, -0.4, 0.7}), opts);

  EXPECT_TRUE(r.converged) << "error_norm=" << r.error_norm;
  const Transform reached = ur5.link_pose(r.q, ee);
  EXPECT_LT((reached.translation() - target.translation()).norm(), 1e-4);
  const double ang =
      Eigen::AngleAxisd(reached.linear() * target.linear().transpose()).angle();
  EXPECT_LT(std::abs(ang), 1e-3);
}
