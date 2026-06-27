// Damped Least Squares IK (CLAUDE.md §9 M3).
//   - reachable targets converge within tolerance
//   - unreachable targets produce a finite best-effort pose (no NaN/divergence)
//   - joint limits are respected
//   - redundant (3R) and prismatic (SCARA, full pose) chains both solve

#include <cmath>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "core/ik/dls.h"
#include "core/mechanism.h"

namespace {

using kp::IkOptions;
using kp::IkResult;
using kp::Mechanism;
using kp::Transform;
using kp::Vec3;
using kp::VecX;

int ee_of(const Mechanism& m) {
  return static_cast<int>(m.links().size()) - 1;
}

VecX vec(std::initializer_list<double> xs) {
  VecX v(static_cast<int>(xs.size()));
  int i = 0;
  for (double x : xs) v[i++] = x;
  return v;
}

// A reachable target is, by construction, FK of some valid configuration.
Transform target_from_fk(const Mechanism& m, const VecX& q_true) {
  return m.link_pose(q_true, ee_of(m));
}

bool all_finite(const VecX& q) {
  for (int i = 0; i < q.size(); ++i)
    if (!std::isfinite(q[i])) return false;
  return true;
}

}  // namespace

TEST(IkDls, Planar2R_ReachablePosition_Converges) {
  auto arm = kp::make_planar_2r(1.0, 1.0);
  const VecX q_true = vec({0.6, 0.8});
  const Transform target = target_from_fk(arm, q_true);

  IkOptions opts;
  opts.position_only = true;
  const IkResult r = kp::solve_ik_dls(arm, ee_of(arm), target, vec({0.1, 0.1}), opts);

  ASSERT_TRUE(all_finite(r.q));
  EXPECT_TRUE(r.converged) << "error_norm=" << r.error_norm;
  const Vec3 reached = arm.link_pose(r.q, ee_of(arm)).translation();
  EXPECT_LT((reached - target.translation()).norm(), 1e-4);
}

TEST(IkDls, Planar2R_UnreachableTarget_BestEffortNoNaN) {
  auto arm = kp::make_planar_2r(1.0, 1.0);  // max reach 2.0
  Transform target = Transform::Identity();
  target.translation() = Vec3(5.0, 0.0, 0.0);  // far outside the workspace

  IkOptions opts;
  opts.position_only = true;
  const IkResult r = kp::solve_ik_dls(arm, ee_of(arm), target, vec({0.2, 0.3}), opts);

  EXPECT_FALSE(r.converged);
  ASSERT_TRUE(all_finite(r.q));
  EXPECT_TRUE(std::isfinite(r.error_norm));
  // Best effort: the arm should stretch out toward the target, reaching close
  // to its maximum radius (≈2), so the residual is ≈ 3, not blown up.
  const double reach = arm.link_pose(r.q, ee_of(arm)).translation().norm();
  EXPECT_NEAR(reach, 2.0, 1e-2);
}

TEST(IkDls, Planar3R_Redundant_ConvergesToSomeSolution) {
  auto arm = kp::make_planar_3r(1.0, 1.0, 1.0);
  const VecX q_true = vec({0.3, 0.5, -0.4});
  const Transform target = target_from_fk(arm, q_true);

  IkOptions opts;
  opts.position_only = true;
  const IkResult r =
      kp::solve_ik_dls(arm, ee_of(arm), target, vec({0.0, 0.1, 0.2}), opts);

  ASSERT_TRUE(all_finite(r.q));
  EXPECT_TRUE(r.converged) << "error_norm=" << r.error_norm;
  const Vec3 reached = arm.link_pose(r.q, ee_of(arm)).translation();
  EXPECT_LT((reached - target.translation()).norm(), 1e-4);
}

TEST(IkDls, Scara_FullPose_ConvergesWithPrismatic) {
  auto arm = kp::make_scara(1.0, 0.8);
  const VecX q_true = vec({0.4, -0.6, 0.35, 0.9});
  const Transform target = target_from_fk(arm, q_true);  // a reachable 6-D pose

  // Full pose (position + orientation) exercises the prismatic joint and the
  // orientation-error path.
  const IkResult r =
      kp::solve_ik_dls(arm, ee_of(arm), target, vec({0.0, 0.0, 0.0, 0.0}), {});

  ASSERT_TRUE(all_finite(r.q));
  EXPECT_TRUE(r.converged) << "error_norm=" << r.error_norm;
  const Transform reached = arm.link_pose(r.q, ee_of(arm));
  EXPECT_LT((reached.translation() - target.translation()).norm(), 1e-4);
  const double ang =
      Eigen::AngleAxisd(reached.linear() * target.linear().transpose()).angle();
  EXPECT_LT(std::abs(ang), 1e-3);
}

TEST(IkDls, JointLimitsAreRespected) {
  auto arm = kp::make_planar_2r(1.0, 1.0);
  // Clamp the shoulder to a narrow band; a target requiring q1 ≈ π/2 must not
  // push q1 past the limit.
  arm.joints()[0].limits.lower = -0.2;
  arm.joints()[0].limits.upper = 0.2;

  Transform target = Transform::Identity();
  target.translation() = Vec3(0.0, 2.0, 0.0);  // straight up: wants q1 ≈ π/2

  IkOptions opts;
  opts.position_only = true;
  const IkResult r = kp::solve_ik_dls(arm, ee_of(arm), target, vec({0.0, 0.0}), opts);

  ASSERT_TRUE(all_finite(r.q));
  EXPECT_GE(r.q[0], -0.2 - 1e-9);
  EXPECT_LE(r.q[0], 0.2 + 1e-9);
}

TEST(IkDls, StartingAtSolution_ZeroIterations) {
  auto arm = kp::make_planar_2r(1.0, 1.0);
  const VecX q_true = vec({0.5, 0.7});
  const Transform target = target_from_fk(arm, q_true);

  IkOptions opts;
  opts.position_only = true;
  const IkResult r = kp::solve_ik_dls(arm, ee_of(arm), target, q_true, opts);

  EXPECT_TRUE(r.converged);
  EXPECT_EQ(r.iterations, 0);
  EXPECT_LT(r.error_norm, opts.tol);
}
