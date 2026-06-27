// Cyclic Coordinate Descent IK (CLAUDE.md §7 secondary solver).
//   - reachable position targets converge (2R, redundant 3R, SCARA prismatic)
//   - CCD and DLS agree on the reached position (A/B cross-check, §7)
//   - unreachable target -> finite best-effort pose, no NaN/divergence
//   - joint limits respected

#include <cmath>

#include <gtest/gtest.h>

#include "core/ik/ccd.h"
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

Transform pos_target(const Vec3& p) {
  Transform t = Transform::Identity();
  t.translation() = p;
  return t;
}

// CCD has a slow asymptotic tail (matrix-free, linear convergence), so it
// wants more sweeps than DLS to reach tight tolerances. Sweeps are cheap.
IkOptions ccd_opts() {
  IkOptions o;
  o.max_iters = 1000;
  return o;
}

Vec3 reached_pos(const Mechanism& m, const VecX& q) {
  return m.link_pose(q, ee_of(m)).translation();
}

bool all_finite(const VecX& q) {
  for (int i = 0; i < q.size(); ++i)
    if (!std::isfinite(q[i])) return false;
  return true;
}

}  // namespace

TEST(IkCcd, Planar2R_ReachablePosition_Converges) {
  auto arm = kp::make_planar_2r(1.0, 1.0);
  const Vec3 target = reached_pos(arm, vec({0.6, 0.8}));
  const IkResult r =
      kp::solve_ik_ccd(arm, ee_of(arm), pos_target(target), vec({0.1, 0.1}), ccd_opts());

  ASSERT_TRUE(all_finite(r.q));
  EXPECT_TRUE(r.converged) << "error_norm=" << r.error_norm;
  EXPECT_LT((reached_pos(arm, r.q) - target).norm(), 1e-4);
}

TEST(IkCcd, Planar3R_Redundant_Converges) {
  auto arm = kp::make_planar_3r(1.0, 1.0, 1.0);
  const Vec3 target = reached_pos(arm, vec({0.3, 0.5, -0.4}));
  const IkResult r = kp::solve_ik_ccd(arm, ee_of(arm), pos_target(target),
                                      vec({0.0, 0.1, 0.2}), ccd_opts());

  ASSERT_TRUE(all_finite(r.q));
  EXPECT_TRUE(r.converged) << "error_norm=" << r.error_norm;
  EXPECT_LT((reached_pos(arm, r.q) - target).norm(), 1e-4);
}

TEST(IkCcd, Scara_Prismatic_ReachesPosition) {
  auto arm = kp::make_scara(1.0, 0.8);
  // A target with nonzero Z forces the prismatic joint to engage.
  const Vec3 target = reached_pos(arm, vec({0.4, -0.6, 0.35, 0.9}));
  ASSERT_GT(std::abs(target.z()), 0.1);
  const IkResult r = kp::solve_ik_ccd(arm, ee_of(arm), pos_target(target),
                                      vec({0.0, 0.0, 0.0, 0.0}), ccd_opts());

  ASSERT_TRUE(all_finite(r.q));
  EXPECT_TRUE(r.converged) << "error_norm=" << r.error_norm;
  EXPECT_LT((reached_pos(arm, r.q) - target).norm(), 1e-4);
}

TEST(IkCcd, AgreesWithDlsOnReachedPosition) {
  auto arm = kp::make_planar_3r(1.0, 1.0, 1.0);
  const Vec3 target = reached_pos(arm, vec({0.7, -0.3, 0.5}));

  IkOptions dls_opts;
  dls_opts.position_only = true;
  const IkResult rd =
      kp::solve_ik_dls(arm, ee_of(arm), pos_target(target), vec({0.1, 0.1, 0.1}), dls_opts);
  const IkResult rc = kp::solve_ik_ccd(arm, ee_of(arm), pos_target(target),
                                       vec({0.1, 0.1, 0.1}), ccd_opts());

  ASSERT_TRUE(rd.converged);
  ASSERT_TRUE(rc.converged);
  // Different joint solutions are fine (redundant arm) — the reached
  // end-effector position must match.
  EXPECT_LT((reached_pos(arm, rd.q) - reached_pos(arm, rc.q)).norm(), 1e-3);
}

TEST(IkCcd, UnreachableTarget_BestEffortNoNaN) {
  auto arm = kp::make_planar_2r(1.0, 1.0);  // max reach 2.0
  const IkResult r =
      kp::solve_ik_ccd(arm, ee_of(arm), pos_target(Vec3(5.0, 0.0, 0.0)),
                       vec({0.2, 0.3}));

  EXPECT_FALSE(r.converged);
  ASSERT_TRUE(all_finite(r.q));
  EXPECT_TRUE(std::isfinite(r.error_norm));
  EXPECT_NEAR(reached_pos(arm, r.q).norm(), 2.0, 1e-2);  // stretched toward it
}

TEST(IkCcd, JointLimitsAreRespected) {
  auto arm = kp::make_planar_2r(1.0, 1.0);
  arm.joints()[0].limits.lower = -0.2;
  arm.joints()[0].limits.upper = 0.2;
  const IkResult r =
      kp::solve_ik_ccd(arm, ee_of(arm), pos_target(Vec3(0.0, 2.0, 0.0)),
                       vec({0.0, 0.0}));

  ASSERT_TRUE(all_finite(r.q));
  EXPECT_GE(r.q[0], -0.2 - 1e-9);
  EXPECT_LE(r.q[0], 0.2 + 1e-9);
}

TEST(IkCcd, StartingAtSolution_ZeroIterations) {
  auto arm = kp::make_planar_2r(1.0, 1.0);
  const Vec3 target = reached_pos(arm, vec({0.5, 0.7}));
  const IkResult r =
      kp::solve_ik_ccd(arm, ee_of(arm), pos_target(target), vec({0.5, 0.7}));
  EXPECT_TRUE(r.converged);
  EXPECT_EQ(r.iterations, 0);
}
