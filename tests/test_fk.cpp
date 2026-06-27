// Forward-kinematics tests for the planar 2R arm (CLAUDE.md §8, Test 1).
// Analytical reference values, tol 1e-10.

#include <cmath>

#include <gtest/gtest.h>

#include "core/mechanism.h"

namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTol = 1e-10;
constexpr int kEeLink = 3;  // see make_planar_2r topology
}  // namespace

using kp::VecX;
using kp::Vec3;
using kp::Transform;
using kp::make_planar_2r;

TEST(Planar2R, ZeroConfig_BothLinksAlongX) {
  auto arm = make_planar_2r(1.0, 1.0);
  VecX q(2); q << 0.0, 0.0;
  const Vec3 p = arm.link_pose(q, kEeLink).translation();
  EXPECT_NEAR(p.x(), 2.0, kTol);
  EXPECT_NEAR(p.y(), 0.0, kTol);
  EXPECT_NEAR(p.z(), 0.0, kTol);
}

TEST(Planar2R, Q1HalfPi_PointsAlongY) {
  auto arm = make_planar_2r(1.0, 1.0);
  VecX q(2); q << kPi / 2.0, 0.0;
  const Vec3 p = arm.link_pose(q, kEeLink).translation();
  EXPECT_NEAR(p.x(), 0.0, kTol);
  EXPECT_NEAR(p.y(), 2.0, kTol);
  EXPECT_NEAR(p.z(), 0.0, kTol);
}

TEST(Planar2R, Q2HalfPi_RightAngleElbow) {
  auto arm = make_planar_2r(1.0, 1.0);
  VecX q(2); q << 0.0, kPi / 2.0;
  const Vec3 p = arm.link_pose(q, kEeLink).translation();
  EXPECT_NEAR(p.x(), 1.0, kTol);
  EXPECT_NEAR(p.y(), 1.0, kTol);
  EXPECT_NEAR(p.z(), 0.0, kTol);
}

TEST(Planar2R, BothQuarterPi) {
  auto arm = make_planar_2r(1.0, 1.0);
  VecX q(2); q << kPi / 4.0, kPi / 4.0;
  const Vec3 p = arm.link_pose(q, kEeLink).translation();
  EXPECT_NEAR(p.x(), std::cos(kPi / 4.0) + std::cos(kPi / 2.0), kTol);
  EXPECT_NEAR(p.y(), std::sin(kPi / 4.0) + std::sin(kPi / 2.0), kTol);
  EXPECT_NEAR(p.z(), 0.0, kTol);
}

TEST(Planar2R, UnequalLinkLengths_GeneralAngle) {
  // Sanity check the closed-form formula with asymmetric link lengths.
  const double L1 = 1.5, L2 = 0.7;
  auto arm = make_planar_2r(L1, L2);
  VecX q(2); q << 0.3, -1.1;
  const Vec3 p = arm.link_pose(q, kEeLink).translation();
  EXPECT_NEAR(p.x(),
              L1 * std::cos(q[0]) + L2 * std::cos(q[0] + q[1]), kTol);
  EXPECT_NEAR(p.y(),
              L1 * std::sin(q[0]) + L2 * std::sin(q[0] + q[1]), kTol);
  EXPECT_NEAR(p.z(), 0.0, kTol);
}

TEST(Planar2R, EeOrientationMatchesSumOfAngles) {
  // The ee frame should be rotated by (q1 + q2) about Z relative to world.
  auto arm = make_planar_2r(1.0, 1.0);
  VecX q(2); q << 0.4, -0.9;
  const Transform T = arm.link_pose(q, kEeLink);
  const double expected = q[0] + q[1];
  // Extract Z rotation: planar, so R = R_z(theta).
  const double theta = std::atan2(T.linear()(1, 0), T.linear()(0, 0));
  EXPECT_NEAR(std::sin(theta - expected), 0.0, kTol);
  EXPECT_NEAR(std::cos(theta - expected), 1.0, kTol);
}

TEST(Planar2R, DofIsTwo) {
  auto arm = make_planar_2r(1.0, 1.0);
  EXPECT_EQ(arm.dof(), 2);
  const auto idx = arm.actuated_joint_indices();
  ASSERT_EQ(idx.size(), 2u);
  EXPECT_EQ(idx[0], 0);
  EXPECT_EQ(idx[1], 1);
}

TEST(Planar2R, RejectsWrongQSize) {
  auto arm = make_planar_2r(1.0, 1.0);
  VecX q(3); q << 0.0, 0.0, 0.0;
  EXPECT_THROW(arm.forward_kinematics(q), std::runtime_error);
}
