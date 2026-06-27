// Geometric Jacobian verified against central finite differences
// (CLAUDE.md §7, §9 M2). This is the single most valuable test in the
// project (§11): it catches indexing, sign, frame, and axis errors.
//
// Tolerance 1e-6 — finite differences won't be more accurate than that.

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "core/jacobian.h"
#include "core/mechanism.h"

namespace {

constexpr double kEps = 1e-6;  // finite-difference step
constexpr double kTol = 1e-6;  // comparison tolerance

using kp::MatX;
using kp::Mechanism;
using kp::Transform;
using kp::Vec3;
using kp::VecX;

// Central finite-difference Jacobian in the world frame, [v; ω] convention.
// Angular columns come from the axis-angle of the relative rotation between
// the +ε and −ε poses, matching CLAUDE.md's orientation-error recipe.
MatX numeric_jacobian(const Mechanism& m, const VecX& q, int ee) {
  const int n = m.dof();
  MatX Jn(6, n);
  for (int i = 0; i < n; ++i) {
    VecX qp = q, qm = q;
    qp[i] += kEps;
    qm[i] -= kEps;
    const Transform Tp = m.link_pose(qp, ee);
    const Transform Tm = m.link_pose(qm, ee);

    const Vec3 dp = (Tp.translation() - Tm.translation()) / (2.0 * kEps);
    const Eigen::AngleAxisd aa(Tp.linear() * Tm.linear().transpose());
    const Vec3 dw = aa.axis() * aa.angle() / (2.0 * kEps);

    Jn.block<3, 1>(0, i) = dp;
    Jn.block<3, 1>(3, i) = dw;
  }
  return Jn;
}

void expect_jacobian_matches_fd(const Mechanism& m, const VecX& q) {
  const int ee = static_cast<int>(m.links().size()) - 1;
  const MatX Ja = kp::geometric_jacobian(m, q, ee);
  const MatX Jn = numeric_jacobian(m, q, ee);
  ASSERT_EQ(Ja.rows(), 6);
  ASSERT_EQ(Ja.cols(), m.dof());
  ASSERT_EQ(Jn.rows(), Ja.rows());
  ASSERT_EQ(Jn.cols(), Ja.cols());
  const double err = (Ja - Jn).cwiseAbs().maxCoeff();
  EXPECT_LT(err, kTol) << "analytical:\n" << Ja << "\nnumeric:\n" << Jn;
}

VecX vec(std::initializer_list<double> xs) {
  VecX v(static_cast<int>(xs.size()));
  int i = 0;
  for (double x : xs) v[i++] = x;
  return v;
}

}  // namespace

TEST(Jacobian, Planar2R_MatchesFiniteDifference) {
  auto arm = kp::make_planar_2r(1.0, 1.0);
  for (const auto& q : std::vector<VecX>{
           vec({0.3, 0.7}), vec({-1.2, 0.4}), vec({2.0, -0.9}),
           vec({0.0, 0.0}),       // singular (arm straight) — J still defined
           vec({1.5, 0.0})}) {    // singular (elbow straight)
    SCOPED_TRACE("q = " + std::to_string(q[0]) + ", " + std::to_string(q[1]));
    expect_jacobian_matches_fd(arm, q);
  }
}

TEST(Jacobian, Planar3R_Redundant_MatchesFiniteDifference) {
  auto arm = kp::make_planar_3r(1.0, 1.0, 1.0);
  for (const auto& q : std::vector<VecX>{
           vec({0.2, 0.5, -0.3}), vec({-0.8, 1.1, 0.6}),
           vec({1.4, -1.0, 0.9}), vec({0.0, 0.0, 0.0})}) {
    SCOPED_TRACE("3R config");
    expect_jacobian_matches_fd(arm, q);
  }
}

TEST(Jacobian, Scara_Prismatic_MatchesFiniteDifference) {
  auto arm = kp::make_scara(1.0, 0.8);
  for (const auto& q : std::vector<VecX>{
           vec({0.3, -0.4, 0.25, 0.6}), vec({-1.0, 0.9, 0.10, -0.5}),
           vec({0.7, 0.2, 0.00, 1.2}), vec({0.0, 0.0, 0.5, 0.0})}) {
    SCOPED_TRACE("SCARA config");
    expect_jacobian_matches_fd(arm, q);
  }
}

TEST(Jacobian, Scara_PrismaticColumnIsPureTranslationAlongZ) {
  // The prismatic joint (q3, column 2) must be [0,0,1, 0,0,0] in world:
  // it slides the tool straight down Z and induces no rotation.
  auto arm = kp::make_scara(1.0, 0.8);
  const VecX q = vec({0.4, -0.7, 0.3, 1.1});
  const int ee = static_cast<int>(arm.links().size()) - 1;
  const MatX J = kp::geometric_jacobian(arm, q, ee);
  EXPECT_NEAR(J(0, 2), 0.0, kTol);
  EXPECT_NEAR(J(1, 2), 0.0, kTol);
  EXPECT_NEAR(J(2, 2), 1.0, kTol);
  EXPECT_NEAR(J(3, 2), 0.0, kTol);
  EXPECT_NEAR(J(4, 2), 0.0, kTol);
  EXPECT_NEAR(J(5, 2), 0.0, kTol);
}

TEST(Jacobian, RejectsWrongQSize) {
  auto arm = kp::make_planar_2r(1.0, 1.0);
  EXPECT_THROW(kp::geometric_jacobian(arm, vec({0.0}), 3), std::runtime_error);
}

TEST(Jacobian, RejectsBadEeLink) {
  auto arm = kp::make_planar_2r(1.0, 1.0);
  EXPECT_THROW(kp::geometric_jacobian(arm, vec({0.0, 0.0}), 99),
               std::runtime_error);
}
