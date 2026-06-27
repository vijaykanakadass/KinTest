// Mechanism JSON load/save + presets (CLAUDE.md §9 M4).
//   - loading assets/presets/planar_2r.json reproduces the hardcoded factory
//   - round-trip (build -> json -> parse) preserves kinematics
//   - save -> load from disk is faithful
//   - malformed / invalid input is rejected with an error message (no throw)

#include <filesystem>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "core/mechanism.h"
#include "io/mechanism_json.h"
#include "io/presets.h"

#ifndef KINPLAY_ASSETS_DIR
#define KINPLAY_ASSETS_DIR "."
#endif

namespace {

using kp::Mechanism;
using kp::Transform;
using kp::VecX;

VecX vec(std::initializer_list<double> xs) {
  VecX v(static_cast<int>(xs.size()));
  int i = 0;
  for (double x : xs) v[i++] = x;
  return v;
}

// Two mechanisms are kinematically equal if every link pose agrees across a
// battery of joint configurations.
void expect_same_kinematics(const Mechanism& a, const Mechanism& b,
                            const std::vector<VecX>& configs) {
  ASSERT_EQ(a.dof(), b.dof());
  ASSERT_EQ(a.links().size(), b.links().size());
  for (const auto& q : configs) {
    const auto Ta = a.forward_kinematics(q);
    const auto Tb = b.forward_kinematics(q);
    ASSERT_EQ(Ta.size(), Tb.size());
    for (size_t i = 0; i < Ta.size(); ++i) {
      EXPECT_LT((Ta[i].matrix() - Tb[i].matrix()).cwiseAbs().maxCoeff(), 1e-12)
          << "link " << i;
    }
  }
}

std::string asset(const std::string& file) {
  return std::string(KINPLAY_ASSETS_DIR) + "/presets/" + file;
}

}  // namespace

TEST(Io, LoadPlanar2R_ReproducesHardcodedFactory) {
  std::string err;
  auto loaded = kp::load_mechanism_json(asset("planar_2r.json"), &err);
  ASSERT_TRUE(loaded.has_value()) << err;

  auto factory = kp::make_planar_2r(1.0, 1.0);
  expect_same_kinematics(*loaded, factory,
                         {vec({0, 0}), vec({M_PI / 2, 0}), vec({0, M_PI / 2}),
                          vec({M_PI / 4, M_PI / 4}), vec({-1.1, 0.7})});

  // And the canonical Test-1 endpoint (CLAUDE.md §8).
  const int ee = static_cast<int>(loaded->links().size()) - 1;
  const kp::Vec3 p = loaded->link_pose(vec({0, 0}), ee).translation();
  EXPECT_NEAR(p.x(), 2.0, 1e-12);
  EXPECT_NEAR(p.y(), 0.0, 1e-12);
}

TEST(Io, LoadPlanar3RAndScara_MatchFactories) {
  std::string err;
  auto p3r = kp::load_mechanism_json(asset("planar_3r.json"), &err);
  ASSERT_TRUE(p3r.has_value()) << err;
  expect_same_kinematics(*p3r, kp::make_planar_3r(1.0, 1.0, 1.0),
                         {vec({0, 0, 0}), vec({0.3, -0.5, 0.9})});

  auto scara = kp::load_mechanism_json(asset("scara.json"), &err);
  ASSERT_TRUE(scara.has_value()) << err;
  expect_same_kinematics(*scara, kp::make_scara(1.0, 0.8),
                         {vec({0, 0, 0, 0}), vec({0.4, -0.6, 0.35, 0.9})});
}

TEST(Io, RoundTripInMemory) {
  auto orig = kp::make_scara(1.0, 0.8);
  const std::string text = kp::mechanism_to_json(orig, "scara");
  std::string err;
  auto back = kp::parse_mechanism_json(text, &err);
  ASSERT_TRUE(back.has_value()) << err;
  expect_same_kinematics(orig, *back,
                         {vec({0, 0, 0, 0}), vec({0.4, -0.6, 0.35, 0.9}),
                          vec({-1.0, 0.9, 0.1, -0.5})});
}

TEST(Io, SaveThenLoadFromDisk) {
  auto orig = kp::make_planar_3r(1.0, 1.0, 1.0);
  const auto path =
      std::filesystem::temp_directory_path() / "kinplay_test_roundtrip.json";
  std::string err;
  ASSERT_TRUE(kp::save_mechanism_json(orig, path.string(), "p3r", &err)) << err;

  auto back = kp::load_mechanism_json(path.string(), &err);
  ASSERT_TRUE(back.has_value()) << err;
  expect_same_kinematics(orig, *back, {vec({0.2, 0.5, -0.3}), vec({1.4, -1.0, 0.9})});
  std::filesystem::remove(path);
}

TEST(Io, RpyOrientationIsParsed) {
  // A 90° yaw about Z via rpy must rotate the child frame's X axis to +Y.
  const std::string text = R"({
    "name": "yaw_test", "root_link": "base",
    "links": [ {"name": "base"}, {"name": "tip"} ],
    "joints": [ {"name": "j", "type": "fixed", "parent": "base", "child": "tip",
                 "origin": { "xyz": [0,0,0], "rpy": [0, 0, 1.5707963267948966] } } ]
  })";
  std::string err;
  auto m = kp::parse_mechanism_json(text, &err);
  ASSERT_TRUE(m.has_value()) << err;
  const kp::Vec3 x_axis = m->link_pose(vec({}), 1).linear().col(0);
  EXPECT_NEAR(x_axis.x(), 0.0, 1e-12);
  EXPECT_NEAR(x_axis.y(), 1.0, 1e-12);
}

TEST(Io, RejectsMalformedJson) {
  std::string err;
  auto m = kp::parse_mechanism_json("{ not valid json ", &err);
  EXPECT_FALSE(m.has_value());
  EXPECT_FALSE(err.empty());
}

TEST(Io, RejectsUnknownJointType) {
  const std::string text = R"({
    "root_link": "a",
    "links": [ {"name": "a"}, {"name": "b"} ],
    "joints": [ {"name": "j", "type": "screw", "parent": "a", "child": "b"} ]
  })";
  std::string err;
  EXPECT_FALSE(kp::parse_mechanism_json(text, &err).has_value());
  EXPECT_NE(err.find("screw"), std::string::npos);
}

TEST(Io, RejectsDanglingLinkReference) {
  const std::string text = R"({
    "root_link": "a",
    "links": [ {"name": "a"} ],
    "joints": [ {"name": "j", "type": "fixed", "parent": "a", "child": "ghost"} ]
  })";
  std::string err;
  EXPECT_FALSE(kp::parse_mechanism_json(text, &err).has_value());
  EXPECT_NE(err.find("ghost"), std::string::npos);
}

TEST(Io, Presets_NamesAndConstruction) {
  const auto names = kp::preset_names();
  EXPECT_FALSE(names.empty());
  for (const auto& n : names) {
    auto m = kp::make_preset(n);
    EXPECT_TRUE(m.has_value()) << "preset " << n;
  }
  EXPECT_FALSE(kp::make_preset("does_not_exist").has_value());
}
