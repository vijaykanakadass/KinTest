#pragma once

#include <limits>
#include <string>

#include "core/types.h"

namespace kp {

enum class JointType { Fixed, Revolute, Prismatic };

struct JointLimits {
  double lower = -std::numeric_limits<double>::infinity();
  double upper =  std::numeric_limits<double>::infinity();

  bool is_active() const {
    return std::isfinite(lower) || std::isfinite(upper);
  }
};

/// A single joint between two links. Axis is unit and expressed in the parent
/// frame *after* applying `origin` but *before* the joint's variable motion
/// (CLAUDE.md §6).
struct Joint {
  std::string name;
  JointType type = JointType::Fixed;

  /// Unit axis of motion (rotation for Revolute, translation for Prismatic),
  /// expressed in the joint's parent frame after `origin`. Ignored for Fixed.
  Vec3 axis = Vec3::UnitZ();

  /// Rigid transform from the parent link's frame to this joint's origin.
  Transform origin = Transform::Identity();

  JointLimits limits;

  /// Indices into Mechanism::links().
  int parent_link = -1;
  int child_link  = -1;
};

/// Joint transform driven by joint coordinate `q`.
/// Revolute: q is rotation in radians about `axis`.
/// Prismatic: q is translation in meters along `axis`.
/// Fixed: q is ignored; returns identity.
Transform joint_transform(const Joint& j, double q);

}  // namespace kp
