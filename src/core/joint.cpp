#include "core/joint.h"

namespace kp {

Transform joint_transform(const Joint& j, double q) {
  Transform t = Transform::Identity();
  switch (j.type) {
    case JointType::Fixed:
      return t;
    case JointType::Revolute:
      t.linear() = Eigen::AngleAxisd(q, j.axis.normalized()).toRotationMatrix();
      return t;
    case JointType::Prismatic:
      t.translation() = q * j.axis.normalized();
      return t;
  }
  return t;
}

}  // namespace kp
