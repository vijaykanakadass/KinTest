#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace kp {

// Rigid transform. T_ab means "frame a expressed in frame b": p_b = T_ab * p_a.
// See CLAUDE.md §6.
using Transform = Eigen::Isometry3d;

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;

// Twist convention: [v; w] — linear stacked on top of angular. CLAUDE.md §6.
using Vec6 = Eigen::Matrix<double, 6, 1>;

using MatX = Eigen::MatrixXd;
using VecX = Eigen::VectorXd;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;

}  // namespace kp
