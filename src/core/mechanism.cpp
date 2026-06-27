#include "core/mechanism.h"

#include <queue>
#include <stdexcept>

namespace kp {

int Mechanism::add_link(Link l) {
  links_.push_back(std::move(l));
  return static_cast<int>(links_.size()) - 1;
}

int Mechanism::add_joint(Joint j) {
  joints_.push_back(std::move(j));
  return static_cast<int>(joints_.size()) - 1;
}

int Mechanism::dof() const {
  int n = 0;
  for (const auto& j : joints_) {
    if (j.type != JointType::Fixed) ++n;
  }
  return n;
}

std::vector<int> Mechanism::actuated_joint_indices() const {
  std::vector<int> out;
  out.reserve(joints_.size());
  for (int i = 0; i < static_cast<int>(joints_.size()); ++i) {
    if (joints_[i].type != JointType::Fixed) out.push_back(i);
  }
  return out;
}

std::vector<Transform> Mechanism::forward_kinematics(const VecX& q) const {
  if (root_link_ < 0 || root_link_ >= static_cast<int>(links_.size())) {
    throw std::runtime_error("Mechanism::forward_kinematics: root link not set");
  }
  const int n = dof();
  if (static_cast<int>(q.size()) != n) {
    throw std::runtime_error("Mechanism::forward_kinematics: q size mismatch");
  }

  // Map joint index -> position in q (or -1 for fixed joints).
  std::vector<int> qmap(joints_.size(), -1);
  {
    int qi = 0;
    for (int i = 0; i < static_cast<int>(joints_.size()); ++i) {
      if (joints_[i].type != JointType::Fixed) qmap[i] = qi++;
    }
  }

  // Adjacency: per parent-link, list of joint indices it drives.
  std::vector<std::vector<int>> children(links_.size());
  for (int i = 0; i < static_cast<int>(joints_.size()); ++i) {
    const auto& j = joints_[i];
    if (j.parent_link >= 0 && j.parent_link < static_cast<int>(links_.size())) {
      children[j.parent_link].push_back(i);
    }
  }

  std::vector<Transform> T(links_.size(), Transform::Identity());
  std::vector<char> visited(links_.size(), 0);

  T[root_link_] = Transform::Identity();
  visited[root_link_] = 1;

  std::queue<int> bfs;
  bfs.push(root_link_);
  while (!bfs.empty()) {
    int parent = bfs.front();
    bfs.pop();
    for (int jidx : children[parent]) {
      const Joint& j = joints_[jidx];
      const int child = j.child_link;
      if (child < 0 || child >= static_cast<int>(links_.size())) continue;
      if (visited[child]) continue;  // tree-only invariant; v1 has no loops
      const double qv = (qmap[jidx] >= 0) ? q[qmap[jidx]] : 0.0;
      T[child] = T[parent] * j.origin * joint_transform(j, qv);
      visited[child] = 1;
      bfs.push(child);
    }
  }
  return T;
}

Transform Mechanism::link_pose(const VecX& q, int link_idx) const {
  return forward_kinematics(q).at(link_idx);
}

Mechanism make_planar_2r(double L1, double L2) {
  Mechanism m;
  const int base  = m.add_link({"base",  0.0});
  const int link1 = m.add_link({"link1", L1});
  const int link2 = m.add_link({"link2", L2});
  const int ee    = m.add_link({"ee",    0.0});
  m.set_root_link(base);

  Joint j1;
  j1.name = "q1";
  j1.type = JointType::Revolute;
  j1.axis = Vec3::UnitZ();
  j1.parent_link = base;
  j1.child_link  = link1;
  j1.origin = Transform::Identity();
  m.add_joint(j1);

  Joint j2;
  j2.name = "q2";
  j2.type = JointType::Revolute;
  j2.axis = Vec3::UnitZ();
  j2.parent_link = link1;
  j2.child_link  = link2;
  j2.origin = Transform::Identity();
  j2.origin.translation() = Vec3(L1, 0.0, 0.0);
  m.add_joint(j2);

  Joint jee;
  jee.name = "ee_fixed";
  jee.type = JointType::Fixed;
  jee.parent_link = link2;
  jee.child_link  = ee;
  jee.origin = Transform::Identity();
  jee.origin.translation() = Vec3(L2, 0.0, 0.0);
  m.add_joint(jee);

  return m;
}

}  // namespace kp
