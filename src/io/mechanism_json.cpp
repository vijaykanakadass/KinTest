#include "io/mechanism_json.h"

#include <fstream>
#include <sstream>
#include <unordered_map>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

namespace kp {

using nlohmann::json;

namespace {

JointType joint_type_from_string(const std::string& s) {
  if (s == "revolute")  return JointType::Revolute;
  if (s == "prismatic") return JointType::Prismatic;
  if (s == "fixed")     return JointType::Fixed;
  throw std::runtime_error("unknown joint type '" + s + "'");
}

const char* joint_type_to_string(JointType t) {
  switch (t) {
    case JointType::Revolute:  return "revolute";
    case JointType::Prismatic: return "prismatic";
    case JointType::Fixed:     return "fixed";
  }
  return "fixed";
}

Vec3 parse_vec3(const json& j) {
  if (!j.is_array() || j.size() != 3)
    throw std::runtime_error("expected a 3-element array");
  return Vec3(j[0].get<double>(), j[1].get<double>(), j[2].get<double>());
}

Transform parse_origin(const json& jo) {
  Transform t = Transform::Identity();
  if (jo.contains("xyz")) t.translation() = parse_vec3(jo["xyz"]);
  if (jo.contains("quat")) {
    const json& q = jo["quat"];
    if (!q.is_array() || q.size() != 4)
      throw std::runtime_error("quat must be [x,y,z,w]");
    Quat quat(q[3].get<double>(), q[0].get<double>(),
              q[1].get<double>(), q[2].get<double>());  // (w,x,y,z) ctor
    quat.normalize();
    t.linear() = quat.toRotationMatrix();
  } else if (jo.contains("rpy")) {
    const Vec3 rpy = parse_vec3(jo["rpy"]);  // [roll, pitch, yaw]
    // URDF fixed-axis convention: R = Rz(yaw)·Ry(pitch)·Rx(roll).
    const Mat3 R = (Eigen::AngleAxisd(rpy.z(), Vec3::UnitZ()) *
                    Eigen::AngleAxisd(rpy.y(), Vec3::UnitY()) *
                    Eigen::AngleAxisd(rpy.x(), Vec3::UnitX()))
                       .toRotationMatrix();
    t.linear() = R;
  }
  return t;
}

}  // namespace

std::optional<Mechanism> parse_mechanism_json(const std::string& text,
                                              std::string* error) {
  auto fail = [&](const std::string& msg) -> std::optional<Mechanism> {
    if (error) *error = msg;
    return std::nullopt;
  };

  json j;
  try {
    j = json::parse(text);
  } catch (const std::exception& e) {
    return fail(std::string("JSON parse error: ") + e.what());
  }

  try {
    if (!j.contains("links") || !j["links"].is_array())
      return fail("missing or non-array 'links'");
    if (!j.contains("joints") || !j["joints"].is_array())
      return fail("missing or non-array 'joints'");

    Mechanism m;
    std::unordered_map<std::string, int> link_index;
    for (const auto& jl : j["links"]) {
      Link l;
      l.name = jl.at("name").get<std::string>();
      l.visual_length = jl.value("visual_length", 0.0);
      if (link_index.count(l.name))
        return fail("duplicate link name '" + l.name + "'");
      const std::string lname = l.name;  // capture before the move
      link_index[lname] = m.add_link(std::move(l));
    }

    auto resolve = [&](const std::string& name) -> int {
      auto it = link_index.find(name);
      return it == link_index.end() ? -1 : it->second;
    };

    for (const auto& jj : j["joints"]) {
      Joint joint;
      joint.name = jj.at("name").get<std::string>();
      joint.type = joint_type_from_string(jj.at("type").get<std::string>());

      const std::string parent = jj.at("parent").get<std::string>();
      const std::string child  = jj.at("child").get<std::string>();
      joint.parent_link = resolve(parent);
      joint.child_link  = resolve(child);
      if (joint.parent_link < 0)
        return fail("joint '" + joint.name + "' references unknown parent '" +
                    parent + "'");
      if (joint.child_link < 0)
        return fail("joint '" + joint.name + "' references unknown child '" +
                    child + "'");

      if (jj.contains("axis")) joint.axis = parse_vec3(jj["axis"]);
      if (jj.contains("origin")) joint.origin = parse_origin(jj["origin"]);
      if (jj.contains("limits")) {
        const json& jl = jj["limits"];
        if (jl.contains("lower")) joint.limits.lower = jl["lower"].get<double>();
        if (jl.contains("upper")) joint.limits.upper = jl["upper"].get<double>();
      }
      m.add_joint(std::move(joint));
    }

    if (!j.contains("root_link"))
      return fail("missing 'root_link'");
    const std::string root = j["root_link"].get<std::string>();
    const int root_idx = resolve(root);
    if (root_idx < 0)
      return fail("root_link '" + root + "' is not a defined link");
    m.set_root_link(root_idx);

    return m;
  } catch (const std::exception& e) {
    return fail(std::string("schema error: ") + e.what());
  }
}

std::optional<Mechanism> load_mechanism_json(const std::string& path,
                                             std::string* error) {
  std::ifstream in(path);
  if (!in) {
    if (error) *error = "cannot open file: " + path;
    return std::nullopt;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  return parse_mechanism_json(ss.str(), error);
}

std::string mechanism_to_json(const Mechanism& m, const std::string& name) {
  json j;
  j["name"] = name;

  const auto& links = m.links();
  j["links"] = json::array();
  for (const auto& l : links) {
    j["links"].push_back({{"name", l.name}, {"visual_length", l.visual_length}});
  }

  if (m.root_link() >= 0 && m.root_link() < static_cast<int>(links.size()))
    j["root_link"] = links[m.root_link()].name;

  j["joints"] = json::array();
  for (const auto& joint : m.joints()) {
    json jj;
    jj["name"]   = joint.name;
    jj["type"]   = joint_type_to_string(joint.type);
    jj["parent"] = links.at(joint.parent_link).name;
    jj["child"]  = links.at(joint.child_link).name;
    jj["axis"]   = {joint.axis.x(), joint.axis.y(), joint.axis.z()};

    const Vec3 p = joint.origin.translation();
    const Quat q(joint.origin.linear());
    jj["origin"] = {{"xyz", {p.x(), p.y(), p.z()}},
                    {"quat", {q.x(), q.y(), q.z(), q.w()}}};

    if (joint.limits.is_active()) {
      json jl = json::object();
      if (std::isfinite(joint.limits.lower)) jl["lower"] = joint.limits.lower;
      if (std::isfinite(joint.limits.upper)) jl["upper"] = joint.limits.upper;
      jj["limits"] = jl;
    }
    j["joints"].push_back(jj);
  }
  return j.dump(2);
}

bool save_mechanism_json(const Mechanism& m, const std::string& path,
                         const std::string& name, std::string* error) {
  std::ofstream out(path);
  if (!out) {
    if (error) *error = "cannot open file for writing: " + path;
    return false;
  }
  out << mechanism_to_json(m, name) << '\n';
  return static_cast<bool>(out);
}

}  // namespace kp
