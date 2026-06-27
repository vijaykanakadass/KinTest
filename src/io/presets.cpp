#include "io/presets.h"

#include "core/dh.h"

namespace kp {

std::vector<std::string> preset_names() {
  return {"planar_2r", "planar_3r", "scara", "ur5"};
}

std::optional<Mechanism> make_preset(const std::string& name) {
  if (name == "planar_2r") return make_planar_2r(1.0, 1.0);
  if (name == "planar_3r") return make_planar_3r(1.0, 1.0, 1.0);
  if (name == "scara")     return make_scara(1.0, 0.8);
  if (name == "ur5")       return make_ur5();
  return std::nullopt;
}

}  // namespace kp
