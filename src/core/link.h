#pragma once

#include <string>

namespace kp {

/// A rigid body in the mechanism tree. Math-only metadata for v1; the
/// `visual_length` is purely a hint for the renderer and is not used by FK.
struct Link {
  std::string name;
  double visual_length = 0.0;
};

}  // namespace kp
