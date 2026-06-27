#pragma once

#include <optional>
#include <string>
#include <vector>

#include "core/mechanism.h"

namespace kp {

/// Built-in example mechanisms, constructed in code (no file IO) so the app and
/// tests always have something to load even without the assets/ directory.
/// These mirror the JSON files in assets/presets/.

/// Names of all built-in presets, in display order.
std::vector<std::string> preset_names();

/// Construct a preset by name, or std::nullopt if the name is unknown.
std::optional<Mechanism> make_preset(const std::string& name);

}  // namespace kp
