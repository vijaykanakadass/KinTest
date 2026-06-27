#pragma once

#include <optional>
#include <string>

#include "core/mechanism.h"

namespace kp {

/// Load/save mechanisms in the kinplay custom JSON schema (CLAUDE.md §1: a small
/// custom format; URDF import is v2). Schema:
///
/// {
///   "name": "planar_2r",
///   "root_link": "base",
///   "links":  [ {"name": "base", "visual_length": 0.0}, ... ],
///   "joints": [ {
///       "name": "q1",
///       "type": "revolute" | "prismatic" | "fixed",
///       "parent": "base", "child": "link1",
///       "axis": [0, 0, 1],
///       "origin": { "xyz": [x,y,z],
///                   "rpy":  [r,p,y]   // URDF order Rz(y)·Ry(p)·Rx(r); OR
///                   "quat": [x,y,z,w] // takes precedence over rpy if present
///                 },
///       "limits": { "lower": -3.14, "upper": 3.14 }  // omit => unlimited
///   }, ... ]
/// }
///
/// Angles are radians, lengths meters (§6). Orientation is stored as a
/// quaternion on save for exact round-trips, but rpy is accepted on load for
/// human editing.

/// Parse a mechanism from an in-memory JSON string. Returns std::nullopt and,
/// if `error` is non-null, a human-readable message on malformed input.
std::optional<Mechanism> parse_mechanism_json(const std::string& text,
                                              std::string* error = nullptr);

/// Load a mechanism from a JSON file on disk.
std::optional<Mechanism> load_mechanism_json(const std::string& path,
                                             std::string* error = nullptr);

/// Serialize a mechanism to a JSON string (pretty-printed).
std::string mechanism_to_json(const Mechanism& m,
                              const std::string& name = "mechanism");

/// Save a mechanism to a JSON file. Returns false and sets `error` on failure.
bool save_mechanism_json(const Mechanism& m, const std::string& path,
                         const std::string& name = "mechanism",
                         std::string* error = nullptr);

}  // namespace kp
