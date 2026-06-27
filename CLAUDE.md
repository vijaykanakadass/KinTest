# Kinematics Playground — Project Context

> This file is read by Claude Code on startup. It defines what we're building, how to build it, what conventions to follow, and what reference material to trust. Update it as decisions evolve.

---

## 1. What we're building

An interactive C++ application that lets a user:

1. **Define a kinematic mechanism** — a tree of links connected by joints (revolute, prismatic, fixed). Parameters exposed to the user include link lengths, joint axes, joint limits, and parent/child relationships.
2. **Play with it** — drive joints directly (forward kinematics slider per joint), or grab the end-effector with a 3D gizmo and have the system solve inverse kinematics in real time.
3. **Visualize** — 3D viewport with the mechanism rendered as links + joint frames + a draggable target for IK.

### Explicit scope decisions (locked in, do not relitigate)

- **Serial (open-chain) mechanisms only for v1.** No closed loops (no four-bar, Delta, Stewart). We may add them behind a feature flag later.
- **URDF import is v2.** For v1 we use a small custom JSON schema (defined below). URDF is a huge format with meshes, inertias, collisions — we don't need any of that yet.
- **Kinematics only.** No dynamics, no gravity, no mass/inertia, no collision detection. Pure geometry.
- **Single-platform in v1:** Linux first. CMake is written portably but only Linux CI is required to pass.

### Non-goals

- Real-time trajectory planning, RRT, motion planning
- Physics simulation
- Mesh import (we render links as capsules/cylinders between joint origins)
- Networking, persistence to cloud, multi-user

---

## 2. Build & run

```bash
# First-time setup
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# Iterative build (fast)
cmake --build build -j
# Run the app
./build/kinplay
# Run the headless tests (THIS is what you verify against in the agent loop)
ctest --test-dir build --output-on-failure
# Single test, verbose
./build/tests/test_fk --gtest_filter=PlanarArm.ThreeR
```

**Always use Ninja, not Make** — incremental builds are much faster and the agent loop is compile-bound.

**Always build with `ccache`** if available:
```bash
export CMAKE_C_COMPILER_LAUNCHER=ccache
export CMAKE_CXX_COMPILER_LAUNCHER=ccache
```

**Sanitizer build** (use this by default during development):
```bash
cmake -S . -B build-asan -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer"
cmake --build build-asan -j
ctest --test-dir build-asan --output-on-failure
```

If AddressSanitizer or UBSan report anything, **fix it before proceeding to the next task**. Do not suppress warnings.

### Build environment notes (M1)

- This environment's git proxy blocks `git clone` of GitHub/GitLab repos but allows HTTPS tarball downloads via `codeload.github.com`. Top-level `CMakeLists.txt` and `tests/CMakeLists.txt` therefore use `FetchContent_Declare(... URL ... URL_HASH SHA256=...)` instead of `GIT_REPOSITORY`. Hashes are pinned for reproducibility — update both URL and hash together if a dep is bumped.
- Eigen 3.4.0 is fetched from the `eigen-mirror/eigen` GitHub mirror because gitlab.com (Eigen's canonical home) returns 503 from this environment.

---

## 3. Tech stack (locked)

| Concern | Library | Reason |
|--|--|--|
| Linear algebra | **Eigen 3.4+** | Standard for robotics C++, header-only, excellent SVD & decompositions |
| Windowing + input | **GLFW 3.4** | Minimal, well-documented, plays nicely with Dear ImGui |
| Immediate-mode UI | **Dear ImGui (docking branch)** | Panels, sliders, debug inspectors |
| 3D gizmo | **ImGuizmo** (CedricGuillemet) | The translate/rotate handles for the IK target |
| Graphics | **OpenGL 3.3 core** + **glad** | Widest support, enough for lines/capsules |
| Testing | **GoogleTest** | FK/Jacobian/IK have clean numerical test cases |
| JSON | **nlohmann/json** | Mechanism definition files |
| Logging | **spdlog** | Fast, structured, nice to tail from the agent's perspective |

All pulled via CMake `FetchContent` so the project is one-command buildable.

**Do NOT pull in:** ROS, MoveIt, KDL, Pinocchio, RBDL. These are excellent libraries but the whole point of the project is to *demonstrate* building the kinematics ourselves. If you find yourself wanting one of them, stop and ask.

---

## 4. Project layout

```
kinplay/
├── CMakeLists.txt
├── CLAUDE.md                    ← this file
├── README.md
├── cmake/                       # helper modules (FetchContent shims, sanitizers)
├── src/
│   ├── core/                    # math + kinematics (NO graphics deps here)
│   │   ├── types.h              # Transform, Twist, Vec3, Quat aliases over Eigen
│   │   ├── link.h/.cpp
│   │   ├── joint.h/.cpp         # Joint types + joint->Transform functions
│   │   ├── mechanism.h/.cpp     # Tree container, FK traversal
│   │   ├── dh.h/.cpp            # Standard-DH import helper (builds UR5 preset)
│   │   ├── jacobian.h/.cpp      # Geometric Jacobian computation
│   │   └── ik/
│   │       ├── ik_solver.h      # Strategy interface
│   │       ├── dls.h/.cpp       # Damped Least Squares
│   │       ├── ccd.h/.cpp       # Cyclic Coordinate Descent
│   │       └── fabrik.h/.cpp    # Forward-And-Backward Reaching IK (position only)
│   ├── io/
│   │   ├── mechanism_json.h/.cpp  # Load/save mechanism definition
│   │   └── presets.h/.cpp         # Built-in example mechanisms
│   ├── gfx/                     # OpenGL rendering
│   │   ├── renderer.h/.cpp
│   │   ├── camera.h/.cpp        # Orbit camera
│   │   ├── primitives.h/.cpp    # Lines, grid, capsules, axes
│   │   └── shader.h/.cpp
│   ├── ui/                      # ImGui panels
│   │   ├── main_window.h/.cpp
│   │   ├── mechanism_panel.h/.cpp   # Tree editor
│   │   ├── joint_panel.h/.cpp       # Per-joint sliders (FK)
│   │   ├── ik_panel.h/.cpp          # Gizmo + solver controls
│   │   └── gizmo_wrapper.h/.cpp
│   └── main.cpp
├── tests/
│   ├── test_fk.cpp              # Forward kinematics against analytical cases
│   ├── test_jacobian.cpp        # Jacobian against finite differences
│   ├── test_dh.cpp             # DH import helper + UR5 6-DOF case
│   ├── test_ik_dls.cpp
│   ├── test_ik_ccd.cpp
│   └── test_io.cpp
├── assets/
│   ├── presets/
│   │   ├── planar_2r.json
│   │   ├── planar_3r.json
│   │   ├── scara.json
│   │   └── ur5_like.json
│   └── shaders/
└── docs/
    ├── math.md                  # DH conventions, twist/screw basics
    └── ik.md                    # solver design notes
```

**Core is deliberately graphics-free.** `src/core/` must not include any GLFW / OpenGL / ImGui headers. This is how we keep the math unit-testable and the agent loop fast.

---

## 5. Coding conventions

- **C++20.** Use `std::span`, designated initializers, concepts where they help.
- **Eigen aliases** in `core/types.h`:
  ```cpp
  using Transform = Eigen::Isometry3d;   // rigid transform
  using Vec3 = Eigen::Vector3d;
  using Vec6 = Eigen::Matrix<double, 6, 1>;  // twist: [v; ω]
  using MatX = Eigen::MatrixXd;
  using VecX = Eigen::VectorXd;
  ```
- **Double precision everywhere** in the math core. Single precision only at the graphics boundary.
- **Pass Eigen objects by `const Ref<const T>&`** for function params when interop matters; otherwise by const-ref.
- `#pragma once` for include guards.
- `snake_case` for functions and variables, `PascalCase` for types, `k_` prefix for compile-time constants.
- No raw `new`/`delete` outside library boundaries — use smart pointers or value types.
- Error handling: prefer `std::expected<T, Error>` or `std::optional<T>` over exceptions for recoverable problems. Use `assert`/`throw std::runtime_error` for programmer errors.
- Every public function in `core/` gets a Doxygen-style comment with the math convention it uses (frame, direction, etc.). Conventions are where bugs hide.

---

## 6. Conventions for the math (READ THIS BEFORE WRITING FK CODE)

Pick conventions once, stick to them, document them.

- **Transforms are 4×4 rigid transforms** stored as `Eigen::Isometry3d`. `T_ab` means "frame a expressed in frame b" — i.e. `p_b = T_ab * p_a`. The subscript reads right-to-left. Write the subscript in variable names: `T_world_ee`, `T_base_link3`.
- **Twists / Jacobian columns are 6-vectors** `[v; ω]` — linear velocity stacked on top of angular velocity. This matches Eigen/Modern Robotics convention. Document it anywhere a 6-vector appears.
- **Joint axes are expressed in the joint's parent frame** (i.e., the frame after applying parent→joint_origin transform but before the joint's variable rotation/translation).
- **Euler order:** Do **not** use Euler angles internally. Convert at UI boundary only. Internal representation of orientation error: axis-angle vector `ω = axis * angle` (length 3), derived from `Eigen::AngleAxisd` on the rotation error matrix.
- **Units:** meters, radians, seconds. Never mix.

### Joint types (v1)

```cpp
enum class JointType { Fixed, Revolute, Prismatic };
// Each non-fixed joint has exactly one DOF and an axis (Vec3, unit).
```

DH parameters are **not** the primary representation — we use URDF-style (parent-frame origin + axis) because it generalizes more naturally. DH is provided as an *import helper* only.

---

## 7. Algorithms — reference implementations

### Forward kinematics

Standard tree traversal. For each joint, compose:
```
T_parent_child = T_parent_jointOrigin * JointTransform(jointType, axis, q)
```
where `JointTransform` for revolute is `AngleAxisd(q, axis)` as a rotation, and for prismatic is a translation `q * axis`.

### Geometric Jacobian

For a serial chain with end-effector frame `ee` and `n` joints, the 6×n geometric Jacobian in the world frame:

- For joint `i` (revolute): column i = `[ω_i × (p_ee - p_i); ω_i]` — where `ω_i` is the joint axis in world frame, `p_i` is the joint origin in world frame, `p_ee` is end-effector position in world.
- For joint `i` (prismatic): column i = `[axis_i; 0]`.

Verify against **finite differences** as a test:
```
J_i ≈ (FK(q + ε e_i) - FK(q)) / ε  (for position)
```
Orientation uses axis-angle of `R(q + ε e_i) * R(q).inverse()`.

### Inverse kinematics — Damped Least Squares (primary solver)

Given target pose `T*`, current pose `T(q)`, error twist `e` (6-vector: position error + axis-angle orientation error):

```
Δq = J^T (J J^T + λ² I)^{-1} e
q  ← q + α Δq     (α ≤ 1, adaptive line search)
```

- **λ (damping):** 0.01 typical. Auto-scale near singularities: `λ² = λ₀² · (1 + σ_ratio)` where `σ_ratio` grows as smallest singular value shrinks. A simpler adaptive rule: `λ² = max(λ_min², κ / manipulability)` where `manipulability = sqrt(det(J J^T))`.
- **Convergence:** iterate until `||e|| < tol` (e.g. 1e-5) or max iterations (e.g. 50). If `||e||` increases for 3 consecutive iterations, halve α.
- **Joint limits:** clamp q after each step. If clamped joints are on their limits and would like to move the wrong way, they should be removed from the Jacobian (set their column to zero) for that iteration.

This is the method from Buss (2009) and Wampler (1986). Our test cases should match Chiaverini's 1994 paper's behavior on shoulder/wrist singularities.

### IK — CCD (secondary solver)

Iterates joints from end-effector back to root; at each, rotates to minimize end-effector distance to target. Position-only, but converges fast and has no matrix ops. Good baseline to A/B test DLS against.

### IK — FABRIK (optional)

Forward And Backward Reaching Inverse Kinematics. Position-only, geometric, extremely fast. Use if we add a chain-of-points visualization. Skip for v1 if time-constrained.

---

## 8. Canonical test cases (build these FIRST)

These have analytical solutions. If FK/IK disagree with them, the implementation is wrong.

### Test 1: Planar 2R arm

Two revolute joints around Z, link lengths `L1, L2`.

- FK: `x = L1 cos(q1) + L2 cos(q1+q2)`, `y = L1 sin(q1) + L2 sin(q1+q2)`.
- IK (closed form, two solutions):
  ```
  cos(q2) = (x² + y² - L1² - L2²) / (2 L1 L2)
  q2 = ±acos(cos(q2))               # elbow up / elbow down
  q1 = atan2(y, x) - atan2(L2 sin(q2), L1 + L2 cos(q2))
  ```
- Workspace: annulus of inner radius `|L1-L2|` and outer radius `L1+L2`.

Test cases with `L1 = L2 = 1.0`:
| q1 | q2 | x expected | y expected |
|--|--|--|--|
| 0 | 0 | 2.0 | 0.0 |
| π/2 | 0 | 0.0 | 2.0 |
| 0 | π/2 | 1.0 | 1.0 |
| π/4 | π/4 | cos(π/4)+cos(π/2) ≈ 0.7071 | sin(π/4)+sin(π/2) ≈ 1.7071 |

Tolerance: 1e-10.

### Test 2: Planar 3R arm

Three revolute Z joints, `L1=L2=L3=1`. Redundant for position-only tasks (3 DOF, 2 task-dims) — good for testing DLS on redundant systems. Infinite IK solutions; verify DLS converges to *some* valid one.

### Test 3: SCARA-like (4 DOF)

Revolute-Z, revolute-Z, prismatic-Z, revolute-Z. Classic pick-and-place topology. Has analytical IK. Good for testing prismatic joint handling.

### Test 4: 6-DOF anthropomorphic arm (UR5-style DH)

DH params (approximate UR5):
| i | a (m) | α (rad) | d (m) | θ (var) |
|--|--|--|--|--|
| 1 | 0 | π/2 | 0.089 | q1 |
| 2 | -0.425 | 0 | 0 | q2 |
| 3 | -0.392 | 0 | 0 | q3 |
| 4 | 0 | π/2 | 0.109 | q4 |
| 5 | 0 | -π/2 | 0.095 | q5 |
| 6 | 0 | 0 | 0.082 | q6 |

Use standard DH (Craig convention — **document which**). Test cases can be generated by: pick random q, run FK, feed FK result back into IK, check IK converges to *some* q' with FK(q') matching.

---

## 9. Development plan (milestones)

Each milestone produces a runnable binary + passing tests. Don't skip ahead.

### M1 — Core math, headless (1–2 sessions)
- `core/types.h`, `link`, `joint`, `mechanism` compile
- FK works for a hardcoded planar 2R arm
- `tests/test_fk.cpp` passes all Test 1 cases
- **No UI yet.** Binary is a CLI that prints FK results.

### M2 — Jacobian + finite-difference tests
- `core/jacobian.cpp`
- `tests/test_jacobian.cpp` verifies J against finite differences for 2R, 3R, SCARA presets
- Tolerance: 1e-6 (finite diff won't be better).

### M3 — DLS IK solver
- `core/ik/dls.cpp` with joint limits + adaptive damping
- `tests/test_ik_dls.cpp` — for planar 2R, verifies IK reaches reachable targets within tolerance; for unreachable targets, verifies it produces a valid best-effort pose without NaN or divergence.

### M4 — JSON loader + presets
- Can load `assets/presets/planar_2r.json` and reproduce hardcoded test case
- `tests/test_io.cpp`

### M5 — OpenGL viewer (minimal)
- GLFW window, orbit camera, grid
- Render mechanism as capsules + joint frames (RGB = XYZ axes)
- No interaction yet — just visualization from preset
- **Milestone demo:** launch binary, see 2R arm rendered, rotate camera

### M6 — ImGui sliders for FK
- Per-joint sliders drive `q`, FK updates, viewport redraws
- Mechanism picker dropdown
- User sanity-check: slide joint 1 of a 2R arm → arm rotates around base

### M7 — ImGuizmo IK target
- Draggable 3D handle in viewport
- Every frame: target pose → DLS IK → new q → FK → render
- Toggle between FK mode and IK mode

### M8 — Mechanism editor
- Add/remove joints via UI
- Edit link lengths, joint axes, limits
- Save/load to JSON
- This is the biggest UX milestone and worth its own session.

### M9+ — Polish
- CCD/FABRIK as selectable solvers for comparison
- Joint limit visualization (color joints red when at limit)
- IK solution enumeration for redundant / multi-solution mechanisms
- URDF importer (v2 territory)
- Closed-loop mechanisms (v2 territory — separate design doc)

---

## 10. Pitfalls — known failure modes to avoid

1. **Editor-first development.** Tempting, wrong. Build the math core and unit-test it against analytical cases BEFORE the viewer. When the arm renders wrong, you won't know if it's the math, the renderer, the camera, or the UI.

2. **Silent convention drift.** FK might work in one file because it assumes column-major extrinsic Euler, and the Jacobian works in another because it assumes quaternion. Both pass their own tests. They break when composed. → Write down conventions in `docs/math.md` and reference that doc in every file that does math.

3. **Single-precision creep.** Someone adds `float` somewhere, it gets upgraded to `double`, gets downgraded, loses precision in a quaternion normalization, cumulative error breaks IK convergence. → Core is `double`, period.

4. **Gimbal lock in orientation error.** Do NOT compute orientation error as `R_target.eulerAngles() - R_current.eulerAngles()`. Use axis-angle of `R_error = R_target * R_current.transpose()`.

5. **Jacobian expressed in the wrong frame.** The Jacobian can be body-frame (expressed in ee frame) or space-frame (world frame). DLS expects the error vector and Jacobian in the **same** frame. → We use space-frame (world) for both. Document this.

6. **Not clamping IK step size.** A large error with a small damping factor can produce a `Δq` that rotates a joint by π radians, flying past the target and oscillating. → Clamp `||Δq|| ≤ Δq_max` (e.g. 0.1 rad) per iteration.

7. **ImGui coordinate mismatch with OpenGL.** ImGuizmo uses the matrix you hand it; make sure you pass view/projection consistent with your renderer. Classic bug: gizmo moves but mechanism doesn't follow, because the gizmo transform is in a different basis.

8. **GLFW+ImGui event capture.** When ImGui wants the mouse (`io.WantCaptureMouse`), don't also pass the click to the orbit camera. Check the flag.

9. **Slow agent loop.** If `cmake --build && ctest` takes >30s, the agent loop becomes painful. Keep tests fast. Split expensive stress tests into a `ctest -L slow` label that's not run by default.

---

## 11. Testing discipline

- **Every new function in `core/` gets a test.** No exceptions.
- Tests are categorized:
  - **Fast** (default): <5s total. FK, Jacobian, IK convergence on small mechanisms.
  - **Slow** (`-L slow`): stress tests, large random mechanism batteries.
- **Parametric tests** via GoogleTest `TEST_P` for the mechanism test battery — add a new preset and it's covered automatically.
- **Finite-difference Jacobian test** is the single most valuable test in the project. It catches indexing bugs, sign errors, frame errors, and axis errors. Write it early.
- **No test should require the viewer.** If you're tempted to write a visual-verification-only test, instead dump the failing state to JSON and write a unit test that loads and checks it.

When a test fails, first question: "is the test right?" Check against the analytical solution with a calculator. The test is wrong surprisingly often.

---

## 12. Reference material (read as needed)

### Math / IK theory
- Samuel Buss, *Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares Methods* (2009). The canonical reference. Covers all the singularity-avoidance math.
  https://mathweb.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
- Buss & Kim, *Selectively Damped Least Squares for Inverse Kinematics* (2005). If we want a smarter default damping strategy later.
  https://mathweb.ucsd.edu/~sbuss/ResearchWeb/ikmethods/SdlsPaper.pdf
- Chiaverini, *Review of the Damped Least-Squares Inverse Kinematics with Experiments on an Industrial Robot Manipulator* (IEEE T-CST 1994). Experimental behavior near singularities.
- Featherstone, *Rigid Body Dynamics Algorithms* (2008). Overkill for v1 but gold-standard for any future dynamics work.
- Lynch & Park, *Modern Robotics* (free online). Chapters 4–6 cover FK/Jacobian/IK cleanly. Our twist convention `[v; ω]` matches this book.

### Libraries for reference (look at their implementations, don't depend on them)
- Pinocchio — https://github.com/stack-of-tasks/pinocchio — production-grade C++ kinematics/dynamics. Their joint type hierarchy is worth studying.
- RBDL — https://github.com/rbdl/rbdl — Featherstone's algorithms in C++.
- Dear ImGui — https://github.com/ocornut/imgui
- ImGuizmo — https://github.com/CedricGuillemet/ImGuizmo

### A project that does almost exactly this (WebAssembly, open source)
- Robin Chen's "Interactive Robot Kinematics" — ImGui + ImGuizmo + ImPlot, 6-DOF arm with DH params, FK + IK, 8-solution enumeration. Good UX reference.
  https://robincpc.github.io/2023/11/07/Interactive-Robot-Kinematics-Hands-On-with-ImGui-and-ImGuizmo/

### C++ / Claude Code practice
- Use `-fsanitize=address,undefined` for all debug builds. If sanitizers report anything, fix it before moving on.
- Use `ccache` + `ninja` to keep the compile-test loop tight.
- When a build error wall appears, read the FIRST error, not the last. Template error cascades are usually one root cause.

---

## 13. Agent-specific notes (Claude Code: read this)

- **Prefer small diffs.** Touching 30 files in one change makes test failures ambiguous. One logical change per commit; one commit per passing test.
- **Run tests after every edit.** `ctest --test-dir build --output-on-failure`. Don't batch changes and run tests at the end.
- **If a test fails, read the test assertion before editing production code.** The test is often asking the question precisely — if your code disagrees, understand why.
- **When adding a new solver or joint type, add the test FIRST**, watch it fail with the expected message, then implement until it passes.
- **Don't silently install system packages.** If a dep is missing, add it via CMake FetchContent rather than `apt install`.
- **Don't add new top-level dependencies without updating this file.** If you pull in a library, it goes in section 3.
- **If something in this file is wrong or stale, fix it in the same PR as the code change.** This file should always reflect reality.
