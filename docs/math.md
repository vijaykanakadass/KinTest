# Math conventions

This is the single source of truth for the geometric conventions used in
`src/core/`. Every file that does math should match what's written here. If code
and this doc disagree, one of them is a bug — fix it, don't paper over it
(CLAUDE.md §10.2).

## Transforms

- Rigid transforms are `Eigen::Isometry3d`, aliased `kp::Transform`.
- `T_ab` means "frame *a* expressed in frame *b*": `p_b = T_ab * p_a`. Subscripts
  read right-to-left and are written into variable names (`T_world_ee`).
- Forward kinematics composes, per joint,
  `T_parent_child = T_parent_jointOrigin · JointTransform(type, axis, q)`.
- `JointTransform` is `Rz/​AngleAxis(q, axis)` for revolute, `Translation(q·axis)`
  for prismatic, identity for fixed.

## Units

Meters, radians, seconds. Never mixed. Orientation is never stored as Euler
angles internally (convert only at the UI/IO boundary).

## Twists and the Jacobian

- 6-vectors are `[v; ω]` — linear velocity stacked on angular velocity
  (Modern Robotics convention). This holds for Jacobian columns and error twists.
- The geometric Jacobian is expressed in the **world (space) frame**. The IK
  error twist must be in the same frame (CLAUDE.md §10.5).
  - Revolute column *i*: `[ω_i × (p_ee − p_i); ω_i]`
  - Prismatic column *i*: `[â_i; 0]`
  where `ω_i`/`â_i` is the joint axis in world coordinates and `p_i` the joint
  origin in world coordinates, both taken in `T_world_parent · origin` (before
  the joint's variable motion).

## Orientation error

For a target rotation `R_target` and current `R_current`, the error is the
axis-angle vector of `R_err = R_target · R_currentᵀ` (length-3, `axis·angle`
from `Eigen::AngleAxisd`). Never subtract Euler angles — that gimbal-locks
(CLAUDE.md §10.4).

## Denavit–Hartenberg (import helper only)

DH is **not** the native representation; it is an import convenience
(`core/dh.{h,cpp}`). We use **standard / distal DH**, with the per-row transform

```
A_i(q) = Rz(θ_i) · Tz(d_i) · Tx(a_i) · Rx(α_i)
```

- Revolute joint: `θ_i = θ + q`, `d_i = d`.
- Prismatic joint: `d_i = d + q`, `θ_i = θ`.

Because the joint variable sits *between* fixed sub-transforms, a DH row cannot
be expressed as a single `origin · JointTransform` pair. `mechanism_from_dh`
therefore folds each row's trailing fixed part `Tz(d)·Tx(a)·Rx(α)` (revolute) or
`Tx(a)·Rx(α)` (prismatic) into the *next* joint's origin, with a final fixed
joint carrying the last row's tail out to the end-effector. End-effector FK then
equals the exact DH product `A_1(q_1)·…·A_n(q_n)`; intermediate link frames are
**not** the DH frames, which does not affect kinematics. This identity is
verified directly in `tests/test_dh.cpp`.
