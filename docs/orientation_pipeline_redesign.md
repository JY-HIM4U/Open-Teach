# Orientation Pipeline Redesign Plan

**Status: proposed, not yet implemented.** This replaces the empirical
`orient_remap` / `orient_flip` calibration described in
`docs/hand_to_robot_mapping.md` §9.5–9.6 with a deterministic four-stage
architecture: (1) source handedness conversion, (2) one fixed frame transform
`C`, (3) clutch with an explicit spatial-delta convention, (4) glitch
filtering. **Position mapping and robot tracking are untouched.**

Estimated effort: 1–2 days implementation, half a day validation.

Background: `docs/hand_to_robot_mapping.md` §9.3 ("Everything is measured
relative to a reset") is the specific defect this plan eliminates — the
current pipeline mixes spatial and body rotation deltas depending on where
they're composed, which makes the correct `orient_remap` dependent on the
reset pose and therefore impossible to calibrate once and trust.

---

## Phase 0 — Ground-truth checks (no code changes, ~1 hour)

Do these first. Each one prevents a silent failure later.

**0.1 Determine `wq*` provenance.** Unity's XR Hands API may already deliver
OpenXR data converted into Unity coordinates. If we convert again, handedness
is applied twice and the bug reappears wearing a disguise. Test: perform one
known physical rotation (roll the palm ~90° clockwise as seen from the
headset), read the logged quaternion, and check which convention it matches.
Record the answer in the config as `source_frame: unity_lh` or
`source_frame: openxr_rh`. This is the only empirical step in the entire
redesign, and it is a single binary check.

**0.2 Check `det(axis_remap)`.** If it is −1, the handedness reflection is
embedded in the position map. Note the result; Phase 2 depends on it.

**0.3 Audit the current delta composition.** Find where the operator computes
the relative rotation. Determine: is it `R_t @ R_0.T` (spatial) or
`R_0.T @ R_t` (body), and is it pre- or post-multiplied onto the robot reset
orientation? If a body delta is being pre-multiplied (or vice versa), that
mixed composition alone produces the reset-dependent axis behavior described
in `hand_to_robot_mapping.md` §9.3 and may be the entire original bug. Write
down what you find before changing it.

---

## Phase 1 — Canonicalize the Quest data at the source

**1.1** Implement one conversion function operating on rotation matrices,
never on quaternion components:

```python
def change_rotation_basis(R_source, B):
    R = B @ R_source @ B.T
    u, _, vt = np.linalg.svd(R)          # project onto SO(3)
    R = u @ vt
    if np.linalg.det(R) < 0:
        u[:, -1] *= -1
        R = u @ vt
    return R
```

Positions convert as `p_rh = B @ p_lh`. For the standard z-flip,
`B = diag(1, 1, -1)`. If Phase 0.1 shows the data is already right-handed,
`B = I` and this stage is a pass-through — **keep the stage in the pipeline
anyway so the boundary exists.**

**1.2** Log both `wq_raw*` (as received) and `wq_rh*` (canonicalized). All
downstream analysis uses `wq_rh*` only.

**1.3** Delete `orient_flip` from this layer. Handedness is no longer a
tunable.

### Tests (pytest, no hardware)

- `Rz(θ)` conjugated by `diag(1,1,-1)` equals `Rz(θ)` (the sanity identity).
- Round-trip: converting twice with the same `B` returns the original.
- Fixture test against one recorded clip with the known rotation from Phase
  0.1: converted axis and sign must match the physical motion.

---

## Phase 2 — One fixed frame transform C

**2.1** Extract `C` from the validated position map. If `det(axis_remap) = +1`,
then `C = axis_remap` directly. If −1, factor: `axis_remap = C @ B_residual`
where `B_residual` is the reflection now handled in Phase 1, leaving
`C ∈ SO(3)`. Verify `CᵀC = I`, `det(C) = +1`.

Note: conjugation by an improper orthogonal matrix is mathematically valid,
so the factoring is not a correctness requirement — it is so `C` can be
stored as a quaternion, logged as a distinct stage, and diagnosed separately
from handedness. Do it anyway.

**2.2** Config change: add `frame_transform` (quaternion or 3×3, one value).
Deprecate `orient_remap` and `orient_flip`: if either is set to a non-default
value, refuse to start with a clear error pointing at this plan. Silent
fallback is how the old and new semantics would get mixed.

**2.3** Do NOT fit `C` from closed-loop teleop logs. The achieved robot
motion is the commanded map applied to hand motion; a Kabsch fit recovers the
existing controller tautologically. Log fitting is a regression check only
(Phase 5).

---

## Phase 3 — Clutch with explicit spatial deltas

**3.1** On teleop resume, snapshot four values:

```python
R_hand_reset, p_hand_reset = R_hand, p_hand      # canonicalized (post-Phase-1)
R_robot_reset, p_robot_reset = R_robot, p_robot
```

**3.2** Per control cycle, spatial mode (default):

```python
dp = p_hand - p_hand_reset
p_target = p_robot_reset + scale * (C @ dp)

dR_world = R_hand @ R_hand_reset.T               # spatial delta
R_target = (C @ dR_world @ C.T) @ R_robot_reset  # delta PRE-multiplies
R_target = project_SO3(R_target)
```

**3.3** Body mode, behind config flag `orientation_mode: spatial | body`:

```python
dR_local = R_hand_reset.T @ R_hand               # body delta
R_target = R_robot_reset @ (D @ dR_local @ D.T)  # delta POST-multiplies
```

with `D` mapping hand-local to tool-local axes (start with `D = C`).

**3.4** Guard against convention mixing: name the variables `dR_world` /
`dR_local` and add an assertion-level comment at each composition site. **A
body delta pre-multiplied as if spatial reproduces exactly the old
reset-dependent behavior — this is the one bug this architecture cannot
survive.**

### Tests

- **Property test:** for random reset poses and random hand deltas, the
  commanded robot delta axis must equal `C @ (hand delta axis)` and be
  independent of both reset orientations. This test is the formal statement
  of the whole redesign; if it passes, §9.3 is dead.
- **Identity test:** zero hand motion ⇒ `R_target == R_robot_reset` exactly.
- **Composition test:** two sequential deltas equal their product applied
  once.

---

## Phase 4 — Filtering order

Keep all existing glitch handling; conjugation fixes frame semantics, not
Quest tracking. Fixed pipeline order:

```
raw Quest pose
  → Phase 1 source conversion
  → quaternion hemisphere / sign continuity
  → angular-velocity threshold (orient_glitch_deg) + 1-2 frame dropout drop
  → optional SLERP smoothing
  → delta computation (Phase 3)
```

Filters run after canonicalization and before the delta. Thresholds carry
over unchanged.

---

## Phase 5 — Validation

**5.1 Dry-run, single-axis** (now validation, not calibration). Three slow
motions — roll, pitch, yaw. Expected: each drives the predicted base axis
with the predicted sign, first try, no tuning. If any axis is wrong, the
failure is in Phase 0.1 (provenance) or Phase 2.1 (factoring) — **do not
reintroduce per-axis knobs to patch it.**

**5.2 Reset-independence check.** Repeat 5.1 from three deliberately
different reset hand orientations. Axis correspondence must not change. This
is the direct empirical test of the §9.3 fix.

**5.3 Regression script.** Repurpose `fit_orient_remap.py` as
`check_frame_consistency.py`: given any clip, Kabsch-fit the transform
between hand and commanded deltas and assert it matches the configured `C`
within tolerance. Require ≥2 non-parallel rotation axes in the clip or report
"insufficient excitation" instead of a fit. Run it on every recorded session
as a cheap invariant.

**5.4 Go live in Low Resolution.** Then A/B spatial vs body on one
CRADLE-relevant task each (e.g., pouring and cap-twisting) before choosing
the default for data collection.

---

## Phase 6 — Documentation

- Rewrite the `hand_to_robot_mapping.md` §9 conclusion: orientation is a
  deterministic frame conversion plus one fixed transform shared with
  position; per-axis empirical tuning is unnecessary; calibration reduces to
  validating one world-to-base transform.
- Document the Phase 0.1 provenance result and the chosen `B` prominently —
  it is the one setup-specific fact the next person cannot derive.
- Mark `orient_remap` / `orient_flip` docs as historical.

---

## Rollback and risk

- Keep the old orientation path behind `orientation_pipeline: legacy` for one
  session. Delete it after 5.1–5.2 pass; a lingering legacy path is a
  convention-mixing hazard, not a safety net.
- **Abort criteria during 5.4:** any commanded jump > `orient_glitch_deg` that
  the filter passes through, or any axis response that differs between
  resets. Both indicate a Phase 0/2 error, not a tuning problem.

---

## Order of work

Phase 0 (1 h) → Phases 1–3 with tests (1 day) → Phase 4 (trivial, hours) →
Phase 5 (half day, robot time) → Phase 6 (1 h). Phases 1–3 are pure software
and fully testable without the arm.
