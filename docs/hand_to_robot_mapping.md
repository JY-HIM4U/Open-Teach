# Wrist/Hand Tracking → Franka + Inspire RH56 Mapping

How Quest hand tracking becomes robot motion, for the `inspire_franka` config
(`FrankaArmOperator` + `InspireHandOperator` running side by side off one
Quest stream). Focused on **orientation**, since that's the part most likely
to feel "off" — this doc ends with a concrete, code-derived explanation of a
likely cause.

Code referenced: `openteach/components/detector/keypoint_transform.py`,
`openteach/components/operators/franka.py`, `openteach/robot/franka.py`,
`openteach/components/operators/inspire_hand.py`,
`openteach/robot/inspire/inspire_retargeter.py`,
`configs/robot/inspire_franka.yaml`.

---

## 1. One Quest stream, two independent consumers

```
Quest (OVRSkeleton bone positions, 24 keypoints incl. wrist)
        |
        v
keypoint_transform.py : TransformHandPositionCoords
        |
        +--> 'transformed_hand_coords'  (24x3, wrist-relative,
        |     rotated into the palm's OWN local frame)
        |         |
        |         v
        |     InspireHandOperator -> RH56 finger curl (6 DOF)
        |     [orientation-invariant by construction, see §4]
        |
        +--> 'transformed_hand_frame'   (4x3: origin + 3 basis vectors,
              published only on 'absolute'-tagged frames)
                  |
                  v
              FrankaArmOperator -> Franka EE pose (position + orientation)
              [THIS is the orientation-sensitive path, see §3]
```

The two operators are separate processes reading the same ZMQ topics
(`inspire_franka.yaml`, "BOTH run as separate processes off the same keypoint
stream"). Nothing synchronizes them frame-to-frame beyond that.

---

## 2. Raw read and the two derived coordinate frames

Every Quest frame gives 24 bone positions in Unity world space (left-handed:
X=right, Y=up, Z=forward), bone 0 = wrist. `TransformHandPositionCoords`
(`keypoint_transform.py`) derives **two different things** from the same
knuckle geometry each frame:

```python
palm_normal = sign_continuous(normalize(cross(index_knuckle, pinky_knuckle)))
```
`_sign_continuous_normal` (line 38) keeps this vector on the same hemisphere
as the previous frame — `cross()` flips sign near-degenerate knuckle geometry
(happens during wrist roll), which used to invert the whole frame ~180° for a
frame or two. This is a real fix for a real bug: **without it, fast wrist roll
could make the reconstructed frame snap backwards.**

- **Finger-curl coordinates** (`transformed_hand_coords`): all 24 keypoints,
  recentered on the wrist and then **rotated into the palm's own basis**
  (`_get_coord_frame` + the `rotation_matrix @ translated_coords.T` in
  `transform_keypoints`, lines 61-90). Because this rotates *into* the hand's
  own frame, whatever the wrist's orientation is gets factored out — finger
  curl comes out the same whether your palm faces up, down, or sideways.

- **Wrist/arm frame** (`transformed_hand_frame`): `_get_hand_dir_frame`
  (line 67) returns `[origin, X, Y, Z]` where `origin` is the **raw, un-rotated
  wrist position** (translation only follows real-world wrist motion 1:1 at
  this stage) and `X/Y/Z` are the palm-normal/palm-direction/cross-product
  basis vectors (this **is** the orientation signal — it changes every time
  you rotate your wrist, by design, since it's what drives EE orientation).

---

## 3. Franka arm: position

`FrankaArmOperator._apply_retargeted_angles` (`operators/franka.py`, from
line 311). Each frame:

1. `H_HI_HH` = wrist frame **at the last teleop reset** (captured once,
   `_reset_teleop`, line 292 — held fixed until the next pause→resume).
2. `H_HT_HH` = wrist frame **now**.
3. `H_HT_HI = pinv(H_HI_HH) @ H_HT_HH` — the wrist's motion since reset,
   expressed in the coordinate basis the wrist had *at reset time*.
4. `H_RT_RH = H_RI_RH @ H_A_R @ H_HT_HI @ pinv(H_A_R)` — conjugate that
   motion through `H_A_R` (a **fixed** matrix: −45° rotation about Z plus a
   6 cm Z offset, originally sized for the Allegro-hand mount — see line
   354-359) and add it on top of the robot's pose at reset.
5. `disp = axis_remap @ (H_RT_RH[:3,3] - H_RI_RH[:3,3])`, then
   `flip_vertical` optionally negates the Z component (lines 364-372).
   `axis_remap` (config, `inspire_franka.yaml` lines 96-99) is the
   per-installation calibration for "which robot axis does each hand
   direction drive" — currently set to
   `[[0,0,1],[1,0,0],[0,1,0]]` (hand-forward→robot X, hand-right→robot Y,
   hand-up→robot Z).
6. Resolution scale (Low=0.6×, High=1×, pinky-pinch toggle), optional
   `Filter` SLERP/LERP smoothing, a ±`workspace_box` clamp around the reset
   pose, and a same-frame jump-rejection safety guard (`MAX_FRAME_JUMP_M`)
   are applied after this.

So: **position tracks raw world-space wrist translation since reset**, passed
through a fixed −45° rotation (`H_A_R`) and *then* through the configurable
`axis_remap`/`flip_vertical`.

---

## 4. Franka arm: orientation (the part you asked about)

```python
Rw = self.hand_moving_H[:3, :3]                       # line 340
# glitch guard: reject a >orient_glitch_deg jump in one frame, hold last-good
...
R_rel = H_HT_HI[:3, :3]                                # line 379: wrist rotation since reset,
                                                        # expressed in the wrist's OWN reset-time basis
C = self._orient_C                                     # = orient_remap, else axis_remap, else I
R_robot_rel = C @ R_rel @ C.T                           # line 381: conjugate into the robot's basis
H_RT_RH[:3, :3] = H_RI_RH[:3, :3] @ R_robot_rel          # line 382: apply on top of the robot's reset orientation
```

Two independent noise-rejection layers exist purely for orientation:

- **Sign-continuity** (§2, in `keypoint_transform.py`) — fixes the
  cross-product flip **at the source**.
- **`orient_glitch_deg`** (default 45°, `franka.py` lines 334-347) — a second,
  per-frame safety net in the operator: if the reconstructed wrist rotation
  jumps more than this in one frame, it's treated as a tracking glitch and
  the last good orientation is held instead.

**Reset semantics matter for orientation exactly like they do for position:**
`R_rel` is relative to whatever your wrist's rotation was *at the moment you
last resumed teleop* (`_reset_teleop`, called on the STOP→CONT pause
transition). If your wrist wasn't in a clean, repeatable pose at that instant,
that offset is baked into the whole session's orientation mapping.

`orient_follow=False` reproduces the old behavior, where orientation was
never independently retargeted — the comment at line 72-80 states plainly:
*"the original code only mapped wrist orientation through the Allegro mount
rotation, with no Unity→robot handedness fix ... the robot didn't follow the
wrist."* `orient_follow=True` (current default in `inspire_franka.yaml`) is
the fix — but see §5, it isn't calibrated the same way position is.

---

## 5. Known asymmetry — likely why position and orientation don't feel aligned

Position and orientation are **not run through the same calibration**, even
though they're both meant to represent "the robot doing what your hand did":

| | goes through `H_A_R` (fixed −45° + Allegro's 6 cm offset)? | goes through `axis_remap`/`orient_remap`? |
|---|---|---|
| **Position** (`disp`, line 368) | **Yes** — baked into `H_RT_RH` before `disp` is even computed | Yes, applied again on top (line 369) |
| **Orientation** (`R_robot_rel`, line 381) | **No** — `orient_follow` bypasses `H_A_R` entirely, operating directly on `R_rel` | Yes (as `C`, defaults to `axis_remap`) |

Concretely: position's effective calibration is *(fixed 45° rotation) then
(your configured `axis_remap`)*; orientation's is *only* your configured
`axis_remap`. `axis_remap` can make **one** of those look correct, but not
both at once, because it's being asked to compensate for two different
things depending on which channel you're looking at. This also means
wrist *rotation* leaks a bit into the commanded *position*, through the fixed
6 cm offset inside `H_A_R` (`R_ar @ R_rel` term) — already flagged in
`docs/inspire_franka_teleop.md` item 4: *"Rotating your wrist will swing the
commanded arm position by roughly this offset."* That offset is sized for the
Allegro mount, not the RH56.

**This is the most likely concrete explanation for "not perfectly aligned":**
you calibrated (or are perceiving) one channel while the other silently used
a different fixed rotation.

Two ways to fix it (not applied here — this is documentation, not a code
change):
- Fold the same `C = axis_remap`/`orient_remap` conjugation into the
  **position** path too, and drop `H_A_R`'s rotation (keep only its mount
  translation, updated to the RH56's actual offset) — so both channels share
  one calibration.
- Or, keep `H_A_R` for position but explicitly set `orient_remap` to
  `H_A_R`'s rotation block (`Rz(−45°)`) composed with whatever `axis_remap`
  represents, so orientation matches the same effective transform position
  already gets.

Use the `wq0..wq3` columns the recorder logs (`franka.py` lines 180-182,
467-473 — the raw Quest wrist quaternion) alongside `tgt_*`/`cur_*` to
calibrate: rotate your wrist through a known axis (e.g. pure roll) and check
whether the EE's commanded quaternion rotates about the axis you expect.

See §8 for a concrete calibration plan using our measured RH56 mount
dimensions, including a second candidate fix that avoids the rotation/position
coupling this offset currently causes.

---

## 6. Inspire RH56: finger curl, and why it has no orientation of its own

`InspireHandOperator` / `InspireRetargeter` (`inspire_retargeter.py`) only
ever look at `transformed_hand_coords` — the **palm-local, rotation-canonicalized**
stream from §2. For each finger it sums the bend angle across consecutive
joints (`calculate_angle`, geometrically rotation-invariant regardless) and
linearly maps it from a calibrated `[open_deg, closed_deg]` range to a
`0..1000` RH56 command:

```
FINGER_TO_DOF = {pinky:0, ring:1, middle:2, index:3}   # + thumb_bend:4, thumb_rot:5
```

**The RH56 has no independent orientation channel.** It is rigidly bolted to
the Franka flange, so its orientation in the world is 100% inherited from
whatever pose `FrankaArmOperator` commands the Franka's end-effector to (§3,
§4) plus the fixed physical mounting rotation. Quest hand *orientation* never
reaches the Inspire operator at all — only the 6 curl scalars do. So: if
finger curl looks right but the whole hand's pointing direction feels wrong,
that is entirely a Franka-orientation-mapping issue (§4/§5), not anything in
`inspire_retargeter.py`.

One real coupling exists in the other direction, already noted in
`docs/inspire_franka_teleop.md` item 1: the retargeter assumes handedness
match (right hand → right RH56); a mismatch mirrors the finger→DOF mapping,
not the arm's orientation.

---

## 7. Config knob reference

| Knob | File | Affects | Default (`inspire_franka.yaml`) |
|---|---|---|---|
| `sign_continuity` | transform config | frame reconstruction (both position-frame origin and orientation basis) — fixes cross-product flips at the source | `True` |
| `axis_remap` | franka operator | position (`disp`) **and** default `C` for orientation if `orient_remap` is unset | `[[0,0,1],[1,0,0],[0,1,0]]` |
| `flip_vertical` | franka operator | position Z only | `False` |
| `orient_follow` | franka operator | whether EE orientation tracks the wrist at all | `True` (see `${orient_follow}`) |
| `orient_remap` | franka operator | orientation `C`, overrides `axis_remap` for orientation only | `null` |
| `orient_flip` | franka operator | folds a `diag(±1,±1,±1)` axis reflection into `C` | `${orient_flip}` |
| `orient_glitch_deg` | franka operator | per-frame orientation-jump rejection threshold | `45.0` |
| `filter_comp_ratio` | franka operator | smoothing lag for both position and orientation (SLERP) | `0.5` |
| `workspace_box` | franka operator | position-only safety clamp around reset pose | from `${workspace_box}` |
| `H_A_R` (hardcoded, not a config knob) | `franka.py` line 354 | position only when `orient_follow=True`; both when `orient_follow=False` | fixed −45°/Z + 6 cm (Allegro-specific, wrong for RH56 — see §8) |
| `tool_offset` (**planned, not implemented**) | franka operator | position only, as a rigid post-hoc offset (§8 Option 2) | `[0, 0, 0.1445]` (measured, RH56 flange→palm) |
| `calibration` (per-DOF) | inspire operator | finger curl→command range, no orientation effect | `null` (needs H2 calibration) |
| `thumb_rot_fixed` | inspire operator | locks DOF5, no orientation effect | `${thumb_rot_fixed}` |

For run/setup/safety procedure (not covered here), see
`docs/franka_validation.md`, `docs/franka_quest_teleop.md`, and
`docs/inspire_franka_teleop.md`.

---

## 8. RH56 mount calibration — measured dimensions and planned fix

**Status: plan only, not yet implemented in code.** This is the concrete
follow-up to the §5 asymmetry, using our actual RH56 mount measurements.

### Measured (hand flat/open, straight-line along the mount axis)

| Measurement | Value |
|---|---|
| Flange face → middle fingertip | 226 mm |
| Middle finger length (MCP joint → fingertip) | 81.5 mm |
| **Flange face → middle finger MCP / palm base** | **226 − 81.5 = 144.5 mm** |

144.5 mm is the number we want: a rigid, grip-state-independent reference
point. The fingertip (226 mm) is *not* usable directly as the calibration
constant, because it moves relative to the flange as `InspireHandOperator`
curls the finger — 144.5 mm (the palm/MCP) does not move regardless of grip
state. This is the RH56 equivalent of `H_A_R`'s hardcoded `-0.06` ("the height
of the allegro mount is 6cm", `franka.py:358`), which was sized for the
Allegro hand and is wrong for this mount.

Assumption: flange face, MCP joint, and fingertip lie on the same straight
line (hand mounted centered, no lateral offset) — confirmed in our earlier
calibration discussion.

### Option 1 — Drop-in replacement (minimal change)

Replace the `-0.06` inside `H_A_R` (`franka.py:358`) with `-0.1445`, keeping
the existing conjugation `H_A_R @ H_HT_HI @ pinv(H_A_R)` (line 362)
structurally unchanged.

- **Pro:** one-line change, no restructuring, no dependency on the §5
  rotation-calibration work being done first.
- **Con:** this offset is *inside* the conjugation, so it isn't a passive
  constant — expanding the matrix product produces a term equivalent to
  `R_ar @ R_rel` scaled by the offset, i.e. wrist **rotation** swings the
  commanded **position**. `docs/inspire_franka_teleop.md` item 4 already
  flags this for the original 6 cm figure; at 144.5 mm the swing is roughly
  **2.4× larger** (up to ~29 cm at a 180° wrist rotation, vs ~12 cm before).
  This keeps the coupling bug alive at a bigger magnitude — it does not fix
  the "not perfectly aligned" feeling, only updates one wrong constant to a
  different (correct-magnitude, still-coupled) one.

### Option 2 — Clean tool-frame offset (structural fix)

Add a `tool_offset` config knob = `[0, 0, 0.1445]`, expressed in the flange's
local frame, and stop routing it through the `H_A_R` conjugation entirely.
Instead:
1. Run the wrist retargeting math (§3/§4, with the §5 fix applied so position
   and orientation share one calibrated rotation `C`) to get a target pose for
   the **palm**, not the flange.
2. Solve backward for the flange pose that places the palm there:
   `target_flange_pose = target_palm_pose ∘ inverse(flange_to_palm)`, where
   `flange_to_palm` is the fixed `[0,0,0.1445]` translation (no rotation
   component, per our centered-mount assumption).
3. Command `target_flange_pose` to the robot (this is what `arm_control`
   actually sends — the flange, not the palm, since that's what
   `franka_arm`/Deoxys controls).

- **Pro:** the offset becomes a true rigid constant — rotating the wrist
  rotates the palm about *itself*, not about the flange ~14.5 cm away. This
  is what "matches my real hand" actually requires: right now, any wrist
  rotation currently swings the palm through a wide arc that has no
  equivalent in how your real wrist/hand moves.
- **Con:** bigger change. It only makes sense *after* §5's rotation asymmetry
  is fixed (position and orientation sharing one `C`) — solving backward
  through an offset is meaningless if position and orientation are still
  calibrated to two different rotations.

### Recommended order

1. Zero out `H_A_R` (both rotation and translation) temporarily.
2. Calibrate `axis_remap`/`orient_remap` per §5's dry-run 3-direction +
   single-axis-rotation procedure, so position and orientation share one `C`.
3. Implement Option 2 using `tool_offset = [0, 0, 0.1445]`.
4. Re-validate in dry-run (`docs/franka_validation.md` Stage 2), then go live
   starting in Low Resolution.

Option 1 is a fallback if time-constrained, with the explicit understanding
that it does not resolve the rotation/position coupling — only Option 2 does.

---

## 9. Why orientation is harder than position (plain-language)

A recurring, reasonable question: *"Once we command an orientation, the Franka
just tracks it — so why is orientation hard to get right?"*

**The robot tracking the orientation is the easy part, and it already works.**
Command the Franka any end-effector orientation and it reaches it accurately;
the recorded `quat*` (commanded) vs `cur_q*` (achieved) columns confirm the arm
follows. The difficulty is **not** the robot. It is the *translation layer*
that decides **which** orientation to command from your hand's orientation.
Four properties make that translation hard — and only for rotation, not
position:

### 9.1 Position is three independent sliders; rotation is not
Position `x/y/z` are independent. "Hand moves right → robot +Y" is a per-axis
rule you can fix one axis at a time without disturbing the others — which is why
`axis_remap` is easy to tune and position "feels right" quickly. Rotations are
**coupled**: rotating about one axis changes where the other two axes point, and
composition order matters (`roll∘pitch ≠ pitch∘roll`). There is no knob that
remaps one rotation axis while leaving the other two untouched. This is the
single biggest reason position calibrates in minutes and orientation does not.

### 9.2 Two coordinate worlds that disagree on handedness
The Quest reports the hand in Unity space: **left-handed, X=right, Y=up,
Z=forward** (§2). The robot base frame is **right-handed, Z-up** (see the Franka
base-frame note). Converting a *rotation* between a left-handed and a
right-handed frame is exactly where "mirrored / backwards / inverted-turn" bugs
come from. Position only picks up a sign flip; a rotation can have its entire
sense of "clockwise vs counter-clockwise" inverted. `orient_flip`
(`diag(±1,±1,±1)`) exists to absorb these per-axis sign inversions.

### 9.3 Everything is measured relative to a reset
The pipeline maps the *change* in wrist orientation since the last teleop
resume, not an absolute pose (§4, `R_rel = H_HT_HI[:3,:3]`). At the reset
instant the hand has some arbitrary orientation and the robot has its own
(`H_HI_HH`, `H_RI_RH`). The mapping is therefore
"change-from-hand-reset → change-from-robot-reset", and those two arbitrary
reset poses **rotate the whole axis correspondence**. Practical consequence:
the correct `orient_remap` **cannot be read directly off a recording**, because
the robot's reset-orientation offset rotates whichever base axis a given hand
twist appears to drive. This is why the axis map is calibrated empirically
(set → test → correct signs), not fit from logs.

### 9.4 The tracking noise hits rotation specifically
The Quest occasionally emits a ~180°-flipped hand pose for a frame or two
(§2, §4). Position barely notices; orientation is destroyed by a 180° flip.
This is handled — `sign_continuity` at the source and `orient_glitch_deg` in the
operator reject the flips, so the *commanded* orientation stays smooth (the
robot never sees them). Note the flips still appear in the **logged `wq*`
column**, which is recorded pre-guard: that is a cosmetic logging artifact, not
a robot fault. Analyze the commanded `quat*` (post-guard) for the true behavior.

### 9.5 So what is `orient_remap`?
A 3×3 signed-permutation table: *"a twist about THIS hand axis becomes a twist
about THAT robot axis, in THIS direction."* It is the orientation twin of
`axis_remap` (which does the same for position `x/y/z`). Defaults to `null`,
in which case orientation reuses `axis_remap` as its `C` (§4, §7). Because of
9.1–9.3, the correct entries can't be read off the sensor — you set it, do one
slow single-axis motion per axis, observe, and correct any sign with
`orient_flip`. That one-time calibration is the entire remaining task; the
reconstruction, the robot tracking, and position are already working.

### 9.6 Calibration procedure (empirical, ~2 minutes)
1. Record **one slow, isolated motion per axis** — roll alone, then pitch alone,
   then yaw alone (§ "Two things to get right", each ~10 s, large angle). Do
   them as separate clips or one clip; single-axis is what matters. Roll is the
   flip-prone one — go slow and keep the palm facing the headset.
2. For each clip, read which robot axis the commanded orientation moved about
   (`scripts/fit_orient_remap.py`), giving the current hand-axis→robot-axis map.
3. If an axis drives the wrong robot axis → fix with `orient_remap`
   (swap the rows). If it drives the right axis but the wrong way → fix with
   `orient_flip` (negate that axis).
4. Re-validate in dry-run, then go live in Low Resolution.

`scripts/fit_orient_remap.py` automates steps 2 using the commanded `quat*`
signal (clean) with the wrist glitch-filtered the same way the operator does.
