# Franka Teleop — Staged Safe Validation

Before trusting the full VR pipeline to move the robot, validate the two
independent subsystems separately. Each stage isolates risk so a bug in one
half cannot reach the robot through the other.

## Which stages need the Meta Quest?

| Stage | What it validates | Needs Quest? | Moves the arm? |
|---|---|---|---|
| **1. Keyboard control** | The robot / Deoxys control path | **No** | **Yes** (small, clamped) |
| **2. Dry-run** | The VR app + hand tracking | Yes | No (zero motion) |
| **3. Full VR teleop** | Everything together | Yes | Yes |

**You can — and should — start with Stage 1 without the headset at all.** It
drives the Franka directly from the keyboard, so you can confirm the robot
moves smoothly before you ever involve the Quest. Stage 2 needs the headset
(that's what produces the hand data) but cannot move the arm. Do Stage 1, then
Stage 2, then Stage 3.

## Prerequisites for every stage

- Franka connected, FCI active, joints unlocked (`https://172.16.0.2/desk`).
- Deoxys arm daemon running in its own terminal:
  ```bash
  conda activate deoxys
  cd ~/deoxys_control/deoxys
  ./auto_scripts/auto_arm.sh config/charmander.yml
  ```
  Wait for `Waiting for control messages...` (connected + idle).
- **A ROS master (`roscore`) must be running.** The robot controllers call
  `rospy.init_node()`, which hangs forever without one. In its own terminal:
  ```bash
  source /opt/ros/noetic/setup.bash
  export ROS_MASTER_URI=http://localhost:11311
  export ROS_HOSTNAME=localhost
  unset ROS_IP
  roscore
  ```
  Note: this machine's shell may have a stale `ROS_MASTER_URI` pointing at an old
  IP. The entry-point scripts (`teleop.py`, `data_collect.py`,
  `franka_keyboard_teleop.py`) all force `ROS_MASTER_URI=http://localhost:11311`
  and `ROS_HOSTNAME=localhost` at startup, so you do NOT need to touch
  `~/.bashrc` — but if you run `roscore` by hand it will also bind to localhost.
- For any stage that moves the arm (1 and 3): workspace clear, hand on the
  physical e-stop.

---

## Stage 1 — Keyboard control (NO Meta Quest)

**Start here.** This controls the Franka directly from the keyboard, with no
headset involved. It uses the *exact same* control path as real VR teleop
(`FrankaArm.arm_control → FrankaController.cartesian_control → robot_interface.control`),
so it proves Deoxys moves the arm smoothly and stably — without trusting any VR
tracking or retargeting.

> ### ⚠️ THE DAEMON MUST ALREADY BE RUNNING FIRST
> The keyboard script (and dry-run, and full teleop) do **nothing** on their
> own — they connect to the Deoxys arm daemon. If the daemon is not running,
> the script hangs forever at `Connecting to Franka...`.
>
> This is a **two-terminal** workflow:
>
> **Terminal 1 — start the daemon, leave it running:**
> ```bash
> conda activate deoxys
> cd ~/deoxys_control/deoxys
> ./auto_scripts/auto_arm.sh config/charmander.yml
> ```
> Wait until you see `Waiting for control messages...`. Leave this terminal open.
>
> **Terminal 2 — then run the keyboard script:**
> ```bash
> conda activate openteach
> cd ~/jaeyoun/Open-Teach
> python franka_keyboard_teleop.py
> ```
>
> Only ONE daemon can run at a time (the robot's FCI allows a single control
> connection). If Terminal 1 is already running the daemon, do not start another.

You must type `GO` at the prompt before any command is ever sent.

**Quit with `q` or Ctrl-C — NOT Ctrl-Z.** Ctrl-Z only suspends the script and
leaves it holding the ZMQ ports; you then have to `kill %1` to clean it up.

**Controls:**
| Key | Action |
|---|---|
| Up / `w` | +X by 0.01 m |
| Down / `s` | -X by 0.01 m |
| Left / `a` | +Y by 0.01 m |
| Right / `d` | -Y by 0.01 m |
| `r` | +Z (up) 0.01 m |
| `f` | -Z (down) 0.01 m |
| SPACE | FREEZE — target reset to current pose, arm holds |
| `h` | toggle HOLD (pause/resume sending) |
| `q` / Ctrl-C | quit (controller times out and holds last pose) |

**Built-in safety rails:**
- Orientation is held fixed — keyboard only translates, never rotates.
- Target is clamped to ±0.15 m per axis around where you started, so a stuck or
  auto-repeating key cannot march the arm away.
- Step is 0.01 m; velocity is further clipped by the limits in
  `franka_arm/constants.py` (`TRANSLATION_VELOCITY_LIMIT`,
  `ROTATION_VELOCITY_LIMIT`).
- Requires a typed `GO` confirmation before arming.

**What to check:**
- Press one arrow once: the arm makes one small, smooth ~1 cm move and stops.
- Hold a key: the arm glides continuously, then stops at the clamp box edge.
- Direction matches expectation (verify the X/Y/Z sign mapping feels right on a
  tiny first move — mounting orientation can flip what "left" means).
- No jitter, no overshoot, no safety-reflex stops under normal small moves.
- SPACE and `q` both stop motion promptly.

Tune `STEP_M`, `WORKSPACE_BOX_M`, `CONTROL_HZ` at the top of
`franka_keyboard_teleop.py` if you want.

> Note: the first command sent here DOES trigger Deoxys' `preprocess()`
> (gripper opens to 8 cm + handshake). Harmless no-op if no gripper is
> attached, but be aware.

---

## Stage 2 — Dry-run: validate the APP with ZERO motion (needs Quest)

**Goal:** confirm the whole Open-Teach pipeline works end-to-end — Quest
streaming, keypoints arriving, retargeting producing sane target poses, finger
tracking alive — while the robot stays connected (its state is read for the
math) but **nothing is ever sent to it.** The arm cannot move.

This is the safest test: `arm_control()` is skipped entirely, so
`robot_interface.control()` is never called — not even the first-command
gripper-open/handshake fires.

**Run it:**
```bash
conda activate openteach
cd ~/jaeyoun/Open-Teach
python teleop.py robot=franka dry_run=True
```
Then put on the Quest, connect, and move your hand as you normally would.

**What you'll see:** a banner confirming DRY-RUN mode, then two log lines about
twice a second — one for the ARM target, one for the HAND posture:
```
[DRY-RUN][ARM]  teleop_state=1 res_scale=1.00 | current_xyz=[...] target_xyz=[...] delta_xyz(m)=[...] target_quat=[...]
[DRY-RUN][HAND] finger flexion angles (deg): thumb=[..] index=[..] middle=[..] ring=[..] pinky=[..]
```

**What to check (arm):**
- `current_xyz` matches where the arm actually is (sanity: state is being read).
- When you move your hand, `target_xyz` / `delta_xyz` change in a sensible,
  bounded way (a hand move of a few cm produces a delta of a few cm, not a
  wild jump or NaN).
- `teleop_state` flips 0↔1 as you use the Pause gesture.
- No crashes, no runaway values over several minutes.

**What to check (hand posture):**
- Angles change smoothly as you curl/extend each finger — e.g. a straight
  finger reads near ~180°, a fully curled finger drops toward ~90° or less at
  the flexed joints. This confirms the full finger-tracking stream is alive and
  sane, which is what you'll retarget onto a robotic hand later.

If both the arm deltas and the finger angles look sane and stable here, the VR
tracking + extraction half is trustworthy.

### Does the pipeline actually extract finger data? Yes — here's the detail

- The VR detector (`OculusVRHandDetector`) always streams the **full 24-point
  hand skeleton** (wrist + every finger joint) as 3D positions — see
  `OCULUS_JOINTS` in `openteach/constants.py`. This happens for *every* robot
  config, franka included.
- What the Quest gives is 3D **keypoint positions**, not joint angles directly.
  "Finger joint angles" are *computed* from three consecutive keypoints
  (`openteach.utils.vectorops.calculate_angle`) — this is exactly how the
  Allegro hand retargeter turns your hand into robot finger commands
  (`calculate_finger_angles` in `allegro_retargeters.py`). The `[DRY-RUN][HAND]`
  line runs that same computation purely for logging.
- **Important:** the arm-only `robot=franka` config runs no hand operator, so
  it never sends finger commands anywhere — the finger stream is extracted and
  (now) logged, but not retargeted to any hand. To actually drive a robotic
  hand you need a hand operator in the config (see next section).

**How it works (for reference):** `configs/teleop.yaml` sets `dry_run: false`
by default; the override `dry_run=True` flows to
`FrankaArmOperator(dry_run=...)`, which logs the computed pose and `return`s
before `self.robot.arm_control(final_pose)`. Nothing else changes.

---

## Stage 3 — Full VR teleop

Only after Stages 1 and 2 pass, run the full live VR teleop (per
`docs/franka_quest_teleop.md`):
```bash
conda activate openteach
python teleop.py robot=franka
```
with the full per-session safety checklist, starting in Low Resolution mode.

Keep a hand on the physical e-stop during every stage that can move the arm
(Stages 1 and 3). Stage 2 cannot move the arm at all.

---

## Reaching your goal: driving a robotic hand

For a real dexterous hand you'd use a config that includes a hand operator —
e.g. `robot=allegro_franka` (Allegro hand + Franka arm) or `robot=allegro`.
Those instantiate `AllegroHandOperator`, which subscribes to the same
`transformed_hand_coords` stream and retargets it to robot joint angles via
`AllegroKDLControl` / `AllegroJointControl`. That path needs the Allegro
controller (a separate, currently-uninitialized submodule) installed and the
hand hardware connected. When you're ready, the dry-run pattern here can be
extended to that operator too so you can validate hand retargeting with zero
motion first.
