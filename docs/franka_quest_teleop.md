# Franka + Meta Quest Teleoperation — Run Guide (this workstation)

This doc is specific to `realm-ThinkStation` (this machine acts as both the Deoxys
NUC *and* the Open-Teach server — everything runs on one box). It covers: which
file to edit when the network changes, which environment variables matter, and
the exact commands to run, in order, to teleoperate the Franka arm with a Meta
Quest.

For general Open-Teach usage/config options see the other files in `docs/`.
This file only documents what's true *on this machine*.

> **Before your first live run, validate the subsystems separately** — see
> `docs/franka_validation.md`. You can start controlling the Franka from the
> keyboard with **no Quest at all** (Stage 1), then dry-run the VR app with
> zero motion (Stage 2), before trusting the full pipeline (Stage 3).

---

## 1. Network topology — read this first

There are **two separate networks** on this machine. Only one of them ever
needs to change:

| Link | Interface | IP | Changes? |
|---|---|---|---|
| Franka FCI link (wired, robot control box) | `enp1s0` | `172.16.0.3` (robot is `172.16.0.2`) | **No** — fixed, do not touch |
| LAN the Meta Quest joins (WiFi) | `wlxa09f10bf31db` | DHCP, changes per network | **Yes** — this is what you update |

The FCI IPs are baked into `deoxys_control/deoxys/config/charmander.yml` and
`OpenTeach-Controllers/src/franka-arm-controllers/franka_arm/configs/deoxys.yml`.
**Do not edit these when you switch WiFi networks** — they only need to change
if the robot's own network config changes or you move to a different NUC.

## 2. The one file you edit every time you switch WiFi networks

`Open-Teach/configs/network.yaml` → `host_address`

Steps:
1. Connect this workstation and the Meta Quest to the **same** WiFi network.
2. Find this machine's IP on that network:
   ```bash
   ip -4 addr show wlxa09f10bf31db | grep inet
   ```
3. Set `host_address` in `configs/network.yaml` to that IP (currently a stale
   placeholder, `172.24.71.206` — replace it).
4. On the Quest, open the VR app → Menu → **Change IP** → enter the same IP →
   **Stream**.

**Watch out for AP/client isolation.** Some WiFi networks (especially
enterprise/campus ones) block device-to-device traffic even though both
devices can reach the internet. If the Quest app connects but nothing streams,
this is the first thing to suspect — test with a personal router/hotspot
instead if in doubt.

## 3. Environments

Two conda envs are used, for different pieces of the pipeline:

| Env | Python | Used for |
|---|---|---|
| `deoxys` | 3.11 | Low-level robot daemons (`auto_arm.sh`, `auto_gripper.sh`) and the ROS gravity-comp node |
| `openteach` | 3.10 | `teleop.py` / `data_collect.py` (the actual Open-Teach pipeline) |

Both envs already have `deoxys`, `franka_arm`, and their dependencies
editable-installed. `openteach` additionally has the `open-teach` package
itself installed editable.

### Environment variable that must be set

```
PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
```

This is required because the protobuf files for Deoxys were generated with an
older `protoc` (3.13) than the `protobuf` Python runtime installed (7.x) — newer
runtimes reject old-style generated code unless forced into pure-Python mode.

**This is already persisted** on both envs via
`conda env config vars set PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python -n <env>`,
so it's applied automatically on `conda activate` — you don't need to export it
by hand. (Verify with `conda env config vars list -n deoxys`.)

## 4. Execution order

Run each of these in its own terminal, in this order. **Nothing here moves the
robot by itself** — motion only begins once you're in `teleop.py` and moving
your hand while in an active (non-Paused) teleop state.

### Terminal 0 — ROS master (required; the controllers use `rospy`)
```bash
source /opt/ros/noetic/setup.bash
roscore
```
Leave it running. Without it, `teleop.py` hangs in `rospy.init_node()`. The
entry-point scripts force `ROS_MASTER_URI`/`ROS_HOSTNAME` to localhost, so a
stale shell setting doesn't matter and you don't need to edit `~/.bashrc`.

### Terminal 1 — arm control daemon (real-time, talks to the robot)
```bash
conda activate deoxys
cd ~/deoxys_control/deoxys
./auto_scripts/auto_arm.sh config/charmander.yml
```
Wait for `Deoxys starting` / `Waiting for control messages...` — that means
it's connected and idle. Leave this running.

### Terminal 2 — gripper daemon (only if a gripper/hand is attached)
```bash
conda activate deoxys
cd ~/deoxys_control/deoxys
./auto_scripts/auto_gripper.sh config/charmander.yml
```

### Terminal 3 — gravity-comp ROS node (optional, only needed for a mounted hand)
```bash
conda activate deoxys
roslaunch franka_arm franka_arm.launch
```
*(Not yet set up on this machine — `franka_arm` isn't on a catkin workspace
path. Skip this for a bare-arm test; ask to have it set up if you need it.)*

### Terminal 4 — Open-Teach teleop
```bash
conda activate openteach
cd ~/jaeyoun/Open-Teach
python teleop.py robot=franka
```
Then put on the Quest headset and click **Stream**.

### Alternative: data collection instead of live teleop only
```bash
conda activate openteach
cd ~/jaeyoun/Open-Teach
python data_collect.py robot=franka demo_num=1
```

## 5. First `.control()` call — know what happens

The first time `teleop.py` actually issues a control command (not just
connects), `FrankaInterface.control()` runs `preprocess()`, which:
- **Automatically opens the gripper to 8cm** (`automatic_gripper_reset=True`
  default) — if no gripper is attached this should be a harmless no-op, but
  confirm this on your hardware before your first real run.
- Sends a short handshake sequence of `NO_CONTROL` dummy messages.

This happens *before* your hand motion starts driving the arm — don't be
surprised by it.

## 6. Safety checklist (do this every session, not just the first)

- [ ] Know where the physical hardware e-stop is and can reach it.
- [ ] Workspace clear of people/objects within the arm's full reach.
- [ ] Franka Desk (`https://172.16.0.2/desk`) collision-behavior thresholds are
      at sane defaults, not disabled.
- [ ] Start in **Low Resolution** mode (pinky pinch in the VR app) for the
      first movements each session.
- [ ] Confirm the **Pause** gesture (ring finger pinch) actually stops the arm
      before doing any real range-of-motion movement.
- [ ] Move slowly/small first, verify the mapping direction feels right.

Pinch-gesture reference (single arm + hand): see `docs/vr.md`.

## 7. Shutting down

Ctrl+C `teleop.py` first, then the ROS node (if running), then the
gripper/arm daemons last. To force-stop a stuck daemon:
```bash
pkill -9 -f "auto_scripts/auto_arm.sh"
pkill -9 -f "bin/franka-interface"
pkill -9 -f "auto_scripts/auto_gripper.sh"
pkill -9 -f "bin/gripper-interface"
```

## 8. Verifying the robot connection without any motion risk

Useful any time you want to sanity-check the FCI link is alive without
touching `teleop.py`:
```bash
conda activate deoxys
cd ~/deoxys_control/deoxys
python deoxys/scripts/print_robot_state.py --interface-cfg charmander.yml
```
This only reads and prints joint state / end-effector pose, then exits — it
never calls `.control()`, so it cannot move the arm.

---

## Appendix: known issues already fixed on this machine

If you ever re-clone these repos or set up a new machine, you'll likely hit
these — see `OpenTeach-Controllers/src/franka-arm-controllers/SETUP_NOTES.md`
for the full list and fixes (missing `__init__.py`, hardcoded paths, missing
Python protobuf bindings, `libyaml-cpp` linking, IP topology, etc).
