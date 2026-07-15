# Franka Arm + Inspire RH56 Hand — Teleoperation

Teleoperate the **Franka arm** (end-effector pose) and the **Inspire RH56
dexterous hand** (6 finger DOF) together from one Meta Quest hand:

- Your **wrist position/orientation** → Franka end-effector (existing arm operator).
- Your **finger curl** → RH56 finger angles (new hand operator).

Config: `robot=inspire_franka`. It runs the `InspireHandOperator` and
`FrankaArmOperator` side by side off the same Quest keypoint stream.

> Read `docs/franka_validation.md` first — the arm safety envelope (filter,
> ±0.2 m clamp, glitch guard, dry-run) all still applies here.

---

## What was built

| File | Purpose |
|---|---|
| `openteach/robot/inspire/inspire_hand_modbus.py` | **Modbus-TCP driver** (wraps the `inspire_demos` library) — the one used |
| `openteach/robot/inspire/inspire_hand_api.py` | Alternative RS485 driver (0xEB0x90 protocol, verified against the manual) |
| `openteach/robot/inspire/inspire_retargeter.py` | Human keypoints → 6-DOF angle commands (calibratable) |
| `openteach/components/operators/inspire_hand.py` | The hand operator (dry-run aware) |
| `configs/robot/inspire_franka.yaml` | Combined arm+hand config |
| `inspire_hand_test.py` | Standalone bring-up + calibration (no arm/VR) |

## Hardware setup (Modbus TCP)

1. **Power**: RH56 runs on **24 V**. Provide it before anything else.
2. **Library**: install the vendor Modbus client —
   ```bash
   conda activate openteach
   pip install git+https://github.com/TechShare-inc/inspire_demos.git
   ```
3. **Network**: the hand is a **Modbus-TCP server** (default `192.168.11.210:6000`).
   This workstation must have an interface **on the `192.168.11.x` subnet** to
   reach it — that's a THIRD network alongside the Franka FCI link (`172.16.0.x`,
   wired) and the Quest WiFi. Options: a dedicated Ethernet port/USB-Ethernet
   adapter on `192.168.11.x`, or route to it. Verify with:
   ```bash
   ping 192.168.11.210
   ```
   The hand's IP/port live in **`configs/network.yaml`** (`inspire_hand_ip`,
   `inspire_hand_port`) — change them there in one place. You can also override
   per-run: `python teleop.py robot=inspire_franka inspire_hand_ip=10.0.0.5`.
4. **Mounting**: attach the RH56 to the Franka flange. **The mount pose matters
   for the mapping** — see "Orientation & calibration" below.

Usage in code (the vendor API this driver wraps):
```python
from inspire_demos import InspireHandModbus
api = InspireHandModbus(ip="192.168.11.210", port=6000, generation=3)
api.connect(); api.set_angle([500,800,600,400,200,1000]); api.getangleact(); api.disconnect()
```

## RH56 DOF order and angle convention (from the manual)

Register `ANGLE_SET` (0x05CE), 6 shorts, per-DOF:

| DOF | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| finger | little | ring | middle | index | thumb bend | thumb rotation |

Angle value **1000 = fully OPEN**, **0 = fully CLOSED**, **-1 = leave unchanged**.

---

## Staged bring-up (do these in order — same safety philosophy as the arm)

Everything up to Stage H3 keeps the arm out of it entirely.

### H0 — Logic check (no hardware, no network, no motion)
```bash
conda activate openteach && cd ~/jaeyoun/Open-Teach
python inspire_hand_test.py --dry
```
Confirms the command clamping/interface without touching anything.

### H1 — Hand hardware check (moves the HAND only; no arm, no VR)
```bash
ping 192.168.11.210        # confirm the hand is reachable first
python inspire_hand_test.py --ip 192.168.11.210 --port 6000 --demo
```
Opens/closes the hand on ENTER prompts. Confirms the Modbus-TCP link + power are
working end-to-end. **Keep fingers/objects clear** — low force, but it can pinch.

### H2 — Calibrate the finger ranges (needs the Quest streaming)
The default retargeting ranges are rough and per-user. Record YOUR open/closed
flexion (Quest app streaming; `teleop.py` NOT needed):
```bash
python inspire_hand_test.py --calibrate
```
Follow the prompts (hold hand OPEN, then CLOSED). It prints a `calibration:`
block — paste it under the Inspire operator in `configs/robot/inspire_franka.yaml`.

### H3 — Full pipeline DRY-RUN (arm + hand, ZERO motion)
```bash
python teleop.py robot=inspire_franka dry_run=True
```
Both operators run but send nothing. You'll see, ~2 Hz:
```
[DRY-RUN][INSPIRE] angle cmd (1000=open,0=closed): little=.. ring=.. middle=.. index=.. thumb_bend=.. thumb_rot=..
[DRY-RUN][ARM]     ... target_xyz=... target_quat=...
```
Curl each finger and confirm its `angle cmd` drops toward 0; open it and confirm
it rises toward 1000. Fix calibration until it feels right — **still no motion.**

### H4 — Real teleoperation (arm AND hand move)
Prereqs: `roscore`, Deoxys arm daemon, hand powered on `/dev/ttyUSB0`, Quest
re-streamed. Then:
```bash
python teleop.py robot=inspire_franka
```
In the headset: middle-pinch → Arm+Hand mode. Move your hand to drive the arm,
curl your fingers to drive the RH56. Start in **Low Resolution**, hand on the
e-stop. Ring-pinch pauses the ARM (the hand keeps following your fingers).

---

## Orientation & calibration — READ THIS before trusting the mapping

Mounting a hand on the flange couples several frames. These are the things to
get right (you said you'd tune them together later — here's the checklist):

1. **Human hand ↔ RH56 handedness.** The RH56 is a specific left/right hand.
   Teleoperate with the **matching** human hand, or the finger→DOF mapping is
   mirrored (your index drives the robot's "index" only if handedness matches).
   The retargeter maps: pinky→DOF0(little), ring→1, middle→2, index→3,
   thumb→4(bend), thumb-abduction→5(rotation).

2. **Thumb rotation (DOF5) is the roughest.** It uses an approximate abduction
   proxy and almost certainly needs hand-tuned `calibration[5]`, possibly a
   sign/offset change in `inspire_retargeter._thumb_rotation_proxy`. Validate it
   carefully in H3 before H4.

3. **Finger ranges are per-user** — always run H2 calibration. Fingers that
   over/under-close are just wrong `[open, closed]` values.

4. **Arm mount offset (`H_A_R` in `franka.py`).** The arm retargeting has a
   fixed `[0,0,-0.06]` mount offset (originally the 6 cm Allegro mount) baked
   into `H_A_R`. With the RH56 attached, the **control point moves** (hand is
   longer/heavier, offset different). Rotating your wrist will swing the
   commanded arm position by roughly this offset — set it to the RH56's actual
   flange-to-control-point distance, or move to the decoupled position mapping
   we discussed (position from wrist world-displacement, orientation separate).

5. **Arm axis mapping (`flip_vertical`, and ultimately `H_A_R`/`R_cal`).** The
   Unity↔robot handedness still applies. If up/down or left/right feel wrong,
   that's the arm calibration from `docs/franka_validation.md`, unaffected by the
   hand — calibrate it with the 3-direction dry-run test.

6. **Physical mount orientation.** However you bolt the RH56 to the flange
   (rotation about the flange axis) shifts where "palm forward" points. Keep it
   consistent, and fold any fixed rotation into `H_A_R` once, rather than
   fighting it live.

## Safety notes specific to the hand

- **Force**: RH56 fingers are low-force (≤15 N thumb, ≤10 N fingers) but can
  pinch. The manual's per-DOF `CURRENT_LIMIT`/`FORCE_SET` protect the actuators
  — leave defaults unless you know why you're changing them.
- **First command**: unlike the Franka gripper, the RH56 has no auto-reset
  handshake — it moves only when you send `ANGLE_SET`. In dry-run it never does.
- **Errors**: on a locked-rotor/over-current the finger stops (STATUS/ERROR
  registers). `inspire_hand.clear_error()` clears clearable faults.
- **Independent of the arm**: the hand operator is its own process. If it
  crashes, the arm keeps running and vice-versa — check both terminals' output.
