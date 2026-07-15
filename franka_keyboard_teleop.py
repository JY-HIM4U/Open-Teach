"""
Keyboard teleoperation for the Franka arm -- a SAFE, minimal replacement for
the Meta Quest, used to validate the low-level robot control path in isolation.

It drives the EXACT same code path as real VR teleop
(openteach.robot.franka.FrankaArm.arm_control -> FrankaController.cartesian_control
 -> robot_interface.control), but instead of VR-tracked hand poses it uses
arrow keys to nudge a target end-effector position by a fixed step.

This lets you answer "does the robot move smoothly and stably through Deoxys?"
WITHOUT trusting any VR tracking or retargeting math.

PREREQUISITES
-------------
  1. Franka connected, FCI active, joints unlocked (Desk: https://172.16.0.2/desk)
  2. The Deoxys arm daemon running in another terminal:
         conda activate deoxys
         cd ~/deoxys_control/deoxys
         ./auto_scripts/auto_arm.sh config/charmander.yml
  3. Run THIS script:
         conda activate openteach
         cd ~/jaeyoun/Open-Teach
         python franka_keyboard_teleop.py

CONTROLS
--------
  Translation (end-effector position):
    Up arrow  / w :  +X   (0.01 m)      Down arrow / s :  -X
    Left arrow / a :  +Y  (0.01 m)      Right arrow / d :  -Y
    r             :  +Z (up 0.01 m)     f              :  -Z (down)
  Rotation (end-effector orientation, about robot BASE axes, 3 deg/press):
    i / k         :  roll  +/-          (about base X)
    j / l         :  pitch +/-          (about base Y)
    u / o         :  yaw   +/-          (about base Z)
  SPACE           :  FREEZE position (target := current pose)
  h               :  toggle HOLD (pause/resume sending)
  q / Ctrl-C      :  quit (stops sending; controller times out and holds)

SAFETY
------
  * Translation is clamped to a box of +/- WORKSPACE_BOX_M around the start
    position; rotation is clamped to +/- MAX_ROT_DEG per axis around the start
    orientation. A stuck/auto-repeating key cannot run the arm away.
  * Step is small (STEP_M). Velocity is additionally clipped inside
    FrankaController.cartesian_control by the limits in franka_arm/constants.py.
  * You must type "GO" at the prompt before any command is ever sent.
  * The FIRST command sent triggers Deoxys' preprocess(): it opens the gripper
    to 8 cm and sends a short handshake. If no gripper is attached this is a
    harmless no-op. Be aware of it.

Keep a hand on the physical e-stop the entire time.
"""

# Force protobuf into pure-python mode BEFORE any deoxys/protobuf import, so
# this works even if the conda env var wasn't applied (e.g. running the env's
# python directly without `conda activate`).
import os
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

# Single-machine setup: force ROS to localhost so rospy.init_node() -- called
# deep inside FrankaArm -> DexArmControl -- reaches the local roscore, ignoring
# any stale ROS_MASTER_URI/ROS_HOSTNAME left in the shell environment. Without
# this, FrankaArm() hangs forever trying to reach a non-existent ROS master.
os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'
os.environ['ROS_HOSTNAME'] = 'localhost'
os.environ.pop('ROS_IP', None)

import sys
import time
import select
import termios
import tty
import threading

import numpy as np
import yaml
import zmq
from scipy.spatial.transform import Rotation as R

from franka_arm.constants import CONFIG_ROOT
from openteach.robot.franka import FrankaArm


def wait_for_daemon(timeout=6.0):
    """Lightweight pre-check: is the Deoxys arm daemon actually publishing
    state? Uses a throwaway raw ZMQ subscriber (binds nothing, holds no port),
    so if this fails we can exit cleanly with NO lingering socket/thread --
    unlike constructing FrankaInterface, which would leave a zombie on timeout.

    Returns True if a state message arrives within `timeout` seconds.
    """
    cfg_path = os.path.join(CONFIG_ROOT, 'deoxys.yml')
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    ip = cfg['NUC']['IP']
    state_port = cfg['NUC']['PUB_PORT']  # daemon publishes robot state here

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt_string(zmq.SUBSCRIBE, '')
    sub.connect('tcp://{}:{}'.format(ip, state_port))
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)
    got = dict(poller.poll(timeout * 1000))  # milliseconds
    ok = sub in got
    sub.close()
    ctx.term()
    return ok

# ----------------------------- tunables ------------------------------------
STEP_M = 0.01              # translation added per key press, in meters
ROT_DEG = 3.0              # rotation added per key press, in degrees (base frame)
WORKSPACE_BOX_M = 0.15     # max +/- displacement from start pose, per axis
MAX_ROT_DEG = 45.0         # max +/- rotation from start orientation, per axis
CONTROL_HZ = 60            # matches VR_FREQ; how often we send toward target
# ---------------------------------------------------------------------------


class KeyboardReader(threading.Thread):
    """Non-blocking terminal key reader (stdlib only, no pynput)."""
    def __init__(self):
        super().__init__(daemon=True)
        self._lock = threading.Lock()
        self._pressed = []
        self._running = True

    def run(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self._running:
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not r:
                    continue
                ch = sys.stdin.read(1)
                key = None
                if ch == '\x1b':  # start of an escape sequence (arrow keys)
                    seq = sys.stdin.read(2) if select.select([sys.stdin], [], [], 0.01)[0] else ''
                    key = {'[A': 'up', '[B': 'down', '[C': 'right', '[D': 'left'}.get(seq)
                else:
                    key = ch.lower()
                if key is not None:
                    with self._lock:
                        self._pressed.append(key)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def pop_all(self):
        with self._lock:
            keys, self._pressed = self._pressed, []
        return keys

    def stop(self):
        self._running = False


# Translation keys -> (axis index, sign). Axis 0=X, 1=Y, 2=Z.
KEYMAP = {
    'up': (0, +1), 'w': (0, +1),
    'down': (0, -1), 's': (0, -1),
    'left': (1, +1), 'a': (1, +1),
    'right': (1, -1), 'd': (1, -1),
    'r': (2, +1),
    'f': (2, -1),
}

# Rotation keys -> (axis index, sign). Rotations are about the robot BASE axes.
ROTMAP = {
    'i': (0, +1), 'k': (0, -1),   # roll  about base X
    'j': (1, +1), 'l': (1, -1),   # pitch about base Y
    'u': (2, +1), 'o': (2, -1),   # yaw   about base Z
}


def main():
    print(__doc__)

    print('Checking for the Deoxys arm daemon...')
    if not wait_for_daemon(timeout=6.0):
        print(
            '\n' + '=' * 70 +
            '\n ERROR: no robot state received -- the Deoxys arm daemon is NOT running'
            '\n        (or not reachable / wrong ports).'
            '\n'
            '\n Start it FIRST, in a separate terminal, and leave it running:'
            '\n     conda activate deoxys'
            '\n     cd ~/deoxys_control/deoxys'
            '\n     ./auto_scripts/auto_arm.sh config/charmander.yml'
            '\n Wait for "Waiting for control messages...", then re-run this script.'
            '\n' + '=' * 70)
        sys.exit(1)

    print('Daemon found. Connecting to Franka...')
    robot = FrankaArm()  # returns quickly now that state is confirmed flowing
    print('Connected. Robot state is streaming.')

    start = np.array(robot.get_cartesian_position(), dtype=np.float64)  # (7,) pos+quat
    start_pos = start[:3].copy()
    start_rot = R.from_quat(start[3:].copy())   # scipy quat is [x, y, z, w]
    target = start.copy()

    # Accumulated rotation offset from the start orientation, per base axis
    # (degrees). Kept explicit so we can clamp it to +/- MAX_ROT_DEG.
    rot_offset_deg = np.zeros(3)

    print('\nStart position (xyz, m): {}'.format(np.round(start_pos, 4)))
    print('Translation clamp: +/- {} m per axis around start.'.format(WORKSPACE_BOX_M))
    print('Rotation clamp:    +/- {} deg per axis around start.'.format(MAX_ROT_DEG))
    print('\nMake sure the workspace is CLEAR and the e-stop is in reach.')
    confirm = input('Type GO and press ENTER to arm keyboard control (anything else aborts): ')
    if confirm.strip() != 'GO':
        print('Aborted. No commands were sent. Robot did not move.')
        return

    reader = KeyboardReader()
    reader.start()
    hold = False
    period = 1.0 / CONTROL_HZ

    print('\nARMED. Translate: arrows/WASD, R/F. Rotate: I/K J/L U/O.')
    print('SPACE=freeze, h=hold, q=quit.\n')
    try:
        while True:
            loop_start = time.time()

            for key in reader.pop_all():
                if key in ('q', '\x03'):
                    raise KeyboardInterrupt
                elif key == ' ':
                    # freeze translation at current pose (rotation offset unchanged)
                    target[:3] = np.array(robot.get_cartesian_position()[:3])
                    print('[FREEZE] target position reset to current.')
                elif key == 'h':
                    hold = not hold
                    print('[HOLD] sending paused.' if hold else '[HOLD] resumed.')
                elif key in KEYMAP:
                    axis, sign = KEYMAP[key]
                    target[axis] += sign * STEP_M
                    # clamp to the safety box around the start position
                    target[axis] = float(np.clip(
                        target[axis], start_pos[axis] - WORKSPACE_BOX_M,
                        start_pos[axis] + WORKSPACE_BOX_M))
                elif key in ROTMAP:
                    axis, sign = ROTMAP[key]
                    rot_offset_deg[axis] = float(np.clip(
                        rot_offset_deg[axis] + sign * ROT_DEG,
                        -MAX_ROT_DEG, MAX_ROT_DEG))

            # Compose current target orientation = accumulated base-frame offset
            # applied to the start orientation, then write quat into the target.
            offset_rot = R.from_euler('xyz', rot_offset_deg, degrees=True)
            target[3:] = (offset_rot * start_rot).as_quat()

            if not hold:
                robot.arm_control(target)

            # keep a steady control rate
            dt = time.time() - loop_start
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        print('\nStopping. No more commands sent; the controller will time out '
              'and hold the last pose. Robot is idle.')


if __name__ == '__main__':
    main()
