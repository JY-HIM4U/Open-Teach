"""
Retarget human hand keypoints (from the Quest, via OpenTeach) to Inspire RH56
6-DOF finger angle commands (0-1000, where 1000=open, 0=closed).

The Quest gives 3D keypoint POSITIONS, not joint angles. We derive a per-finger
"flexion" scalar from three-consecutive-keypoint angles (the same method the
Allegro retargeter uses, openteach.utils.vectorops.calculate_angle), then map
each finger's flexion range onto the RH56's 0-1000 command range.

*** THIS NEEDS CALIBRATION PER USER. *** The default flex_open/flex_closed
values below are rough. Use inspire_hand_test.py (calibration mode) to record
YOUR open-hand and closed-hand flexion values and paste them into the config.
See docs/inspire_franka_teleop.md.
"""

import numpy as np
from openteach.constants import OCULUS_JOINTS
from openteach.utils.vectorops import calculate_angle

# RH56 DOF order: [little, ring, middle, index, thumb_bend, thumb_rot]
# The 4 simple fingers map from these OpenTeach finger names:
FINGER_TO_DOF = {'pinky': 0, 'ring': 1, 'middle': 2, 'index': 3}

# Default per-DOF calibration: (flex_open, flex_closed) in the same "summed
# flexion angle (deg)" units this module computes. flex_open -> command 1000
# (open), flex_closed -> command 0 (closed). ROUGH DEFAULTS - CALIBRATE.
DEFAULT_CALIB = {
    0: (25.0, 150.0),   # little
    1: (25.0, 150.0),   # ring
    2: (25.0, 150.0),   # middle
    3: (25.0, 150.0),   # index
    4: (30.0, 120.0),   # thumb bending
    5: (20.0, 90.0),    # thumb rotation (abduction proxy) - especially rough
}


def _finger_flexion(hand_coords, finger):
    """Summed bend (deg) over a finger's joints. ~0 = straight, large = curled."""
    chain = [0] + list(OCULUS_JOINTS[finger])  # wrist + this finger's joints
    pts = hand_coords[chain]
    total = 0.0
    for i in range(len(pts) - 2):
        total += np.degrees(calculate_angle(pts[i], pts[i + 1], pts[i + 2]))
    return total


def _thumb_rotation_proxy(hand_coords):
    """Rough thumb opposition/rotation proxy: angle between the thumb metacarpal
    direction and the index-knuckle direction (both relative to the wrist).
    Increases as the thumb swings across the palm. NEEDS CALIBRATION."""
    wrist = hand_coords[0]
    thumb_mcp = hand_coords[OCULUS_JOINTS['thumb'][0]]
    index_knuckle = hand_coords[OCULUS_JOINTS['index'][0]]
    return np.degrees(calculate_angle(index_knuckle, wrist, thumb_mcp))


def _map(flex, open_val, closed_val):
    """flex_open -> 1000 (open), flex_closed -> 0 (closed), clamped."""
    span = (closed_val - open_val)
    if abs(span) < 1e-6:
        return 1000
    frac = (flex - open_val) / span      # 0 at open, 1 at closed
    frac = min(1.0, max(0.0, frac))
    return int(round(1000 * (1.0 - frac)))


class InspireRetargeter:
    def __init__(self, calibration=None, thumb_rot_fixed=None):
        # calibration: dict {dof_index: [open_val, closed_val]}
        self.calib = dict(DEFAULT_CALIB)
        if calibration:
            for k, v in calibration.items():
                self.calib[int(k)] = (float(v[0]), float(v[1]))
        # DOF5 (thumb rotation) from the Quest is noisy and entangled with the
        # thumb bend, which wrecks pinching. If thumb_rot_fixed is set, we LOCK
        # DOF5 to that constant (a good opposition angle for pinching) instead
        # of retargeting it. None -> retarget as before.
        self.thumb_rot_fixed = (None if thumb_rot_fixed is None
                                else max(0, min(1000, int(thumb_rot_fixed))))

    def raw_flexions(self, hand_coords):
        """Return the raw flexion metrics per DOF (useful for calibration)."""
        f = {name: _finger_flexion(hand_coords, name)
             for name in ['pinky', 'ring', 'middle', 'index', 'thumb']}
        return {
            0: f['pinky'], 1: f['ring'], 2: f['middle'], 3: f['index'],
            4: f['thumb'], 5: _thumb_rotation_proxy(hand_coords),
        }

    def retarget(self, hand_coords):
        """hand_coords: (24,3) transformed hand keypoints -> [6] angle commands."""
        flex = self.raw_flexions(hand_coords)
        cmd = [_map(flex[d], *self.calib[d]) for d in range(6)]
        if self.thumb_rot_fixed is not None:
            cmd[5] = self.thumb_rot_fixed   # lock thumb rotation for stable pinch
        return cmd
