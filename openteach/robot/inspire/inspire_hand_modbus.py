"""
Modbus-TCP driver for the Inspire RH56 dexterous hand, wrapping the vendor
`inspire_demos` library (https://github.com/TechShare-inc/inspire_demos):

    from inspire_demos import InspireHandModbus
    api = InspireHandModbus(ip="192.168.11.210", port=6000, generation=3)
    api.connect()
    api.set_angle([500, 800, 600, 400, 200, 1000])   # 6 DOF, 0-1000
    api.getangleact()
    api.disconnect()

Install the library first:
    pip install git+https://github.com/TechShare-inc/inspire_demos.git

This class exposes the SAME interface as the RS485 driver (inspire_hand_api.py)
so the operator/retargeter don't care which transport is used.

DOF order (6): [little, ring, middle, index, thumb_bend, thumb_rot]
Angle: 1000 = OPEN, 0 = CLOSED.
"""

import numpy as np

from openteach.robot.inspire.inspire_hand_api import DOF_NAMES, NUM_DOF


class InspireHandTCP:
    def __init__(self, ip='192.168.11.210', port=6000, generation=3, dry_run=False):
        """dry_run=True never imports/connects and never transmits -- use it to
        validate retargeting with no hardware/network."""
        self.dry_run = dry_run
        self._api = None
        if not dry_run:
            try:
                from inspire_demos import InspireHandModbus
            except ImportError as e:
                raise ImportError(
                    'inspire_demos is required for Modbus-TCP control of the '
                    'Inspire hand. Install with:\n'
                    '  pip install git+https://github.com/TechShare-inc/inspire_demos.git\n'
                    '(original error: %s)' % e)
            self._api = InspireHandModbus(ip=ip, port=port, generation=generation)
            self._api.connect()

    def set_angles(self, angles):
        """Command all 6 DOF. angles: 6 ints in [0,1000] (1000=open, 0=closed)."""
        assert len(angles) == NUM_DOF, 'expected 6 angle values'
        cmd = [max(0, min(1000, int(round(a)))) for a in angles]
        if self.dry_run:
            return cmd
        # vendor inspire_demos requires a numpy int array (rejects lists)
        self._api.set_angle(np.asarray(cmd, dtype=np.int32))
        return cmd

    def get_angles(self):
        """Actual angle of all 6 DOF (0-1000), or None on failure."""
        if self.dry_run:
            return [0] * NUM_DOF
        return self._api.get_angle_actual()   # inspire_demos v0.2.0 API

    def get_force(self):
        """Actual per-DOF force reading (FORCE_ACT, 6 values), or zeros in dry-run."""
        if self.dry_run:
            return [0] * NUM_DOF
        try:
            return list(self._api.get_force_actual())
        except Exception as e:
            print('[inspire] get_force failed: %s' % e)
            return [0] * NUM_DOF

    def set_speed(self, speeds):
        """Set per-DOF motion speed (0-1000)."""
        if self.dry_run:
            return
        try:
            self._api.set_speed(np.asarray(speeds, dtype=np.int32))
        except Exception as e:
            print('[inspire] set_speed not applied: %s' % e)

    def set_force(self, forces):
        """Set per-DOF force threshold (FORCE_SET, 0-1000; lower = gentler grip).
        Values are clamped to [0,1000] so a bad config can't command out of range."""
        assert len(forces) == NUM_DOF, 'expected 6 force values'
        cmd = [max(0, min(1000, int(round(f)))) for f in forces]
        if self.dry_run:
            return cmd
        try:
            self._api.set_force(np.asarray(cmd, dtype=np.int32))
        except Exception as e:
            print('[inspire] set_force not applied: %s' % e)
        return cmd

    def clear_error(self):
        if self.dry_run:
            return
        try:
            self._api.reset_error()
        except Exception as e:
            print('[inspire] reset_error not applied: %s' % e)

    def open_hand(self):
        self.set_angles([1000] * NUM_DOF)

    def close_hand(self):
        # thumb rotation (DOF5) left open so it doesn't fight the fingers
        self.set_angles([0, 0, 0, 0, 0, 1000])

    def close(self):
        if self._api is not None:
            self._api.disconnect()
