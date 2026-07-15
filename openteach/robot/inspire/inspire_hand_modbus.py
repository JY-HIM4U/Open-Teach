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
        self._api.set_angle(cmd)
        return cmd

    def get_angles(self):
        """Actual angle of all 6 DOF (0-1000), or None on failure."""
        if self.dry_run:
            return [0] * NUM_DOF
        return self._api.getangleact()

    def set_speed(self, speeds):
        """Optional: set per-DOF speed if the library supports it (guarded)."""
        if self.dry_run:
            return
        fn = getattr(self._api, 'set_speed', None)
        if fn is not None:
            try:
                fn(speeds)
            except Exception as e:
                print('[inspire] set_speed not applied: %s' % e)

    def clear_error(self):
        if self.dry_run:
            return
        fn = getattr(self._api, 'clear_error', None) or getattr(self._api, 'clearerror', None)
        if fn is not None:
            try:
                fn()
            except Exception as e:
                print('[inspire] clear_error not applied: %s' % e)

    def open_hand(self):
        self.set_angles([1000] * NUM_DOF)

    def close_hand(self):
        # thumb rotation (DOF5) left open so it doesn't fight the fingers
        self.set_angles([0, 0, 0, 0, 0, 1000])

    def close(self):
        if self._api is not None:
            self._api.disconnect()
