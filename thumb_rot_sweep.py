"""
Find the best FIXED thumb-rotation (DOF5) value for pinching on the RH56.

Thumb rotation from the Quest is too noisy to teleoperate, so we lock it at a
constant opposition angle and pinch with thumb-bend + fingers. This tool holds
a pinch-test pose and lets you sweep DOF5 live to see where the thumb tip lines
up with the index/middle fingertips. When it pinches nicely, note the value and
put it in configs/robot/inspire_franka.yaml as `thumb_rot_fixed`.

  conda activate openteach && cd ~/jaeyoun/Open-Teach
  python thumb_rot_sweep.py

Keys:
  k / j     thumb rotation +25 / -25   (0=one extreme, 1000=other)
  K / J     thumb rotation +100 / -100
  c         CLOSE the pinch (curl index + thumb bend to test if tips meet)
  o         OPEN the pinch (uncurl) - thumb rotation stays where you set it
  q         quit and print the chosen thumb_rot value

SAFETY: this moves the hand (force-capped). Keep fingers/objects clear.
"""

import sys
import termios
import tty

import numpy as np
from inspire_demos import InspireHandModbus

IP, PORT = '192.168.123.211', 6000
# pinch-test pose: [little, ring, middle, index, thumb_bend, thumb_rot]
OPEN_POSE = [1000, 1000, 1000, 1000, 1000]      # fingers (DOF0-4) open
PINCH_POSE = [1000, 1000, 600, 350, 350]        # index+thumb curl to meet, others out


def _getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main():
    api = InspireHandModbus(ip=IP, port=PORT, generation=4)
    api.connect()
    api.set_force(np.array([400] * 6, dtype=np.int32))
    api.set_speed(np.array([600] * 6, dtype=np.int32))

    rot = 500        # current thumb rotation
    closed = False   # pinch closed?
    print(__doc__)

    def send():
        base = list(PINCH_POSE if closed else OPEN_POSE)
        api.set_angle(np.array(base + [rot], dtype=np.int32))
        print('  thumb_rot=%4d   pinch=%s' % (rot, 'CLOSED' if closed else 'open'))

    send()
    try:
        while True:
            ch = _getch()
            if ch == 'k':
                rot = min(1000, rot + 25); send()
            elif ch == 'j':
                rot = max(0, rot - 25); send()
            elif ch == 'K':
                rot = min(1000, rot + 100); send()
            elif ch == 'J':
                rot = max(0, rot - 100); send()
            elif ch == 'c':
                closed = True; send()
            elif ch == 'o':
                closed = False; send()
            elif ch in ('q', '\x03'):
                break
    finally:
        api.set_angle(np.array(OPEN_POSE + [rot], dtype=np.int32))
        api.disconnect()
        print('\n==> Chosen thumb rotation: %d' % rot)
        print('    Put this in configs/robot/inspire_franka.yaml:')
        print('        thumb_rot_fixed: %d' % rot)


if __name__ == '__main__':
    main()
