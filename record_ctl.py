"""
Keyboard start/stop for teleop kinematics recording.

Run this in its OWN terminal, alongside `python teleop.py ...`. It toggles a
sentinel file that the operators poll, so recording starts/stops across BOTH
the hand and arm processes at once. Works the same in dry-run or real mode.

  conda activate openteach && cd ~/jaeyoun/Open-Teach
  python record_ctl.py

Keys:
  s / ENTER  -> start a new recording segment (new session id)
  x / SPACE  -> stop the current segment
  q          -> quit (stops recording first)

CSVs land in logs/recordings/ :
  hand_<session>.csv  - raw finger flexions + 6 RH56 angle commands
  arm_<session>.csv   - commanded arm target pose + teleop state
Same <session> label on both files; every row also has t_epoch / t_rel.
"""

import os
import sys
import time
import termios
import tty

from openteach.utils.kinematics_recorder import sentinel_path, DEFAULT_DIR


def _getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def _session_id():
    return time.strftime('%Y%m%d_%H%M%S')


def _start(sentinel):
    sid = _session_id()
    tmp = sentinel + '.tmp'
    with open(tmp, 'w') as f:
        f.write(sid)
    os.replace(tmp, sentinel)   # atomic: operators never read a half-written id
    return sid


def _stop(sentinel):
    if os.path.exists(sentinel):
        os.remove(sentinel)


def main():
    os.makedirs(DEFAULT_DIR, exist_ok=True)
    sentinel = sentinel_path()
    _stop(sentinel)   # clean slate on launch

    print(__doc__)
    print('Ready. (s/ENTER = start, x/SPACE = stop, q = quit)\n')

    recording = False
    try:
        while True:
            ch = _getch().lower()
            if ch in ('s', '\r', '\n'):
                if recording:
                    print('  already recording — stop first (x)')
                    continue
                sid = _start(sentinel)
                recording = True
                print('  ● RECORDING  session={}'.format(sid))
            elif ch in ('x', ' '):
                if not recording:
                    print('  (not recording)')
                    continue
                _stop(sentinel)
                recording = False
                print('  ■ stopped')
            elif ch in ('q', '\x03'):   # q or Ctrl-C
                _stop(sentinel)
                print('  bye')
                break
    except KeyboardInterrupt:
        _stop(sentinel)


if __name__ == '__main__':
    main()
