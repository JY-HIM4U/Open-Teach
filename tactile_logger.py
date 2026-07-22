"""
Standalone tactile logger for the Inspire RH56 (Gen 4) tactile sensors.

Runs in its OWN process with its OWN Modbus connection to the hand, so the
28 ms tactile read never stalls the 60 Hz hand-control loop. It watches the
SAME recording sentinel the operators use (logs/recordings/.recording), so it
starts/stops in sync with the arm/hand CSVs (gesture- or keyboard-triggered)
and every row is timestamped for alignment.

Run it alongside teleop, in its own terminal:
  conda activate openteach && cd ~/jaeyoun/Open-Teach
  python tactile_logger.py                 # 30 Hz (default)
  python tactile_logger.py --rate 20       # slower
  python tactile_logger.py --raw           # also dump full flattened taxel arrays

Output: logs/recordings/tactile_<session>.csv
Columns: per pad <pad>_sum and <pad>_max (contact force summary); with --raw,
also <pad>_raw = space-separated flattened taxel values. Pads: each finger
top/tip/base (+ thumb mid) and palm.
"""

import argparse
import time

import numpy as np

from inspire_demos import InspireHandModbus
from openteach.utils.kinematics_recorder import SegmentRecorder

# (finger attribute, sensor position) pairs on the TactileData object.
PADS = [
    ('pinky', 'top'), ('pinky', 'tip'), ('pinky', 'base'),
    ('ring', 'top'), ('ring', 'tip'), ('ring', 'base'),
    ('middle', 'top'), ('middle', 'tip'), ('middle', 'base'),
    ('index', 'top'), ('index', 'tip'), ('index', 'base'),
    ('thumb', 'top'), ('thumb', 'tip'), ('thumb', 'mid'), ('thumb', 'base'),
]


def _pad_arrays(td):
    """Yield (pad_name, ndarray) for every tactile pad including the palm."""
    for finger, pos in PADS:
        arr = getattr(getattr(td, finger), pos, None)
        yield '%s_%s' % (finger, pos), arr
    yield 'palm', getattr(td, 'palm', None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ip', default='192.168.123.211')
    ap.add_argument('--port', type=int, default=6000)
    ap.add_argument('--rate', type=float, default=30.0, help='target Hz (tactile maxes ~35)')
    ap.add_argument('--raw', action='store_true', help='also log full flattened taxel arrays')
    args = ap.parse_args()

    fields = ['mode']
    for finger, pos in PADS:
        fields += ['%s_%s_sum' % (finger, pos), '%s_%s_max' % (finger, pos)]
    fields += ['palm_sum', 'palm_max']
    if args.raw:
        for finger, pos in PADS:
            fields.append('%s_%s_raw' % (finger, pos))
        fields.append('palm_raw')

    rec = SegmentRecorder('tactile', fields, poll_every=1)

    print('Connecting to RH56 tactile (Gen 4) at %s:%d ...' % (args.ip, args.port))
    api = InspireHandModbus(ip=args.ip, port=args.port, generation=4)
    api.connect()
    period = 1.0 / args.rate
    print('Tactile logger ready at %.0f Hz. Waiting for a recording session '
          '(start it from record_ctl.py or the resume pinch). Ctrl-C to quit.' % args.rate)

    try:
        while True:
            t0 = time.time()
            try:
                td = api.get_all_tactile_data()
            except Exception as e:
                print('[tactile] read failed: %s' % e)
                time.sleep(period)
                continue
            row = {'mode': 'real'}
            for name, arr in _pad_arrays(td):
                if arr is not None and getattr(arr, 'size', 0):
                    row[name + '_sum'] = int(arr.sum())
                    row[name + '_max'] = int(arr.max())
                    if args.raw:
                        row[name + '_raw'] = ' '.join(str(int(v)) for v in arr.flatten())
                else:
                    row[name + '_sum'] = 0
                    row[name + '_max'] = 0
            rec.maybe_log(row)
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        pass
    finally:
        rec.close()
        api.disconnect()
        print('Tactile logger stopped.')


if __name__ == '__main__':
    main()
