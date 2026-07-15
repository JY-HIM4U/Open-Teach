"""
Standalone bring-up + calibration tool for the Inspire RH56 dexterous hand.
NO Franka, NO Quest, NO ROS -- just the hand over RS485. Use it to:

  1. Confirm the serial link and protocol work (open/close the hand).
  2. Record YOUR open-hand / closed-hand flexion values for the retargeter
     (calibration mode) -- needs the Quest streaming, teleop.py NOT required.

USAGE
-----
  conda activate openteach
  cd ~/jaeyoun/Open-Teach

  # 1) Hardware check (moves the hand!). Uses Modbus TCP (inspire_demos).
  python inspire_hand_test.py --ip 192.168.11.210 --port 6000 --demo

  # 1b) Logic check with NO hardware / NO motion / NO network
  python inspire_hand_test.py --dry

  # 2) Calibration: needs the Quest app streaming to this machine
  #    (set host_address in configs/network.yaml, Stream in the headset).
  #    Follow the prompts: hold hand OPEN, then CLOSED.
  python inspire_hand_test.py --calibrate

Requires the vendor library:
  pip install git+https://github.com/TechShare-inc/inspire_demos.git

SAFETY: --demo and the calibration "send" path physically move the hand. Keep
fingers/objects clear. The hand is low-force but can pinch.
"""

import argparse
import time

import yaml

from openteach.robot.inspire.inspire_hand_api import DOF_NAMES
from openteach.robot.inspire.inspire_hand_modbus import InspireHandTCP


def _load_network():
    """Read the shared network config (single source of truth)."""
    with open('configs/network.yaml') as f:
        return yaml.safe_load(f)


def demo(ip, port, generation):
    print('Connecting to Inspire hand at %s:%d (Modbus TCP) ...' % (ip, port))
    hand = InspireHandTCP(ip=ip, port=port, generation=generation)
    try:
        hand.set_speed([600] * 6)
        print('Actual angles now:', hand.get_angles())
        input('Press ENTER to OPEN the hand (it will move)...')
        hand.open_hand(); time.sleep(1.5)
        print('Actual angles:', hand.get_angles())
        input('Press ENTER to CLOSE the hand...')
        hand.close_hand(); time.sleep(1.5)
        print('Actual angles:', hand.get_angles())
        input('Press ENTER to OPEN again and finish...')
        hand.open_hand(); time.sleep(1.5)
    finally:
        hand.close()
    print('Done.')


def dry():
    hand = InspireHandTCP(dry_run=True)
    print('set_angles open  [1000]*6 -> clamped', hand.set_angles([1000] * 6))
    print('set_angles close          -> clamped', hand.set_angles([0, 0, 0, 0, 0, 1000]))
    print('get_angles (dry)          ->', hand.get_angles())
    print('(no hardware/network touched)')


def calibrate(host, port_num):
    """Record open/closed flexion metrics from the live Quest stream."""
    import numpy as np, zmq
    from openteach.utils.network import ZMQKeypointSubscriber
    from openteach.constants import OCULUS_NUM_KEYPOINTS
    from openteach.robot.inspire.inspire_retargeter import InspireRetargeter

    sub = ZMQKeypointSubscriber(host=host, port=port_num, topic='transformed_hand_coords')
    rt = InspireRetargeter()

    def sample(n=30):
        vals = []
        while len(vals) < n:
            d = sub.recv_keypoints()
            if d is None:
                continue
            hc = np.asanyarray(d).reshape(OCULUS_NUM_KEYPOINTS, 3)
            vals.append(rt.raw_flexions(hc))
        return {k: float(np.median([v[k] for v in vals])) for k in vals[0]}

    print('\nMake sure the Quest app is streaming (border green/blue).')
    input('Hold your hand fully OPEN (fingers straight), then press ENTER...')
    open_v = sample()
    input('Now CLENCH your hand fully CLOSED, then press ENTER...')
    closed_v = sample()

    print('\n# Paste this under the Inspire operator\'s `calibration:` in')
    print('# configs/robot/inspire_franka.yaml (dof: [open, closed]):')
    print('    calibration:')
    for d in range(6):
        print('      %d: [%.1f, %.1f]   # %s' % (d, open_v[d], closed_v[d], DOF_NAMES[d]))
    sub.stop()


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--ip', default=None, help='Inspire hand IP (default: inspire_hand_ip in network.yaml)')
    p.add_argument('--port', type=int, default=None, help='Inspire hand port (default: inspire_hand_port in network.yaml)')
    p.add_argument('--generation', type=int, default=3)
    p.add_argument('--demo', action='store_true', help='move the hand open/close')
    p.add_argument('--dry', action='store_true', help='logic check, no hardware/network')
    p.add_argument('--calibrate', action='store_true', help='record open/closed flexions')
    p.add_argument('--host', default=None, help='host_address for --calibrate (default: read network.yaml)')
    args = p.parse_args()

    if args.dry:
        dry()
    elif args.demo:
        net = _load_network()
        ip = args.ip or net['inspire_hand_ip']
        port = args.port or net['inspire_hand_port']
        demo(ip, port, args.generation)
    elif args.calibrate:
        net = _load_network()
        host = args.host or net['host_address']
        port_num = net['transformed_position_keypoint_port']
        calibrate(host, port_num)
    else:
        print('Nothing to do. Use --demo, --dry, or --calibrate. See --help.')
