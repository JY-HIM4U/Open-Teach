"""
Operator that teleoperates the Inspire RH56 dexterous hand from the Quest hand
keypoints. Subscribes to the same 'transformed_hand_coords' stream the Allegro
operator uses, retargets to 6-DOF angle commands, and drives the RH56 over
RS485. Runs alongside FrankaArmOperator (see configs/robot/inspire_franka.yaml).

dry_run=True: run the full pipeline (subscribe, retarget, log) but send NOTHING
to the hand -- the safe way to validate finger retargeting before it moves.
"""

import zmq
import numpy as np

from openteach.constants import VR_FREQ, OCULUS_NUM_KEYPOINTS
from openteach.utils.timer import FrequencyTimer
from openteach.utils.network import ZMQKeypointSubscriber
from openteach.robot.inspire.inspire_hand_api import DOF_NAMES
from openteach.robot.inspire.inspire_hand_modbus import InspireHandTCP
from openteach.robot.inspire.inspire_retargeter import InspireRetargeter
from .operator import Operator


class InspireHandOperator(Operator):
    def __init__(
        self,
        host,
        transformed_keypoints_port,
        ip='192.168.11.210',
        port=6000,
        generation=3,
        speed=1000,
        calibration=None,
        dry_run=False,
    ):
        self.notify_component_start('inspire hand operator')
        self._host, self._port = host, transformed_keypoints_port

        # Full finger-keypoint stream (24 x 3), same topic the Allegro op uses.
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host, port=transformed_keypoints_port, topic='transformed_hand_coords')
        # Arm-frame subscriber (unused here, required by the Operator interface).
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host, port=transformed_keypoints_port, topic='transformed_hand_frame')

        self.retargeter = InspireRetargeter(calibration=calibration)
        self.dry_run = dry_run
        self._log_counter = 0

        # Inspire RH56 over Modbus TCP (inspire_demos library).
        self._hand = InspireHandTCP(
            ip=ip, port=port, generation=generation, dry_run=dry_run)
        if not dry_run and speed is not None:
            self._hand.set_speed([speed] * 6)

        self._timer = FrequencyTimer(VR_FREQ)

        if self.dry_run:
            print('\n' + '=' * 70)
            print(' INSPIRE HAND OPERATOR RUNNING IN DRY-RUN MODE')
            print(' Retargeting is live but NO commands are sent to the hand.')
            print(' The hand will NOT move. Logging computed angle commands only.')
            print('=' * 70 + '\n')

    # ---- Operator interface ----
    @property
    def timer(self):
        return self._timer

    @property
    def robot(self):
        return self._hand

    @property
    def transformed_hand_keypoint_subscriber(self):
        return self._transformed_hand_keypoint_subscriber

    @property
    def transformed_arm_keypoint_subscriber(self):
        return self._transformed_arm_keypoint_subscriber

    def return_real(self):
        return False

    def _get_hand_keypoints(self):
        data = None
        for _ in range(10):
            data = self._transformed_hand_keypoint_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
            if data is not None:
                break
        if data is None:
            return None
        return np.asanyarray(data).reshape(OCULUS_NUM_KEYPOINTS, 3)

    def _apply_retargeted_angles(self):
        hand_coords = self._get_hand_keypoints()
        if hand_coords is None:
            return

        angles = self.retargeter.retarget(hand_coords)  # [6] in 0..1000

        if self.dry_run:
            self._log_counter += 1
            if self._log_counter % 30 == 0:  # ~2 Hz at VR_FREQ=60
                pretty = ' '.join('{}={}'.format(n, a) for n, a in zip(DOF_NAMES, angles))
                print('[DRY-RUN][INSPIRE] angle cmd (1000=open,0=closed): ' + pretty)
            return

        self._hand.set_angles(angles)

    # Own stream loop (does not need robot joint state to run).
    def stream(self):
        self.notify_component_start('inspire hand control')
        print('Start controlling the Inspire RH56 hand with the Oculus.\n')
        try:
            while True:
                self.timer.start_loop()
                self._apply_retargeted_angles()
                self.timer.end_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self._transformed_hand_keypoint_subscriber.stop()
            if not self.dry_run:
                self._hand.close()
            print('Stopping the Inspire hand operator.')
