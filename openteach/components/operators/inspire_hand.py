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

import time

from openteach.constants import (VR_FREQ, OCULUS_NUM_KEYPOINTS,
                                 ARM_TELEOP_CONT, ARM_TELEOP_STOP)
from openteach.utils.timer import FrequencyTimer
from openteach.utils.network import ZMQKeypointSubscriber
from openteach.robot.inspire.inspire_hand_api import DOF_NAMES
from openteach.robot.inspire.inspire_hand_modbus import InspireHandTCP
from openteach.robot.inspire.inspire_retargeter import InspireRetargeter
from openteach.utils import kinematics_recorder as krec
from openteach.utils.kinematics_recorder import SegmentRecorder
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
        force=500,
        calibration=None,
        thumb_rot_fixed=None,
        dry_run=False,
        teleoperation_reset_port=None,
        respect_teleop_pause=True,
        record_on_teleop=True,
    ):
        self.notify_component_start('inspire hand operator')
        self._host, self._port = host, transformed_keypoints_port

        # Full finger-keypoint stream (24 x 3), same topic the Allegro op uses.
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host, port=transformed_keypoints_port, topic='transformed_hand_coords')
        # Arm-frame subscriber (unused here, required by the Operator interface).
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host, port=transformed_keypoints_port, topic='transformed_hand_frame')

        # Same pause/resume signal the Franka operator uses (published by the
        # Quest detector): index+thumb pinch -> STOP, middle+thumb -> CONT.
        # We reuse it so one gesture pauses BOTH arm and hand.
        self.respect_teleop_pause = respect_teleop_pause
        self.record_on_teleop = record_on_teleop
        self._teleop_state_subscriber = None
        if teleoperation_reset_port is not None:
            self._teleop_state_subscriber = ZMQKeypointSubscriber(
                host=host, port=teleoperation_reset_port, topic='pause')
        # Start paused (like the arm) so the hand only follows after a resume
        # pinch; cached because we read the signal non-blocking.
        self._teleop_state = ARM_TELEOP_STOP
        self._recording = False
        self._seg_count = 0

        self.retargeter = InspireRetargeter(
            calibration=calibration, thumb_rot_fixed=thumb_rot_fixed)
        self.dry_run = dry_run
        self._log_counter = 0

        # Kinematics recorder (toggled from record_ctl.py). Logs the extracted
        # per-DOF flexions and the 6 RH56 angle commands, dry-run or real.
        self._recorder = SegmentRecorder(
            'hand',
            ['mode', 'flex_little', 'flex_ring', 'flex_middle', 'flex_index',
             'flex_thumb_bend', 'flex_thumb_rot',
             'cmd_little', 'cmd_ring', 'cmd_middle', 'cmd_index',
             'cmd_thumb_bend', 'cmd_thumb_rot',
             # ACTUAL finger joint angles read back from the RH56 (0-1000)
             'act_little', 'act_ring', 'act_middle', 'act_index',
             'act_thumb_bend', 'act_thumb_rot',
             # per-DOF force reading (FORCE_ACT)
             'frc_little', 'frc_ring', 'frc_middle', 'frc_index',
             'frc_thumb_bend', 'frc_thumb_rot',
             # raw thumb-related keypoints (to design a working thumb-rotation
             # proxy; the current one is stuck). kp4=thumb tip, kp2=thumb mcp,
             # kp6=index knuckle, kp15=pinky knuckle, kp0=wrist.
             'thumbtip_x', 'thumbtip_y', 'thumbtip_z',
             'thumbmcp_x', 'thumbmcp_y', 'thumbmcp_z',
             'idxknk_x', 'idxknk_y', 'idxknk_z',
             'pnkknk_x', 'pnkknk_y', 'pnkknk_z'])

        # Inspire RH56 over Modbus TCP (inspire_demos library).
        self._hand = InspireHandTCP(
            ip=ip, port=port, generation=generation, dry_run=dry_run)
        if not dry_run and speed is not None:
            self._hand.set_speed([speed] * 6)
        # Cap grip force so the fingers can't over-drive into an object/joint.
        # FORCE_SET threshold 0-1000 (lower = gentler). Protects the actuators.
        if not dry_run and force is not None:
            applied = self._hand.set_force([force] * 6)
            print('[inspire] force cap set to {} (0-1000, lower=gentler)'.format(force))

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

    def _poll_teleop_state(self):
        """Read the shared pause/resume signal (non-blocking) and, if enabled,
        start/stop recording on the transition. Same signal that pauses the arm:
        index+thumb -> STOP, middle+thumb -> CONT."""
        if self._teleop_state_subscriber is None:
            return
        data = self._teleop_state_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
        if data is not None:
            self._teleop_state = int(np.asanyarray(data).reshape(1)[0])

        if not self.record_on_teleop:
            return
        want = (self._teleop_state == ARM_TELEOP_CONT)
        if want and not self._recording:
            self._seg_count += 1
            label = '{}_{:02d}'.format(time.strftime('%Y%m%d_%H%M%S'), self._seg_count)
            krec.start_session(label)
            self._recording = True
            print('[REC] recording STARTED (resume pinch) -> session {}'.format(label))
        elif not want and self._recording:
            krec.stop_session()
            self._recording = False
            print('[REC] recording STOPPED (pause pinch)')

    def _apply_retargeted_angles(self):
        self._poll_teleop_state()

        hand_coords = self._get_hand_keypoints()
        if hand_coords is None:
            return

        flex = self.retargeter.raw_flexions(hand_coords)  # {dof: deg}
        angles = self.retargeter.retarget(hand_coords)    # [6] in 0..1000

        # Record every cycle (~60 Hz) whenever a session is active.
        row = {
            'mode': 'dry' if self.dry_run else 'real',
            'flex_little': round(flex[0], 2), 'flex_ring': round(flex[1], 2),
            'flex_middle': round(flex[2], 2), 'flex_index': round(flex[3], 2),
            'flex_thumb_bend': round(flex[4], 2), 'flex_thumb_rot': round(flex[5], 2),
            'cmd_little': angles[0], 'cmd_ring': angles[1], 'cmd_middle': angles[2],
            'cmd_index': angles[3], 'cmd_thumb_bend': angles[4], 'cmd_thumb_rot': angles[5],
        }
        # Only read back actual angles/force from the RH56 while recording, so
        # the control loop isn't burdened by ~2 ms of Modbus reads when idle.
        if self._recorder.is_active():
            act = self._hand.get_angles()
            frc = self._hand.get_force()
            names = ['little', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rot']
            if act is not None:
                for i, n in enumerate(names):
                    row['act_' + n] = int(act[i])
            if frc is not None:
                for i, n in enumerate(names):
                    row['frc_' + n] = int(frc[i])
            # raw thumb-related keypoints for designing a real thumb-rot proxy
            for lbl, kp in (('thumbtip', 4), ('thumbmcp', 2), ('idxknk', 6), ('pnkknk', 15)):
                row[lbl + '_x'] = round(float(hand_coords[kp][0]), 5)
                row[lbl + '_y'] = round(float(hand_coords[kp][1]), 5)
                row[lbl + '_z'] = round(float(hand_coords[kp][2]), 5)
        self._recorder.maybe_log(row)

        # Freeze the hand while teleop is paused (index+thumb pinch): the RH56
        # holds its last commanded pose because we simply stop sending.
        if self.respect_teleop_pause and self._teleop_state == ARM_TELEOP_STOP:
            return

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
            if self._recording:
                krec.stop_session()
                self._recording = False
            self._recorder.close()
            self._transformed_hand_keypoint_subscriber.stop()
            if self._teleop_state_subscriber is not None:
                self._teleop_state_subscriber.stop()
            if not self.dry_run:
                self._hand.close()
            print('Stopping the Inspire hand operator.')
