import numpy as np
import matplotlib.pyplot as plt
import zmq

from mpl_toolkits.mplot3d import Axes3D
from tqdm import tqdm

from copy import deepcopy as copy
from openteach.constants import *
from openteach.utils.timer import FrequencyTimer
from openteach.utils.network import ZMQKeypointSubscriber
from openteach.utils.vectorops import *
from openteach.utils.files import *
from openteach.robot.franka import FrankaArm
from scipy.spatial.transform import Rotation, Slerp
from .operator import Operator



np.set_printoptions(precision=2, suppress=True)

# Safety guard: max allowed change in the commanded end-effector target between
# consecutive frames (~60 Hz). A real hand cannot move this far in one frame, so
# a bigger jump means a VR tracking-loss / glitch frame -> we refuse to send it
# and the robot holds instead of lunging. 0.15 m/frame ~= 9 m/s of hand motion.
MAX_FRAME_JUMP_M = 0.15

# Workspace clamp: the commanded target position is clamped to a box of
# +/- WORKSPACE_BOX_M (per axis) around the reset reference position, so a large
# hand movement cannot drive the arm outside a safe, reachable region. Reset
# reference = the robot pose captured each time teleop starts/resumes.
WORKSPACE_BOX_M = 0.2

# Filter to smooth out the arm cartesian state
class Filter:
    def __init__(self, state, comp_ratio=0.6):
        self.pos_state = state[:3]
        self.ori_state = state[3:7]
        self.comp_ratio = comp_ratio

    def __call__(self, next_state):
        self.pos_state = self.pos_state[:3] * self.comp_ratio + next_state[:3] * (1 - self.comp_ratio)
        ori_interp = Slerp([0, 1], Rotation.from_quat(
            np.stack([self.ori_state, next_state[3:7]], axis=0)),)
        self.ori_state = ori_interp([1 - self.comp_ratio])[0].as_quat()
        return np.concatenate([self.pos_state, self.ori_state])

class FrankaArmOperator(Operator):
    def __init__(
        self,
        host,
        transformed_keypoints_port,
        use_filter=False,
        arm_resolution_port = None,
        teleoperation_reset_port = None,
        dry_run = False,
        flip_vertical = False,
    ):
        self.notify_component_start('franka arm operator')
        # Correct the Unity(left-handed, Y-up) vs robot(right-handed, Z-up)
        # handedness mismatch that makes hand-up map to robot-down. When True,
        # the vertical component of the mapped displacement is negated so
        # hand-up -> robot-up. Only affects vertical translation.
        self.flip_vertical = flip_vertical
        # Safety/validation flag. When True, everything runs normally
        # (streaming, state reading, retargeting, logging) but NO command is
        # ever sent to the robot -> the arm cannot move. See
        # docs/franka_validation.md.
        self.dry_run = dry_run
        self._dry_run_counter = 0
        if self.dry_run:
            print('\n' + '=' * 70)
            print(' FRANKA OPERATOR RUNNING IN DRY-RUN MODE')
            print(' Pipeline is fully live but NO commands will be sent to the robot.')
            print(' The arm will NOT move. Logging computed target poses only.')
            print('=' * 70 + '\n')
        # Subscribers for the transformed hand keypoints
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=transformed_keypoints_port,
            topic='transformed_hand_coords'
        )
        # Subscribers for the transformed arm frame
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=transformed_keypoints_port,
            topic='transformed_hand_frame'
        )

        # Initalizing the robot controller
        self._robot = FrankaArm()
        self.resolution_scale = 1 # NOTE: Get this from a socket
        self.arm_teleop_state = ARM_TELEOP_STOP # We will start as the cont

        # Subscribers for the resolution scale and teleop state
        self._arm_resolution_subscriber = ZMQKeypointSubscriber(
            host = host,
            port = arm_resolution_port,
            topic = 'button'
        )

        self._arm_teleop_state_subscriber = ZMQKeypointSubscriber(
            host = host, 
            port = teleoperation_reset_port,
            topic = 'pause'
        )
        # Robot Initial Frame
        self.robot_init_H = self.robot.get_pose()['position']
        self.is_first_frame = True

        # Safety-guard state: last target we actually accepted/sent, used to
        # reject implausibly large single-frame jumps (VR tracking glitches).
        self._prev_sent_pos = None
        self._reject_counter = 0
        # Center of the workspace clamp box, set at each teleop reset/resume.
        self._clamp_center = None

        self.use_filter = use_filter
        if use_filter:
            robot_init_cart = self._homo2cart(self.robot_init_H)
            self.comp_filter = Filter(robot_init_cart, comp_ratio=0.8)

        self._timer = FrequencyTimer(VR_FREQ)

    @property
    def timer(self):
        return self._timer

    @property
    def robot(self):
        return self._robot

    @property
    def transformed_hand_keypoint_subscriber(self):
        return self._transformed_hand_keypoint_subscriber
    
    @property
    def transformed_arm_keypoint_subscriber(self):
        return self._transformed_arm_keypoint_subscriber

    # Get the hand frame
    def _get_hand_frame(self):
        for i in range(10):
            data = self.transformed_arm_keypoint_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
            if not data is None: break
        if data is None: return None
        return np.asanyarray(data).reshape(4, 3)

    # Get the full transformed hand keypoints (all finger joints), wrist-relative.
    # This is the same 'transformed_hand_coords' stream a dexterous-hand
    # operator (e.g. Allegro) consumes; the arm operator normally ignores it.
    def _get_hand_keypoints(self):
        data = None
        for i in range(10):
            data = self.transformed_hand_keypoint_subscriber.recv_keypoints(flags=zmq.NOBLOCK)
            if data is not None: break
        if data is None: return None
        return np.asanyarray(data).reshape(OCULUS_NUM_KEYPOINTS, 3)

    # Compute per-finger flexion angles (degrees) from the hand keypoints,
    # using the same three-consecutive-keypoint method the Allegro retargeter
    # uses to drive robot finger joints (openteach.utils.vectorops.calculate_angle).
    def _finger_joint_angles(self, hand_coords):
        angles = {}
        for finger in ['thumb', 'index', 'middle', 'ring', 'pinky']:
            chain = [0] + list(OCULUS_JOINTS[finger])  # wrist + this finger's joints
            pts = hand_coords[chain]
            finger_angles = []
            for i in range(len(pts) - 2):
                rad = calculate_angle(pts[i], pts[i + 1], pts[i + 2])
                finger_angles.append(round(float(np.degrees(rad)), 1))
            angles[finger] = finger_angles
        return angles

    # Get the resolution scale mode (High or Low)
    def _get_resolution_scale_mode(self):
        data = self._arm_resolution_subscriber.recv_keypoints()
        res_scale = np.asanyarray(data).reshape(1)[0] # Make sure this data is one dimensional
        return res_scale  

    # Get the teleop state (Pause or Continue)
    def _get_arm_teleop_state(self):
        reset_stat = self._arm_teleop_state_subscriber.recv_keypoints()
        reset_stat = np.asanyarray(reset_stat).reshape(1)[0] # Make sure this data is one dimensional
        return reset_stat

    # Converts a frame to a homogenous transformation matrix
    def _turn_frame_to_homo_mat(self, frame):
        t = frame[0]
        R = frame[1:]

        homo_mat = np.zeros((4, 4))
        homo_mat[:3, :3] = np.transpose(R)
        homo_mat[:3, 3] = t
        homo_mat[3, 3] = 1

        return homo_mat
    
    # Converts Homogenous Transformation Matrix to Cartesian Coords
    def _homo2cart(self, homo_mat):
        
        t = homo_mat[:3, 3]
        R = Rotation.from_matrix(
            homo_mat[:3, :3]).as_quat()

        cart = np.concatenate(
            [t, R], axis=0
        )

        return cart
    
    # Gets the Scaled Resolution pose
    def _get_scaled_cart_pose(self, moving_robot_homo_mat):
        # Get the cart pose without the scaling
        unscaled_cart_pose = self._homo2cart(moving_robot_homo_mat)

        # Get the current cart pose
        current_homo_mat = copy(self.robot.get_pose()['position'])
        current_cart_pose = self._homo2cart(current_homo_mat)

        # Get the difference in translation between these two cart poses
        diff_in_translation = unscaled_cart_pose[:3] - current_cart_pose[:3]
        scaled_diff_in_translation = diff_in_translation * self.resolution_scale
        # print('SCALED_DIFF_IN_TRANSLATION: {}'.format(scaled_diff_in_translation))
        
        scaled_cart_pose = np.zeros(7)
        scaled_cart_pose[3:] = unscaled_cart_pose[3:] # Get the rotation directly
        scaled_cart_pose[:3] = current_cart_pose[:3] + scaled_diff_in_translation # Get the scaled translation only

        return scaled_cart_pose

    # Reset the teleoperation and get the first frame
    def _reset_teleop(self):
        # Just updates the beginning position of the arm
        print('****** RESETTING TELEOP ****** ')
        self.robot_init_H = self.robot.get_pose()['position']
        first_hand_frame = self._get_hand_frame()
        while first_hand_frame is None:
            first_hand_frame = self._get_hand_frame()
        self.hand_init_H = self._turn_frame_to_homo_mat(first_hand_frame)
        self.hand_init_t = copy(self.hand_init_H[:3, 3])
        self.is_first_frame = False
        # Clear the jump guard so the first frame after a reset/resume is
        # accepted (it snaps target to the robot's current pose -> ~0 motion).
        self._prev_sent_pos = None
        # Center the workspace clamp box on the robot pose at this reset.
        self._clamp_center = self._homo2cart(self.robot_init_H)[:3].copy()
        return first_hand_frame

    # Apply the retargeted angles
    def _apply_retargeted_angles(self, log=False):

        # See if there is a reset in the teleop
        new_arm_teleop_state = self._get_arm_teleop_state()
        if self.is_first_frame or (self.arm_teleop_state == ARM_TELEOP_STOP and new_arm_teleop_state == ARM_TELEOP_CONT):
            moving_hand_frame = self._reset_teleop() # Should get the moving hand frame only once
        else:
            moving_hand_frame = self._get_hand_frame() # Should get the hand frame 
        self.arm_teleop_state = new_arm_teleop_state 

        # Get the arm resolution
        arm_teleoperation_scale_mode = self._get_resolution_scale_mode()
        if arm_teleoperation_scale_mode == ARM_HIGH_RESOLUTION:
            self.resolution_scale = 1
        elif arm_teleoperation_scale_mode == ARM_LOW_RESOLUTION:
            self.resolution_scale = 0.6

        if moving_hand_frame is None: 
            return # It means we are not on the arm mode yet instead of blocking it is directly returning
        
        # Get the moving hand frame
        self.hand_moving_H = self._turn_frame_to_homo_mat(moving_hand_frame)

        # Transformation code
        H_HI_HH = copy(self.hand_init_H) # Homo matrix that takes P_HI  to P_HH - Point in Inital Hand Frame to Point in current hand Frame
        H_HT_HH = copy(self.hand_moving_H) # Homo matrix that takes P_HT to P_HH
        H_RI_RH = copy(self.robot_init_H) # Homo matrix that takes P_RI to P_RH

        # Rotation from allegro to franka
        H_A_R = np.array( 
            [[1/np.sqrt(2), 1/np.sqrt(2), 0, 0],
             [-1/np.sqrt(2), 1/np.sqrt(2), 0, 0],
             [0, 0, 1, -0.06], # The height of the allegro mount is 6cm
             [0, 0, 0, 1]])  

        H_HT_HI = np.linalg.pinv(H_HI_HH) @ H_HT_HH # Homo matrix that takes P_HT to P_HI
        H_RT_RH = H_RI_RH @ H_A_R @ H_HT_HI @ np.linalg.pinv(H_A_R) # Homo matrix that takes P_RT to P_RH

        # Vertical-axis handedness correction (Unity Y-up left-handed vs robot
        # Z-up right-handed made hand-up map to robot-down). Negate the vertical
        # component of the displacement from the reset pose so hand-up -> robot-up.
        if self.flip_vertical:
            disp = H_RT_RH[:3, 3] - H_RI_RH[:3, 3]
            disp[2] = -disp[2]
            H_RT_RH[:3, 3] = H_RI_RH[:3, 3] + disp

        self.robot_moving_H = copy(H_RT_RH)

        # Use the resolution scale to get the final cart pose
        final_pose = self._get_scaled_cart_pose(self.robot_moving_H)
        # Use a Filter
        if self.use_filter:
            final_pose = self.comp_filter(final_pose)

        # ---- Workspace clamp: keep target within a safe reachable box ----
        # Clamp the commanded position to +/- WORKSPACE_BOX_M per axis around the
        # reset reference pose, so a large hand movement can't drive the arm out
        # of reach. Orientation is left unclamped.
        clamped = False
        if self._clamp_center is not None:
            lo = self._clamp_center - WORKSPACE_BOX_M
            hi = self._clamp_center + WORKSPACE_BOX_M
            clamped_pos = np.clip(final_pose[:3], lo, hi)
            clamped = not np.allclose(clamped_pos, final_pose[:3])
            final_pose[:3] = clamped_pos

        # ---- Safety guard: reject tracking-loss / glitch frames ----
        # A non-finite pose, or a target that jumped further than a real hand
        # could move in one frame, indicates lost VR tracking. Refuse to send
        # it (the robot holds its last command) instead of lunging.
        reject_reason = None
        if not np.all(np.isfinite(final_pose)):
            reject_reason = 'non-finite pose (NaN/inf)'
        elif self._prev_sent_pos is not None:
            frame_jump = float(np.linalg.norm(final_pose[:3] - self._prev_sent_pos))
            if frame_jump > MAX_FRAME_JUMP_M:
                reject_reason = 'target jumped {:.2f} m in one frame (> {:.2f} m limit)'.format(
                    frame_jump, MAX_FRAME_JUMP_M)
        if reject_reason is not None:
            self._reject_counter += 1
            if self._reject_counter % 30 == 1:  # rate-limit the warning
                print('[SAFETY] rejecting frame: {} -- robot holding. '
                      '(pause+resume to re-reference)'.format(reject_reason))
            return
        # Accepted: remember it as the reference for the next frame's jump check.
        self._prev_sent_pos = final_pose[:3].copy()

        if self.dry_run:
            # Validation mode: log the computed target instead of moving.
            # Never calls arm_control -> robot_interface.control() is never
            # invoked, so nothing (not even the gripper handshake) is sent.
            self._dry_run_counter += 1
            if self._dry_run_counter % 30 == 0:  # ~2 Hz at VR_FREQ=60
                current_pose = self._homo2cart(self.robot.get_pose()['position'])
                delta = final_pose[:3] - current_pose[:3]
                print(
                    '[DRY-RUN][ARM] teleop_state={} res_scale={:.2f}{} | '
                    'current_xyz={} target_xyz={} delta_xyz(m)={} '
                    'target_quat={}'.format(
                        self.arm_teleop_state,
                        self.resolution_scale,
                        ' CLAMPED' if clamped else '',
                        np.round(current_pose[:3], 4),
                        np.round(final_pose[:3], 4),
                        np.round(delta, 4),
                        np.round(final_pose[3:], 4),
                    )
                )
                # Also log the human HAND posture (all finger joints). This is
                # the same finger-keypoint stream a dexterous-hand operator
                # would retarget onto a robotic hand -- logged here so you can
                # verify finger tracking is alive and sane before wiring up a
                # real hand. (The arm-only franka config never moves a hand.)
                hand_coords = self._get_hand_keypoints()
                if hand_coords is None:
                    print('[DRY-RUN][HAND] no finger keypoints received yet')
                else:
                    joint_angles = self._finger_joint_angles(hand_coords)
                    print('[DRY-RUN][HAND] finger flexion angles (deg): ' +
                          ' '.join('{}={}'.format(f, joint_angles[f])
                                   for f in ['thumb', 'index', 'middle', 'ring', 'pinky']))
            return
        # Move the robot arm
        self.robot.arm_control(final_pose)

    def stream(self):
        self.notify_component_start('{} control'.format(self.robot.name))
        print("Start controlling the robot hand using the Oculus Headset.\n")

        # Assume that the initial position is considered initial after 3 seconds of the start
        while True:
            try:
                if self.robot.get_joint_position() is not None:
                    self.timer.start_loop()

                    # Retargeting function
                    self._apply_retargeted_angles(log=False)

                    self.timer.end_loop()
            except KeyboardInterrupt:
                break

        self.transformed_arm_keypoint_subscriber.stop()
        print('Stopping the teleoperator!')
