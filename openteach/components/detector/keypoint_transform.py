import numpy as np
from copy import deepcopy as copy
from openteach.components import Component
from openteach.constants import *
from openteach.utils.vectorops import *
from openteach.utils.network import ZMQKeypointPublisher, ZMQKeypointSubscriber,ZMQButtonFeedbackSubscriber
from openteach.utils.timer import FrequencyTimer

class TransformHandPositionCoords(Component):
    def __init__(self, host, keypoint_port, transformation_port,moving_average_limit = 5,
                 sign_continuity = True, frame_glitch_deg = 45.0):
        self.notify_component_start('keypoint position transform')

        # Initializing the subscriber for right hand keypoints
        self.original_keypoint_subscriber = ZMQKeypointSubscriber(host, keypoint_port, 'right')
        # Initializing the publisher for transformed right hand keypoints
        self.transformed_keypoint_publisher = ZMQKeypointPublisher(host, transformation_port)
        # Timer
        self.timer = FrequencyTimer(VR_FREQ)
        # Keypoint indices for knuckles
        self.knuckle_points = (OCULUS_JOINTS['knuckles'][0], OCULUS_JOINTS['knuckles'][-1])
        # Moving average queue
        self.moving_average_limit = moving_average_limit
        # Create a queue for moving average
        self.coord_moving_average_queue, self.frame_moving_average_queue = [], []
        # Sign-continuity for the reconstructed palm normal (see below). The wrist
        # frame is rebuilt each frame from cross(index_knuckle, pinky_knuckle); that
        # cross product INVERTS sign when the knuckle geometry passes near-degenerate
        # (which happens as you roll/tilt the wrist), flipping the whole frame ~180deg
        # even though the hand moved continuously -> the arm can't follow roll. Keeping
        # the normal on the same branch as the previous frame lets roll pass through
        # smoothly. Toggle off (per robot, via config) to restore the old behaviour.
        self.sign_continuity = sign_continuity
        self._prev_palm_normal = None

        # Single-frame outlier rejection for the WRIST/ARM frame. Quest hand
        # tracking occasionally spits out a ~180deg-flipped pose for one frame
        # at extreme wrist poses; sign-continuity (which only guards palm_normal)
        # does not catch these, and feeding one into the 5-sample moving average
        # corrupts the arm orientation for several frames. Reject any frame whose
        # rotation jumps more than frame_glitch_deg from the last good one and
        # hold the last good ROTATION (position/origin still flows through), so
        # the average downstream never sees the flip. Same threshold idea as the
        # operator's orient_glitch_deg, but applied at the source before averaging.
        self.frame_glitch_deg = frame_glitch_deg
        self._prev_good_frame = None
        print('[keypoint_transform] BUILD 2026-07-22b : orthonormal arm frame + '
              'outlier-reject filter ACTIVE (frame_glitch_deg=%s, sign_continuity=%s)'
              % (frame_glitch_deg, sign_continuity), flush=True)

    # Keep the palm normal on the same hemisphere as the previous frame so a
    # sign inversion of cross(index, pinky) doesn't flip the whole wrist frame.
    def _sign_continuous_normal(self, palm_normal):
        if self.sign_continuity and self._prev_palm_normal is not None:
            if np.dot(palm_normal, self._prev_palm_normal) < 0:
                palm_normal = -palm_normal
        self._prev_palm_normal = palm_normal
        return palm_normal

    # Reject single-frame ~180deg tracking flips in the wrist/arm frame. Compares
    # the new frame's rotation (the 3 basis vectors) against the last accepted one;
    # if it jumped more than frame_glitch_deg, hold the last good ROTATION but keep
    # the new origin so wrist position keeps tracking. hand_dir_frame is
    # [origin, X, Y, Z]; the basis rows are already orthonormal (see _get_coord_frame
    # / _get_hand_dir_frame), so trace() gives the geodesic angle directly.
    def _reject_frame_glitch(self, hand_dir_frame):
        if self.frame_glitch_deg is None or self.frame_glitch_deg <= 0:
            return hand_dir_frame
        R_cur = np.asarray(hand_dir_frame[1:], dtype=float)
        if self._prev_good_frame is not None:
            R_prev = np.asarray(self._prev_good_frame[1:], dtype=float)
            cos = (np.trace(R_cur @ R_prev.T) - 1.0) / 2.0
            ang = np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))
            if ang > self.frame_glitch_deg:
                # Reject: keep fresh wrist position, hold last good rotation.
                return [hand_dir_frame[0]] + list(self._prev_good_frame[1:])
        self._prev_good_frame = hand_dir_frame
        return hand_dir_frame

    # Function to get the hand coordinates from the VR
    def _get_hand_coords(self):
        data = self.original_keypoint_subscriber.recv_keypoints()
        if data[0] == 0:
            data_type = 'absolute'
        else:
            data_type = 'relative'
        return data_type, np.asanyarray(data[1:]).reshape(OCULUS_NUM_KEYPOINTS, 3)
    
    # Function to find hand coordinates with respect to the wrist
    def _translate_coords(self, hand_coords):
        return copy(hand_coords) - hand_coords[0]

    # Create a coordinate frame for the hand. palm_normal is passed in (sign-
    # corrected) so cross_product follows its sign and the frame stays a proper
    # right-handed rotation on a continuous branch.
    def _get_coord_frame(self, index_knuckle_coord, pinky_knuckle_coord, palm_normal):
        palm_direction = normalize_vector(index_knuckle_coord + pinky_knuckle_coord)         # Current Y
        cross_product = normalize_vector(np.cross(palm_direction, palm_normal))              # Current X
        return [cross_product, palm_direction, palm_normal]

    # Create a coordinate frame for the arm. Uses the same sign-corrected palm_normal.
    # cross_product is DERIVED from cross(palm_direction, palm_normal) -- NOT from an
    # independent (index - pinky) difference -- so the three axes are guaranteed
    # orthonormal and right-handed, and cross_product's sign is tied to the
    # sign-continuous palm_normal. The old (index - pinky) axis was neither
    # orthogonalized nor sign-stabilized, so during roll/yaw the triple flipped
    # handedness ~180deg (bimodal 0/180 wrist frame -> arm couldn't follow roll/yaw).
    def _get_hand_dir_frame(self, origin_coord, index_knuckle_coord, pinky_knuckle_coord, palm_normal):
        palm_direction = normalize_vector(index_knuckle_coord + pinky_knuckle_coord)         # Unity space - Z
        cross_product = normalize_vector(np.cross(palm_direction, palm_normal))              # Unity space - X

        return [origin_coord, cross_product, palm_normal, palm_direction]

    def transform_keypoints(self, hand_coords):
        translated_coords = self._translate_coords(hand_coords)
        index_knuckle_coord = translated_coords[self.knuckle_points[0]]
        pinky_knuckle_coord = translated_coords[self.knuckle_points[1]]

        # Reconstruct the palm normal ONCE and enforce sign-continuity, then share
        # it between both frames so a spurious cross-product flip can't invert one
        # frame relative to the other (or relative to the previous timestep).
        palm_normal = self._sign_continuous_normal(
            normalize_vector(np.cross(index_knuckle_coord, pinky_knuckle_coord)))

        original_coord_frame = self._get_coord_frame(
            index_knuckle_coord, pinky_knuckle_coord, palm_normal
        )

        # Finding the rotation matrix and rotating the coordinates
        rotation_matrix = np.linalg.solve(original_coord_frame, np.eye(3)).T
        transformed_hand_coords = (rotation_matrix @ translated_coords.T).T

        hand_dir_frame = self._get_hand_dir_frame(
            hand_coords[0],
            index_knuckle_coord,
            pinky_knuckle_coord,
            palm_normal
        )

        return transformed_hand_coords, hand_dir_frame

    def stream(self):
        while True:
            try:
                self.timer.start_loop()
                data_type, hand_coords = self._get_hand_coords()

               
                # Shift the points to required axes
                transformed_hand_coords, translated_hand_coord_frame = self.transform_keypoints(hand_coords)

                # Reject single-frame tracking flips in the arm frame BEFORE the
                # moving average, so one bad frame can't corrupt several outputs.
                translated_hand_coord_frame = self._reject_frame_glitch(translated_hand_coord_frame)

                # Passing the transformed coords into a moving average
                self.averaged_hand_coords = moving_average(
                    transformed_hand_coords, 
                    self.coord_moving_average_queue, 
                    self.moving_average_limit
                )

                self.averaged_hand_frame = moving_average(
                    translated_hand_coord_frame, 
                    self.frame_moving_average_queue, 
                    self.moving_average_limit
                )

                self.transformed_keypoint_publisher.pub_keypoints(self.averaged_hand_coords, 'transformed_hand_coords')
                if data_type == 'absolute':
                    self.transformed_keypoint_publisher.pub_keypoints(self.averaged_hand_frame, 'transformed_hand_frame')

                self.timer.end_loop()
            except:
                break
        
        self.original_keypoint_subscriber.stop()
        self.transformed_keypoint_publisher.stop()

        print('Stopping the keypoint position transform process.')