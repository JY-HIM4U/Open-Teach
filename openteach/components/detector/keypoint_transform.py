import numpy as np
from copy import deepcopy as copy
from openteach.components import Component
from openteach.constants import *
from openteach.utils.vectorops import *
from openteach.utils.network import ZMQKeypointPublisher, ZMQKeypointSubscriber,ZMQButtonFeedbackSubscriber
from openteach.utils.timer import FrequencyTimer

class TransformHandPositionCoords(Component):
    def __init__(self, host, keypoint_port, transformation_port,moving_average_limit = 5,
                 sign_continuity = True):
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

    # Keep the palm normal on the same hemisphere as the previous frame so a
    # sign inversion of cross(index, pinky) doesn't flip the whole wrist frame.
    def _sign_continuous_normal(self, palm_normal):
        if self.sign_continuity and self._prev_palm_normal is not None:
            if np.dot(palm_normal, self._prev_palm_normal) < 0:
                palm_normal = -palm_normal
        self._prev_palm_normal = palm_normal
        return palm_normal

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
    def _get_hand_dir_frame(self, origin_coord, index_knuckle_coord, pinky_knuckle_coord, palm_normal):
        palm_direction = normalize_vector(index_knuckle_coord + pinky_knuckle_coord)         # Unity space - Z
        cross_product = normalize_vector(index_knuckle_coord - pinky_knuckle_coord)          # Unity space - X

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