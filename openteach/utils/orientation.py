"""Pure orientation-mapping math for the Franka teleop operator.

Kept import-light (numpy only) so it is unit-testable without pulling in the
robot/deoxys stack. The operator (openteach/components/operators/franka.py)
imports map_relative_orientation from here; see
docs/orientation_pipeline_redesign.md and docs/hand_to_robot_mapping.md §9.
"""
import numpy as np


def project_so3(R):
    """Nearest proper rotation to R (SVD projection onto SO(3)), det forced +1.

    Keeps a commanded orientation a clean rotation after the conjugation and
    matrix products in the mapping, so numerical drift can't accumulate into a
    skewed or improper matrix.
    """
    u, _, vt = np.linalg.svd(R)
    Rp = u @ vt
    if np.linalg.det(Rp) < 0:
        u = u.copy()
        u[:, -1] *= -1
        Rp = u @ vt
    return Rp


def map_relative_orientation(R_hand_now, R_hand_reset, R_robot_reset, C,
                             mode='body'):
    """Map the wrist's rotation-since-reset onto the robot EE orientation.

    Args:
        R_hand_now:    current wrist rotation (3x3), canonicalized.
        R_hand_reset:  wrist rotation captured at the last teleop resume.
        R_robot_reset: robot EE rotation captured at the last teleop resume.
        C:             calibratable frame transform (3x3, orient_remap / axis_remap).
        mode:          'body' (legacy, reset-pose dependent) or 'spatial'
                       (reset-independent).

    Returns:
        R_ee (3x3), projected onto SO(3).

    body    : R_ee = R_robot_reset @ (C @ dR_body  @ C^T),  dR_body  = R_hand_reset^T @ R_hand_now
              The EE's *base-frame* rotation-since-reset depends on R_robot_reset.
    spatial : R_ee = (C @ dR_world @ C^T) @ R_robot_reset,   dR_world = R_hand_now @ R_hand_reset^T
              The EE's base-frame rotation-since-reset = C @ dR_world @ C^T, a
              fixed function of C only -- independent of BOTH reset poses.
    """
    if mode == 'spatial':
        dR_world = R_hand_now @ R_hand_reset.T
        R_ee = (C @ dR_world @ C.T) @ R_robot_reset
    elif mode == 'body':
        dR_body = R_hand_reset.T @ R_hand_now
        R_ee = R_robot_reset @ (C @ dR_body @ C.T)
    else:
        raise ValueError("mode must be 'body' or 'spatial', got %r" % mode)
    return project_so3(R_ee)
