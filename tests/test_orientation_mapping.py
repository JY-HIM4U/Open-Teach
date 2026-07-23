"""Property tests for the Franka orientation mapping (spatial vs body).

Pure-math tests -- no hardware, no robot/deoxys imports. Run with:
    python -m pytest tests/test_orientation_mapping.py -q
or standalone:
    python tests/test_orientation_mapping.py

These formalize the claim in docs/orientation_pipeline_redesign.md: the
'spatial' mode makes the EE's base-frame rotation-since-reset a fixed function
of C only -- independent of both the hand and robot reset poses -- while the
legacy 'body' mode does not. If test_spatial_is_reset_independent passes,
hand_to_robot_mapping.md §9.3 (reset-dependent axis map) is dead for spatial.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from openteach.utils.orientation import (  # noqa: E402
    map_relative_orientation, project_so3)


def _rand_rot(seed):
    """Deterministic pseudo-random rotation matrix (no scipy dependency)."""
    rng = np.random.RandomState(seed)
    A = rng.randn(3, 3)
    q, r = np.linalg.qr(A)
    q = q @ np.diag(np.sign(np.diag(r)))       # fix QR sign ambiguity
    if np.linalg.det(q) < 0:
        q[:, 0] *= -1
    return q


def _axis_angle(R):
    ang = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if ang < 1e-9:
        return np.array([0.0, 0.0, 1.0]), 0.0
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    ax = ax / (np.linalg.norm(ax) + 1e-12)
    return ax, ang


C_PERM = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])   # the config axis_remap (det +1)


def test_project_so3_is_proper_rotation():
    for s in range(20):
        R = project_so3(_rand_rot(s) + 1e-3 * np.random.RandomState(s).randn(3, 3))
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
        assert np.isclose(np.linalg.det(R), 1.0, atol=1e-9)


def test_identity_no_hand_motion_holds_robot_reset():
    """Zero hand motion => EE stays exactly at the robot reset, both modes."""
    for mode in ('body', 'spatial'):
        for s in range(10):
            Rh = _rand_rot(s)
            Rr = _rand_rot(s + 100)
            out = map_relative_orientation(Rh, Rh, Rr, C_PERM, mode=mode)
            assert np.allclose(out, Rr, atol=1e-9), mode


def test_spatial_is_reset_independent():
    """THE redesign claim: in spatial mode the EE's base-frame rotation-since-
    reset (out @ R_robot_reset^T) depends only on C and the hand's WORLD motion
    -- not on either reset pose. Vary both reset poses; it must not change."""
    for s in range(50):
        rng = np.random.RandomState(s)
        # a fixed physical hand world-motion (same across the reset variations)
        dR_world = _rand_rot(1000 + s)
        results = []
        for k in range(4):
            R_hand_reset = _rand_rot(2000 + s * 7 + k)
            R_hand_now = dR_world @ R_hand_reset          # same world delta, different reset
            R_robot_reset = _rand_rot(3000 + s * 13 + k)
            out = map_relative_orientation(R_hand_now, R_hand_reset,
                                           R_robot_reset, C_PERM, mode='spatial')
            ee_world_delta = out @ R_robot_reset.T        # base-frame rotation since reset
            results.append(ee_world_delta)
        for r in results[1:]:
            assert np.allclose(r, results[0], atol=1e-7), \
                "spatial EE world-delta changed with reset pose -- reset-dependent!"
        # and it equals C @ dR_world @ C^T
        assert np.allclose(results[0], C_PERM @ dR_world @ C_PERM.T, atol=1e-7)


def test_body_is_reset_dependent():
    """Contrast: body mode's base-frame rotation-since-reset DOES vary with the
    robot reset pose (this is the §9.3 behavior spatial fixes)."""
    changed = False
    for s in range(50):
        dR_world = _rand_rot(1000 + s)
        deltas = []
        for k in range(4):
            R_hand_reset = _rand_rot(2000 + s * 7 + k)
            R_hand_now = dR_world @ R_hand_reset
            R_robot_reset = _rand_rot(3000 + s * 13 + k)
            out = map_relative_orientation(R_hand_now, R_hand_reset,
                                           R_robot_reset, C_PERM, mode='body')
            deltas.append(out @ R_robot_reset.T)
        if not all(np.allclose(d, deltas[0], atol=1e-6) for d in deltas[1:]):
            changed = True
            break
    assert changed, "body mode unexpectedly reset-independent -- test assumption broken"


def test_spatial_preserves_rotation_angle():
    """Conjugation by an orthogonal C preserves rotation angle: the EE turns by
    the same amount the hand did (with C_PERM, which is a proper rotation)."""
    for s in range(30):
        R_hand_reset = _rand_rot(4000 + s)
        dR_world = _rand_rot(5000 + s)
        R_hand_now = dR_world @ R_hand_reset
        R_robot_reset = _rand_rot(6000 + s)
        out = map_relative_orientation(R_hand_now, R_hand_reset,
                                       R_robot_reset, C_PERM, mode='spatial')
        _, hand_ang = _axis_angle(dR_world)
        _, ee_ang = _axis_angle(out @ R_robot_reset.T)
        assert np.isclose(hand_ang, ee_ang, atol=1e-6)


def test_spatial_axis_maps_by_C():
    """A hand world-rotation about axis a produces an EE base-frame rotation
    about C @ a -- the property the empirical calibration is trying to find,
    now deterministic."""
    from numpy import cos, sin
    for a_idx, axis in enumerate(np.eye(3)):
        th = 0.5
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        dR_world = np.eye(3) + sin(th) * K + (1 - cos(th)) * (K @ K)   # Rodrigues
        R_hand_reset = _rand_rot(7000 + a_idx)
        R_hand_now = dR_world @ R_hand_reset
        R_robot_reset = _rand_rot(8000 + a_idx)
        out = map_relative_orientation(R_hand_now, R_hand_reset,
                                       R_robot_reset, C_PERM, mode='spatial')
        ee_axis, _ = _axis_angle(out @ R_robot_reset.T)
        expected = C_PERM @ axis
        # axis sign is ambiguous; compare up to sign
        assert (np.allclose(ee_axis, expected, atol=1e-5)
                or np.allclose(ee_axis, -expected, atol=1e-5))


if __name__ == '__main__':
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        fn()
        print('PASS', fn.__name__)
    print('\nall %d tests passed' % len(fns))
