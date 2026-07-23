#!/usr/bin/env python3
"""Fit the wrist->EE mapping from arm_*.csv recordings.

We map *relative* motion since each teleop reset, not absolute position, so
this script:

  1. Splits a recording into teleop-CONT segments (reset at file start and at
     each STOP->CONT transition -- the same instants `_reset_teleop` fires).
  2. References every frame to its segment's first (reset) frame.
  3. Reconstructs the hand-side input the pipeline actually sees:
        d_hand(t) = R_hand0^T @ (wpos(t) - wpos(t0))     # == H_HT_HI[:3,3]
     and the robot-side response in the base frame:
        d_robot(t) = cur(t) - cur(t0)   (optionally / res_scale)
  4. Fits the position map  M (3x3)  with   d_robot ~= M @ d_hand
     and the orientation map C (3x3, orthonormal) with
        R_robot_rel ~= C @ R_hand_rel @ C^T
     via the rotation-axis correspondence u_robot = C @ u_hand.

M / C are the *effective* calibration the system is applying. Compare M's
per-axis readout to the configured `axis_remap`, and C to `orient_remap`, to
see the README-section-5 asymmetry directly in data.

Usage:
    python scripts/analyze_wrist_ee_mapping.py logs/recordings/arm_*.csv
    python scripts/analyze_wrist_ee_mapping.py FILE --no-res-normalize --min-deg 5
"""
import argparse
import csv
import glob
import sys

import numpy as np

try:
    from scipy.spatial.transform import Rotation
except ImportError:
    sys.exit("scipy is required (pip install scipy)")

AXES = ('x', 'y', 'z')


def _f(row, key):
    """Parse a float cell; return None if missing/blank/unparseable."""
    v = row.get(key, '')
    if v is None or v == '':
        return None
    try:
        return float(v)
    except ValueError:
        return None


def load_frames(path):
    """Return list of per-frame dicts with parsed hand/robot pose, in order."""
    frames = []
    with open(path, newline='') as fh:
        reader = csv.DictReader(fh)
        cols = reader.fieldnames or []
        if 'wpos_x' not in cols:
            raise ValueError(
                "%s has no wpos_* columns -- it predates wrist-position "
                "logging. Re-record after the franka.py change to analyze "
                "position; only orientation (wq*) is available here." % path)
        for row in reader:
            wpos = [_f(row, 'wpos_' + a) for a in AXES]
            wq = [_f(row, 'wq%d' % i) for i in range(4)]           # xyzw
            cur = [_f(row, 'cur_' + a) for a in AXES]
            cq = [_f(row, 'cur_q%d' % i) for i in range(4)]        # xyzw
            state = _f(row, 'teleop_state')
            res = _f(row, 'res_scale')
            if None in wpos or None in wq or None in cur or None in cq:
                continue  # incomplete frame (e.g. robot state not ready yet)
            frames.append(dict(
                wpos=np.array(wpos), wq=np.array(wq),
                cur=np.array(cur), cq=np.array(cq),
                state=1 if state is None else int(round(state)),
                res=1.0 if (res is None or res == 0) else res,
            ))
    return frames


def segment(frames):
    """Split into CONT segments. New segment at index 0 and at every 0->1
    teleop transition; drop STOP (state==0) frames -- nothing is mapped then."""
    segments, cur_seg, prev_state = [], [], None
    for fr in frames:
        s = fr['state']
        if s == 0:
            prev_state = s
            continue
        # s == 1 (CONT)
        if prev_state == 0 or prev_state is None:
            if cur_seg:
                segments.append(cur_seg)
            cur_seg = []
        cur_seg.append(fr)
        prev_state = s
    if cur_seg:
        segments.append(cur_seg)
    return segments


def relative_signals(segments, res_normalize):
    """From all segments, build paired relative signals referenced to each
    segment's reset frame. Returns dicts of stacked arrays."""
    d_hand, d_robot = [], []              # (N,3) relative translations
    axis_hand, axis_robot = [], []        # (M,3) rotation-axis * angle
    ang_hand, ang_robot = [], []          # (M,) rotation angles (rad)
    for seg in segments:
        if len(seg) < 2:
            continue
        f0 = seg[0]
        R_h0 = Rotation.from_quat(f0['wq']).as_matrix()
        R_r0 = Rotation.from_quat(f0['cq']).as_matrix()
        t_h0, t_r0 = f0['wpos'], f0['cur']
        for fr in seg[1:]:
            # translation
            dh = R_h0.T @ (fr['wpos'] - t_h0)                 # == H_HT_HI[:3,3]
            dr = fr['cur'] - t_r0
            if res_normalize:
                dr = dr / fr['res']
            d_hand.append(dh)
            d_robot.append(dr)
            # rotation (relative to reset, each in its own base)
            R_hr = R_h0.T @ Rotation.from_quat(fr['wq']).as_matrix()
            R_rr = R_r0.T @ Rotation.from_quat(fr['cq']).as_matrix()
            rv_h = Rotation.from_matrix(R_hr).as_rotvec()
            rv_r = Rotation.from_matrix(R_rr).as_rotvec()
            axis_hand.append(rv_h)
            axis_robot.append(rv_r)
            ang_hand.append(np.linalg.norm(rv_h))
            ang_robot.append(np.linalg.norm(rv_r))
    return dict(
        d_hand=np.array(d_hand), d_robot=np.array(d_robot),
        axis_hand=np.array(axis_hand), axis_robot=np.array(axis_robot),
        ang_hand=np.array(ang_hand), ang_robot=np.array(ang_robot),
    )


def fit_linear_map(X, Y):
    """Least-squares M with Y ~= X @ M^T (rows are samples). Returns M (3x3),
    RMS residual, and relative residual (fraction of signal RMS unexplained)."""
    # Solve M^T minimizing ||X M^T - Y||;  X:(N,3) Y:(N,3)
    Mt, *_ = np.linalg.lstsq(X, Y, rcond=None)
    M = Mt.T
    resid = Y - X @ Mt
    rms = float(np.sqrt(np.mean(np.sum(resid**2, axis=1))))
    sig = float(np.sqrt(np.mean(np.sum(Y**2, axis=1)))) or 1.0
    return M, rms, rms / sig


def orthogonal_procrustes(A, B):
    """Best orthonormal C with B ~= C @ A (columns are vectors). det>0 forced."""
    U, _, Vt = np.linalg.svd(B @ A.T)
    C = U @ Vt
    if np.linalg.det(C) < 0:            # reflection -> flip to proper rotation
        U[:, -1] *= -1
        C = U @ Vt
    return C


def axis_readout(M):
    """Human-readable 'which robot axis does each hand axis drive'."""
    lines = []
    for j, ha in enumerate(AXES):
        col = M[:, j]
        i = int(np.argmax(np.abs(col)))
        sign = '+' if col[i] >= 0 else '-'
        mag = abs(col[i])
        leak = np.sum(np.abs(col)) - mag
        lines.append("  hand %s -> %srobot_%s  (gain %.3f, off-axis leak %.3f)"
                     % (ha, sign, AXES[i], mag, leak))
    return '\n'.join(lines)


def fmt_mat(M):
    return '\n'.join('   [' + '  '.join('% .4f' % v for v in r) + ']' for r in M)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('paths', nargs='+', help='arm_*.csv file(s) or globs')
    ap.add_argument('--no-res-normalize', action='store_true',
                    help='do NOT divide robot displacement by res_scale '
                         '(default: normalize so Low/High-res frames combine)')
    ap.add_argument('--min-deg', type=float, default=5.0,
                    help='ignore rotation frames below this angle when fitting '
                         'C (default 5 deg)')
    args = ap.parse_args()

    files = []
    for p in args.paths:
        files.extend(sorted(glob.glob(p)) or [p])

    all_frames, loaded, skipped = [], [], []
    for f in files:
        try:
            fr = load_frames(f)
        except (OSError, ValueError) as e:
            skipped.append((f, str(e)))
            continue
        if fr:
            all_frames.append(fr)
            loaded.append((f, len(fr)))

    for f, why in skipped:
        print("SKIP %s\n     %s" % (f, why))
    if not all_frames:
        sys.exit("No analyzable recordings (need wpos_* columns).")

    print("\n=== loaded ===")
    for f, n in loaded:
        print("  %-55s %5d frames" % (f.split('/')[-1], n))

    segments = []
    for fr in all_frames:
        segments.extend(segment(fr))
    print("teleop segments: %d  (reset-referenced independently)" % len(segments))

    sig = relative_signals(segments, res_normalize=not args.no_res_normalize)
    N = len(sig['d_hand'])
    if N < 4:
        sys.exit("Too few paired frames (%d) -- need real hand+arm motion. "
                 "Record a calibration pass (translate each axis, then pure "
                 "roll/pitch/yaw)." % N)

    # ---------------- POSITION ----------------
    span_h = np.ptp(sig['d_hand'], axis=0)
    span_r = np.ptp(sig['d_robot'], axis=0)
    M, rms, rel = fit_linear_map(sig['d_hand'], sig['d_robot'])
    # scale + closest rotation of M (a clean mapping is scale * rotation/perm)
    U, S, Vt = np.linalg.svd(M)
    R_close = orthogonal_procrustes(np.eye(3), M / (S.mean() or 1.0))

    print("\n=========================== POSITION ===========================")
    print("paired frames: %d   res_normalized: %s"
          % (N, not args.no_res_normalize))
    print("hand motion span  (m): x %.3f  y %.3f  z %.3f" % tuple(span_h))
    print("robot motion span (m): x %.3f  y %.3f  z %.3f" % tuple(span_r))
    print("\neffective position map  M   (d_robot = M @ d_hand):")
    print(fmt_mat(M))
    print("\naxis readout:")
    print(axis_readout(M))
    print("\nsingular values (isotropic scale if ~equal): %s"
          % np.array2string(S, precision=3))
    print("fit residual: %.4f m RMS   (%.1f%% of robot motion unexplained)"
          % (rms, 100 * rel))
    if max(span_h) < 0.02:
        print("!! hand barely moved (<2 cm) -- position fit is unreliable.")

    # ---------------- ORIENTATION ----------------
    min_rad = np.deg2rad(args.min_deg)
    mask = (sig['ang_hand'] > min_rad) & (sig['ang_robot'] > min_rad)
    print("\n========================= ORIENTATION ==========================")
    print("frames with rotation > %.0f deg: %d / %d"
          % (args.min_deg, int(mask.sum()), N))
    if mask.sum() >= 3:
        A = sig['axis_hand'][mask].T      # (3, m)
        B = sig['axis_robot'][mask].T
        C = orthogonal_procrustes(A, B)
        # angle-consistency: a pure conjugation preserves rotation angle
        dang = np.abs(sig['ang_robot'][mask] - sig['ang_hand'][mask])
        # residual of the axis correspondence after applying C
        axis_resid = np.linalg.norm(B - C @ A, axis=0)
        base = np.linalg.norm(B, axis=0)
        rel_axis = float(np.mean(axis_resid) / (np.mean(base) or 1.0))
        print("\neffective orientation map  C  (u_robot = C @ u_hand):")
        print(fmt_mat(C))
        print("\naxis readout:")
        print(axis_readout(C))
        print("\nrotation-angle match |theta_robot - theta_hand|: "
              "median %.1f deg  (should be ~0 for a clean rotation remap)"
              % np.rad2deg(np.median(dang)))
        print("axis-correspondence residual: %.1f%% unexplained" % (100 * rel_axis))
        print("\n-- compare M (position) vs C (orientation): if they differ, "
              "that IS the section-5 asymmetry, measured from your data.")
    else:
        print("Not enough wrist rotation to fit C. Add pure roll/pitch/yaw to "
              "the calibration pass.")
    print()


if __name__ == '__main__':
    main()
