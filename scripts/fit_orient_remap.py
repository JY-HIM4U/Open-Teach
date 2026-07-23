#!/usr/bin/env python3
"""Fit orient_remap (the wrist->robot orientation axis map) from arm_*.csv.

Uses the COMMANDED robot orientation (quat0-3) -- the clean signal the operator
actually produces -- not the noisy achieved pose (cur_q*). Glitch-filters the
raw wrist (wq*) the same way the operator's orient_glitch guard does, so the
flip artifacts in the log don't corrupt the fit.

Record roll, then pitch, then yaw (slow, ~10 s each) -- either as one session or
three files -- and pass the arm CSV(s):

    python scripts/fit_orient_remap.py logs/recordings/arm_<roll> <pitch> <yaw>

Output: the nearest signed-permutation matrix to drop into orient_remap, a
readable per-axis mapping, and a residual (the quality gate -- want < ~25%).
"""
import argparse, csv, sys
import numpy as np
try:
    from scipy.spatial.transform import Rotation as R
except ImportError:
    sys.exit("scipy required")

HAND_AX = ['hand_across(x)', 'hand_palmNormal=YAW(y)', 'hand_point=ROLL(z)']
ROB_AX = ['robotX', 'robotY', 'robotZ']


def load(path):
    wq, tq = [], []
    for r in csv.DictReader(open(path)):
        try:
            wq.append(R.from_quat([float(r['wq%d' % i]) for i in range(4)]).as_matrix())
            tq.append(R.from_quat([float(r['quat%d' % i]) for i in range(4)]).as_matrix())
        except (KeyError, ValueError):
            continue
    return wq, tq


def glitch_filter(Ms, thr_deg=45):
    """Reproduce the operator's orient_glitch guard: reject >thr/frame, hold."""
    out = [Ms[0]]
    good = Ms[0]
    for m in Ms[1:]:
        ang = np.degrees(np.arccos(np.clip((np.trace(m @ good.T) - 1) / 2, -1, 1)))
        if ang > thr_deg:
            out.append(good)
        else:
            out.append(m)
            good = m
    return out


def nearest_signed_perm(C):
    P = np.zeros((3, 3))
    a = np.abs(C).copy()
    for _ in range(3):
        i, j = np.unravel_index(np.argmax(a), a.shape)
        P[i, j] = 1 if C[i, j] >= 0 else -1
        a[i, :] = -1
        a[:, j] = -1
    return P


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('paths', nargs='+', help='arm_*.csv (roll pitch yaw)')
    ap.add_argument('--min-deg', type=float, default=8.0,
                    help='ignore frames below this hand rotation (default 8)')
    args = ap.parse_args()

    A, B = [], []
    print("=== per-clip ===")
    for p in args.paths:
        wq, tq = load(p)
        if len(wq) < 5:
            print("  %s: too few frames / no quat columns -- skipped" % p)
            continue
        wq = glitch_filter(wq)
        h0, r0 = wq[0], tq[0]
        hh, rr = [], []
        for m, c in zip(wq[1:], tq[1:]):
            hv = R.from_matrix(h0.T @ m).as_rotvec()
            rv = R.from_matrix(r0.T @ c).as_rotvec()
            if np.degrees(np.linalg.norm(hv)) > args.min_deg:
                A.append(hv); B.append(rv); hh.append(hv); rr.append(rv)
        if not hh:
            print("  %s: no motion above %.0f deg" % (p.split('/')[-1], args.min_deg))
            continue
        hh, rr = np.array(hh), np.array(rr)
        ha = np.abs(hh).mean(0); ha /= ha.sum()
        ra = np.abs(rr).mean(0); ra /= ra.sum()
        corr = np.corrcoef(np.linalg.norm(hh, axis=1), np.linalg.norm(rr, axis=1))[0, 1]
        print("  %-40s hand~%s -> cmd~%s  (corr %+.2f, hand max %.0f deg)"
              % (p.split('/')[-1], HAND_AX[np.argmax(ha)], ROB_AX[np.argmax(ra)],
                 corr, np.degrees(np.linalg.norm(hh, axis=1)).max()))

    if len(A) < 10:
        sys.exit("\nNot enough motion to fit. Record slower/larger roll/pitch/yaw.")

    A = np.array(A).T
    B = np.array(B).T
    w = np.linalg.norm(A, axis=0)
    U, S, Vt = np.linalg.svd((B * w) @ (A * w).T)
    C = U @ Vt
    if np.linalg.det(C) < 0:
        U[:, -1] *= -1
        C = U @ Vt
    resid = float(np.linalg.norm(B - C @ A) / (np.linalg.norm(B) + 1e-9))
    P = nearest_signed_perm(C)

    print("\n=== fitted orientation map (robot_axis = C @ hand_axis) ===")
    for r in C:
        print("   [% .3f % .3f % .3f]" % tuple(r))
    print("\n=== orient_remap (nearest signed permutation) -> put in inspire_franka.yaml ===")
    print("    orient_remap:")
    for r in P:
        print("      - [% .0f, % .0f, % .0f]" % tuple(r))
    print("\nreadable:")
    for j in range(3):
        i = int(np.argmax(np.abs(P[:, j])))
        s = '+' if P[i, j] >= 0 else '-'
        print("   %-24s -> %s%s" % (HAND_AX[j], s, ROB_AX[i]))
    print("\nfit residual: %.0f%%  %s" % (
        100 * resid,
        "GOOD -- trust this" if resid < 0.25 else
        "HIGH -- data too noisy/incomplete; re-record cleaner roll/pitch/yaw"))


if __name__ == '__main__':
    main()
