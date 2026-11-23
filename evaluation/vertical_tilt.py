#!/usr/bin/env python3
"""Vertical and tilt errors of LIO trajectories in a gravity-aligned evaluation.

The paper's APE aligns each estimate with a full SE(3) Umeyama fit, whose rotation can tilt
the whole trajectory and so trade vertical error for horizontal error. A LIO estimate lives
in a gravity-aligned frame, so here it is aligned with a rotation about the vertical axis
plus a translation only (4 DoF, as in visual-inertial evaluation). Per run:

  ape4      position RMSE after the 4-DoF fit [m]
  ape4_xy   horizontal part of that RMSE [m]
  ape4_z    vertical part [m]
  z_end_pct |vertical error| at the end, first pose anchored (yaw + translation), % of path
  zstep5    largest sustained vertical error, first pose anchored: max |5 s running median| [m]
  tilt_rms  RMSE of the tilt error, the angle between the true and estimated gravity
            directions in the body frame [deg] (independent of yaw and of any alignment)
  tilt_max  its maximum [deg];  tilt_0: its mean over the first 2 s [deg]
  se3_tilt  tilt of the SE(3) Umeyama rotation, i.e. how much the paper's alignment tilts
            the trajectory [deg]
  ape_z_se3 vertical RMSE after the SE(3) fit (the paper's ape_z) [m];  ape_xy_se3 horizontal
  tilt_se3  RMSE of the tilt error after the rotation of that SE(3) fit [deg]

Start attitude given ("b" columns). MOLA and FAST-LIO2 output their trajectory in the frame of
the first pose, which is as tilted as the start; DLIO starts gravity-aligned. To compare the
estimation and not that convention, the whole estimate is first rotated so that its first
attitude is the true one, then fitted in yaw and translation only:
  ape4b, ape4b_xy, ape4b_z   as ape4*, after that rotation [m]
  zstep5b, z_end_pctb        as zstep5, z_end_pct, first pose fully anchored
  tilt_rmsb, tilt_maxb       tilt error after that rotation: attitude drift since the start

Usage: vertical_tilt.py --runs <runs root> --methods m1,m2 [--variants ...] -o out.tsv
"""
import argparse
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_traj import classify, gt_spline, load_tum, score, to_T, umeyama  # noqa: E402

COLS = ['variant', 'world', 'traj', 'method', 'status', 'length_m', 'ape', 'ape4', 'ape4_xy',
        'ape4_z', 'ape_z_se3', 'zstep5', 'z_end_pct', 'tilt_rms', 'tilt_max', 'tilt_0', 'se3_tilt',
        'ape_xy_se3', 'tilt_se3', 'ape4b', 'ape4b_xy', 'ape4b_z', 'zstep5b', 'z_end_pctb',
        'tilt_rmsb', 'tilt_maxb']


def yaw_fit(src, dst):
    """Rotation about z and translation that best map src points onto dst points."""
    ms = src.mean(0)
    md = dst.mean(0)
    a = src[:, :2] - ms[:2]
    b = dst[:, :2] - md[:2]
    th = np.arctan2((a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum(), (a * b).sum())
    T = np.eye(4)
    T[:2, :2] = [[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]
    T[:3, 3] = md - T[:3, :3] @ ms
    return T


def tilt_deg(R):
    """Tilt of a rotation: angle between its z axis and the world z axis."""
    return np.degrees(np.arccos(np.clip(R[..., 2, 2], -1, 1)))


def tilt_err(G, R):
    """Angle [deg] between the true and estimated gravity directions in the body frame."""
    est = R[:, 2, :3] / np.linalg.norm(R[:, 2, :3], axis=1, keepdims=True)
    return np.degrees(np.arccos(np.clip((G[:, 2, :3] * est).sum(1), -1, 1)))


def zstep(t, ez):
    lo = np.searchsorted(t, t - 2.5)
    hi = np.searchsorted(t, t + 2.5)
    return float(max(abs(np.median(ez[i:j])) for i, j in zip(lo, hi)))


def one(job):
    variant, world, traj, method, tum = job
    base = score(world, traj, tum)
    r = {'variant': variant, 'world': world, 'traj': traj, 'method': method}
    if base.get('status') == 'no_output':
        r['status'] = 'no_output'
        return r
    r['status'] = classify(base)
    r['length_m'] = base['length_m']
    r['ape'] = base['ape']
    t, E = load_tum(tum)
    t0, pos, rot = gt_spline(world, traj)
    ok = (t >= t0[0]) & (t <= t0[-1])
    t = t[ok]
    E = E[ok]
    G = to_T(pos(t), rot(t).as_quat())
    A = yaw_fit(E[:, :3, 3], G[:, :3, 3])
    Ea = A @ E
    err = Ea[:, :3, 3] - G[:, :3, 3]
    r['ape4'] = float(np.sqrt((err ** 2).sum(1).mean()))
    r['ape4_xy'] = float(np.sqrt((err[:, :2] ** 2).sum(1).mean()))
    r['ape4_z'] = float(np.sqrt((err[:, 2] ** 2).mean()))
    # First pose anchored: yaw and translation of the first pose only.
    dyaw = (np.arctan2(G[0, 1, 0], G[0, 0, 0]) - np.arctan2(E[0, 1, 0], E[0, 0, 0]))
    T0 = np.eye(4)
    T0[:2, :2] = [[np.cos(dyaw), -np.sin(dyaw)], [np.sin(dyaw), np.cos(dyaw)]]
    E0 = T0 @ E
    E0[:, :3, 3] += G[0, :3, 3] - E0[0, :3, 3]
    r['z_end_pct'] = float(100 * abs(E0[-1, 2, 3] - G[-1, 2, 3]) / max(base['length_m'], 1e-9))
    ez = E0[:, 2, 3] - G[:, 2, 3]
    lo = np.searchsorted(t, t - 2.5)
    hi = np.searchsorted(t, t + 2.5)
    r['zstep5'] = float(max(abs(np.median(ez[i:j])) for i, j in zip(lo, hi)))
    # Gravity direction in the body frame: the third row of each rotation.
    gt_g = G[:, 2, :3]
    est_g = E[:, 2, :3] / np.linalg.norm(E[:, 2, :3], axis=1, keepdims=True)
    tilt = np.degrees(np.arccos(np.clip((gt_g * est_g).sum(1), -1, 1)))
    r['tilt_rms'] = float(np.sqrt((tilt ** 2).mean()))
    r['tilt_max'] = float(tilt.max())
    first = t <= t[0] + 2.0
    r['tilt_0'] = float(tilt[first].mean())
    S = umeyama(E[:, :3, 3], G[:, :3, 3])
    r['se3_tilt'] = float(tilt_deg(S[:3, :3]))
    SE = S @ E
    es = SE[:, :3, 3] - G[:, :3, 3]
    r['ape_z_se3'] = float(np.sqrt((es[:, 2] ** 2).mean()))
    r['ape_xy_se3'] = float(np.sqrt((es[:, :2] ** 2).sum(1).mean()))
    r['tilt_se3'] = float(np.sqrt((tilt_err(G, SE[:, :3, :3]) ** 2).mean()))
    # Start attitude given: rotate the whole estimate so its first attitude is the true one.
    RA = np.eye(4)
    RA[:3, :3] = G[0, :3, :3] @ E[0, :3, :3].T
    EB = RA @ E
    B = yaw_fit(EB[:, :3, 3], G[:, :3, 3])
    eb = (B @ EB)[:, :3, 3] - G[:, :3, 3]
    r['ape4b'] = float(np.sqrt((eb ** 2).sum(1).mean()))
    r['ape4b_xy'] = float(np.sqrt((eb[:, :2] ** 2).sum(1).mean()))
    r['ape4b_z'] = float(np.sqrt((eb[:, 2] ** 2).mean()))
    EB[:, :3, 3] += G[0, :3, 3] - EB[0, :3, 3]
    ezb = EB[:, 2, 3] - G[:, 2, 3]
    r['zstep5b'] = zstep(t, ezb)
    r['z_end_pctb'] = float(100 * abs(ezb[-1]) / max(base['length_m'], 1e-9))
    tb = tilt_err(G, EB[:, :3, :3])
    r['tilt_rmsb'] = float(np.sqrt((tb ** 2).mean()))
    r['tilt_maxb'] = float(tb.max())
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', required=True)
    ap.add_argument('--methods', required=True)
    ap.add_argument('--variants', default='')
    ap.add_argument('--procs', type=int, default=4)
    ap.add_argument('-o', '--output', required=True)
    a = ap.parse_args()
    methods = a.methods.split(',')
    variants = set(a.variants.split(',')) if a.variants else None
    jobs = []
    for tum in sorted(Path(a.runs).glob('*/*/*.tum')):
        v = tum.parent.parent.name
        if tum.stem not in methods or (variants and v not in variants):
            continue
        world, traj = tum.parent.name.split('__')
        jobs.append((v, world, traj, tum.stem, tum))
    with Pool(a.procs) as p:
        rows = p.map(one, jobs, chunksize=4)
    lines = ['\t'.join(COLS)]
    for r in rows:
        lines.append('\t'.join(f'{r[k]:.6g}' if isinstance(r.get(k), float) else str(r.get(k, ''))
                               for k in COLS))
    Path(a.output).write_text('\n'.join(lines) + '\n')
    print(f'{len(rows)} runs -> {a.output}')


if __name__ == '__main__':
    main()
