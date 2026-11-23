#!/usr/bin/env python3
"""Scores estimated trajectories against the exact ground truth.

The ground truth is evaluated at each estimate's own timestamps from the same
C2 spline the generator used (worlds/<w>/trajectories/<t>.tum), so methods
that stamp a scan at its start, middle or end are all scored exactly. A
per-method constant time offset (stamp convention) can be applied.

Metrics (translation of the reported frame, base_link unless stated):
  ape        RMSE of the position error after SE(3) Umeyama alignment [m]
  ape_rot    RMSE of the rotation error after that alignment [deg]
  drift_pct  end-point error aligning only the first pose, % of the GT length
  rte10/50   mean relative translation error over 10/50 m segments [%]
  rre10      mean relative rotation error over 10 m segments [deg/m]
  coverage   fraction of the sequence duration covered by the estimate
  len_ratio  estimated / GT path length over the covered interval, both measured on
             the poses at every LEN_STEP meters of the GT path (pose jitter does not
             count as distance)
  cpu_s      user + sys CPU time [s];  rt_factor = sequence duration / cpu_s

Usage: eval_traj.py --runs <dir> [--offsets offsets.json] [--frame-check] -o results.tsv
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from mvsim_lio import densify  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_lio import limit_kill  # noqa: E402

BENCH = Path(__file__).resolve().parents[1]
T_BASE_LIDAR = np.eye(4)
T_BASE_LIDAR[2, 3] = 0.75
LEN_STEP = 1.0  # [m] pose spacing for the path length ratio

_splines = {}


def gt_spline(world, traj):
    key = (world, traj)
    if key not in _splines:
        _splines[key] = densify.spline(BENCH / 'worlds' / world / 'trajectories' / f'{traj}.tum')
    return _splines[key]


def to_T(p, q):
    T = np.tile(np.eye(4), (len(p), 1, 1))
    T[:, :3, :3] = Rotation.from_quat(q).as_matrix()
    T[:, :3, 3] = p
    return T


def load_tum(path):
    d = np.loadtxt(path, comments='#', ndmin=2)
    if d.size == 0:
        return np.zeros(0), np.zeros((0, 4, 4))
    d = d[np.argsort(d[:, 0])]
    _, idx = np.unique(d[:, 0], return_index=True)
    d = d[idx]
    return d[:, 0], to_T(d[:, 1:4], d[:, 4:8])


def inv(T):
    Ti = np.tile(np.eye(4), (len(T), 1, 1))
    R = T[:, :3, :3].transpose(0, 2, 1)
    Ti[:, :3, :3] = R
    Ti[:, :3, 3] = -np.einsum('nij,nj->ni', R, T[:, :3, 3])
    return Ti


def umeyama(src, dst):
    """Rigid transform (no scale) that best maps src points onto dst points."""
    ms, md = src.mean(0), dst.mean(0)
    C = (dst - md).T @ (src - ms) / len(src)
    U, _, Vt = np.linalg.svd(C)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = md - R @ ms
    return T


def rot_angle_deg(R):
    c = (np.trace(R, axis1=-2, axis2=-1) - 1) / 2
    return np.degrees(np.arccos(np.clip(c, -1, 1)))


def rpe(G, E, dist, seg, step=1.0):
    """Mean relative translation [%] and rotation [deg/m] errors over segments."""
    starts = np.searchsorted(dist, np.arange(0, dist[-1] - seg, step))
    ends = np.searchsorted(dist, dist[starts] + seg)
    ok = ends < len(dist)
    starts, ends = starts[ok], ends[ok]
    if len(starts) == 0:
        return float('nan'), float('nan')
    Dg = inv(G[starts]) @ G[ends]
    De = inv(E[starts]) @ E[ends]
    Err = inv(Dg) @ De
    L = dist[ends] - dist[starts]
    te = np.linalg.norm(Err[:, :3, 3], axis=1) / L * 100
    re = rot_angle_deg(Err[:, :3, :3]) / L
    return float(te.mean()), float(re.mean())


def score(world, traj, est_path, offset=0.0, frame='base'):
    t, E = load_tum(est_path)
    t0, pos, rot = gt_spline(world, traj)
    seq_dur = t0[-1] - t0[0]
    out = {'n_poses': int(len(t))}
    if len(t) < 10:
        out.update(coverage=0.0, status='no_output')
        return out
    t = t + offset
    ok = (t >= t0[0]) & (t <= t0[-1])
    t, E = t[ok], E[ok]
    G = to_T(pos(t), rot(t).as_quat())
    if frame == 'lidar':
        G = G @ T_BASE_LIDAR
    out['coverage'] = float((t[-1] - t[0]) / seq_dur)
    seg = np.linalg.norm(np.diff(G[:, :3, 3], axis=0), axis=1)
    dist = np.concatenate([[0], np.cumsum(seg)])
    out['length_m'] = float(dist[-1])
    # Poses every LEN_STEP meters of the GT path, so that high-frequency jitter
    # of an otherwise accurate estimate does not inflate its path length.
    sub = np.unique(np.searchsorted(dist, np.arange(0.0, dist[-1], LEN_STEP)))
    sub = np.append(sub, len(dist) - 1) if sub[-1] != len(dist) - 1 else sub
    len_est = np.linalg.norm(np.diff(E[sub, :3, 3], axis=0), axis=1).sum()
    len_gt = np.linalg.norm(np.diff(G[sub, :3, 3], axis=0), axis=1).sum()
    out['len_ratio'] = float(len_est / max(len_gt, 1e-9))
    A = umeyama(E[:, :3, 3], G[:, :3, 3])
    Ea = A @ E
    err = Ea[:, :3, 3] - G[:, :3, 3]
    out['ape'] = float(np.sqrt((err ** 2).sum(1).mean()))
    out['ape_max'] = float(np.linalg.norm(err, axis=1).max())
    out['ape_z'] = float(np.sqrt((err[:, 2] ** 2).mean()))
    rerr = rot_angle_deg(np.einsum('nji,njk->nik', G[:, :3, :3], Ea[:, :3, :3]))
    out['ape_rot'] = float(np.sqrt((rerr ** 2).mean()))
    E0 = (G[0] @ np.linalg.inv(E[0])) @ E
    out['drift_pct'] = float(100 * np.linalg.norm(E0[-1, :3, 3] - G[-1, :3, 3])
                             / max(dist[-1], 1e-9))
    out['rte10'], out['rre10'] = rpe(G, E, dist, 10.0)
    out['rte50'], _ = rpe(G, E, dist, 50.0)
    return out


def classify(r):
    if r.get('status') == 'no_output':
        return 'no_output'
    if r['coverage'] < 0.9:
        return 'incomplete'
    if not (0.8 < r['len_ratio'] < 1.25) or r['ape'] > 0.1 * r['length_m']:
        return 'diverged'
    return 'ok'


def read_time(path):
    out = {}
    if path.exists():
        for tok in path.read_text().split():
            if '=' in tok:
                k, v = tok.split('=')
                out[k] = float(v)
    return out


COLS = ['variant', 'world', 'traj', 'method', 'status', 'coverage', 'length_m', 'len_ratio',
        'ape', 'ape_max', 'ape_z', 'ape_rot', 'drift_pct', 'rte10', 'rte50', 'rre10', 'n_poses',
        'cpu_s', 'wall_s', 'maxrss_mb', 'rt_factor', 'exit']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', required=True)
    ap.add_argument('--offsets', default=str(Path(__file__).parent / 'offsets.json'))
    ap.add_argument('--frame-check', action='store_true',
                    help='also score against the LiDAR frame and print both')
    ap.add_argument('-o', '--output', default='')
    a = ap.parse_args()
    cfg = json.loads(Path(a.offsets).read_text()) if Path(a.offsets).exists() else {}
    rows = []
    # Every attempted job leaves a .time file; one without a trajectory is a
    # failure (crash, or killed by the runner's memory or time limit).
    for tf in sorted(Path(a.runs).glob('*/*/*.time')):
        if tf.with_suffix('.tum').exists() or tf.stat().st_size == 0:
            continue  # scored below, or still running
        if 'signal 15' in tf.read_text():
            continue  # stopped on purpose by the operator, to be re-run
        variant = tf.parent.parent.name
        world, traj = tf.parent.name.split('__')
        kill = limit_kill(tf.with_suffix('.tum'))
        status = {'memory': 'memory_limit', 'time': 'timeout'}.get(kill, 'no_output')
        rows.append({'variant': variant, 'world': world, 'traj': traj, 'method': tf.stem,
                     'status': status, 'coverage': 0.0})
    for tum in sorted(Path(a.runs).glob('*/*/*.tum')):
        variant = tum.parent.parent.name
        world, traj = tum.parent.name.split('__')
        method = tum.stem
        c = cfg.get(method, {})
        r = score(world, traj, tum, c.get('offset', 0.0), c.get('frame', 'base'))
        if a.frame_check:
            other = 'lidar' if c.get('frame', 'base') == 'base' else 'base'
            r2 = score(world, traj, tum, c.get('offset', 0.0), other)
            print(f'{variant} {world}/{traj} {method}: ape {c.get("frame", "base")}='
                  f'{r.get("ape", float("nan")):.4f} {other}={r2.get("ape", float("nan")):.4f}')
        r['status'] = classify(r)
        tm = read_time(tum.with_suffix('.time'))
        t0, _, _ = gt_spline(world, traj)
        cpu = tm.get('user', np.nan) + tm.get('sys', np.nan)
        r.update(variant=variant, world=world, traj=traj, method=method, cpu_s=cpu,
                 wall_s=tm.get('wall', np.nan), maxrss_mb=tm.get('maxrss_kb', np.nan) / 1024,
                 rt_factor=(t0[-1] - t0[0]) / cpu if cpu > 0 else np.nan,
                 exit=tm.get('exit', np.nan))
        rows.append(r)
    lines = ['\t'.join(COLS)]
    for r in rows:
        vals = []
        for k in COLS:
            v = r.get(k, '')
            vals.append(f'{v:.6g}' if isinstance(v, float) else str(v))
        lines.append('\t'.join(vals))
    txt = '\n'.join(lines) + '\n'
    if a.output:
        Path(a.output).write_text(txt)
    else:
        print(txt)


if __name__ == '__main__':
    main()
