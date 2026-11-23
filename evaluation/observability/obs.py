#!/usr/bin/env python3
"""Geometric observability of the LiDAR sweeps along the true trajectory, method-independent.

For every clean global-shutter sweep, placed at its true pose, each point is matched to the
exact reference map (10 cm voxel centroids of all clean sweeps at the true poses) and gets the
normal of the map there. The Fisher information of point-to-plane registration of the sweep
against the exact map, with range noise sigma along the normal, is

    H = sum_i J_i^T J_i / sigma^2,   J_i = [n_i^T, ((p_i - c) x n_i)^T]

(translation in the world frame, rotation about the sensor c). The Cramer-Rao bound of the
translation, with the rotation also unknown, is the translation block of H^-1; it is reported
along the direction of travel (axial), across it (lateral, horizontal) and vertically. It is a
lower bound for any estimator that registers this sweep against a perfect map: it assumes known
correspondences and independent noise.

Two more outputs:
- Ambiguity along the axis: every --every-shift sweeps, the sweep is shifted along the direction
  of travel by -15..15 m; at each shift, the points farther than 10 cm from the map are counted.
  The minimum, over shifts beyond 0.5 m, of that count (minus the count at the true pose) is the
  number of points that tell the true position from the best alternative one.
- Axial drift bound of an ideal LiDAR-inertial odometry: a 1D Kalman filter along the axis with
  the benchmark's accelerometer (white noise and bias random walk) and, at each sweep, a
  measurement of the displacement since the previous sweep with the sweep's axial bound.

Usage: obs.py <world/traj> <data_dir> <out_dir> [--sigma 0.02] [--workers 16]
"""
import argparse
import json
import os
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

DATA = Path(os.environ.get('LIO_DATA', Path.home() / 'mvsim_lio'))
REF = Path(os.environ.get('LIO_ROOT', Path.home() / 'mvsim_lio_eval')) / 'maps' / 'ref'
REC = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('ring', '<u2'), ('label', '<u2')])
LABELS = {0: 'unlabeled', 1: 'outlier', 10: 'car', 20: 'other-vehicle', 40: 'road', 44: 'parking',
          48: 'sidewalk', 49: 'other-ground', 50: 'building', 51: 'fence',
          52: 'other-structure', 70: 'vegetation', 71: 'trunk', 72: 'terrain', 80: 'pole',
          81: 'traffic-sign', 99: 'other-object', 100: 'floor', 101: 'ceiling', 102: 'wall',
          103: 'column', 104: 'stairs', 105: 'table', 106: 'chair', 107: 'shelf',
          108: 'other-furniture', 109: 'goods'}
# Accelerometer of the noisy and real levels: noise density [m/s^2/sqrt(Hz)], bias random walk
# [m/s^2/sqrt(s)].
ACC_NOISE = 2.3e-3
ACC_BIAS_RW = 1.0e-3
SIGMA_CAP = 1e3


def read_tum(path):
    a = np.loadtxt(path, comments='#')
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def map_normals(xyz, tree, workers, k=20, chunk=400_000):
    n = np.zeros_like(xyz)
    for s in range(0, len(xyz), chunk):
        _, idx = tree.query(xyz[s:s + chunk], k=k, workers=workers)
        nb = xyz[idx]
        nb = nb - nb.mean(axis=1, keepdims=True)
        cov = np.einsum('nki,nkj->nij', nb, nb)
        _, v = np.linalg.eigh(cov)
        n[s:s + chunk] = v[:, :, 0]
    return n


def travel_dirs(stamps, gt_t, gt_p):
    """Horizontal unit direction of travel at each stamp (the last one when stopped)."""
    v = np.gradient(gt_p, gt_t, axis=0)
    vi = np.stack([np.interp(stamps, gt_t, v[:, j]) for j in range(3)], 1)
    d = np.zeros((len(stamps), 3))
    last = np.array([1.0, 0.0, 0.0])
    for k, vk in enumerate(vi):
        h = np.array([vk[0], vk[1], 0.0])
        if np.linalg.norm(h) > 0.05:
            last = h / np.linalg.norm(h)
        d[k] = last
    if not np.any(np.linalg.norm(vi[:, :2], axis=1) > 0.05):
        return d
    first = np.argmax(np.linalg.norm(vi[:, :2], axis=1) > 0.05)
    d[:first] = d[first]
    return d


def marginal_sigma(H, dirs):
    """Std of the translation along each direction, with the rotation marginalized out."""
    Htt = H[:3, :3]
    Htr = H[:3, 3:]
    Hrr = H[3:, 3:]
    It = Htt - Htr @ np.linalg.pinv(Hrr) @ Htr.T
    w, V = np.linalg.eigh(0.5 * (It + It.T))
    w = np.maximum(w, 1.0 / SIGMA_CAP ** 2)
    C = V @ np.diag(1.0 / w) @ V.T
    out = [float(np.sqrt(max(d @ C @ d, 0.0))) for d in dirs]
    return out, w, V


def drift_bound(stamps, sig_axial, sig0_v=0.01, sig0_b=0.01):
    """1D delayed-state Kalman filter: std of the axial position at each sweep."""
    qa = ACC_NOISE ** 2
    qb = ACC_BIAS_RW ** 2
    P = np.diag([0.0, sig0_v ** 2, sig0_b ** 2, 0.0])
    out = [0.0]
    for k in range(1, len(stamps)):
        dt = stamps[k] - stamps[k - 1]
        F = np.array([[1, dt, -dt * dt / 2, 0], [0, 1, -dt, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
        Q = np.zeros((4, 4))
        Q[0, 0] = qa * dt ** 3 / 3 + qb * dt ** 5 / 20
        Q[0, 1] = Q[1, 0] = qa * dt ** 2 / 2 + qb * dt ** 4 / 8
        Q[0, 2] = Q[2, 0] = -qb * dt ** 3 / 6
        Q[1, 1] = qa * dt + qb * dt ** 3 / 3
        Q[1, 2] = Q[2, 1] = -qb * dt ** 2 / 2
        Q[2, 2] = qb * dt
        P = F @ P @ F.T + Q
        r2 = min(sig_axial[k], SIGMA_CAP) ** 2
        h = np.array([1.0, 0, 0, -1.0])
        s = h @ P @ h + r2
        K = P @ h / s
        P = P - np.outer(K, h @ P)
        T = np.eye(4)
        T[3] = [1, 0, 0, 0]
        P = T @ P @ T.T
        out.append(float(np.sqrt(max(P[0, 0], 0.0))))
    return np.array(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('seq')
    ap.add_argument('data_dir')
    ap.add_argument('out_dir')
    ap.add_argument('--sigma', type=float, default=0.02)
    ap.add_argument('--workers', type=int, default=16)
    ap.add_argument('--every-shift', type=int, default=10)
    ap.add_argument('--shift-points', type=int, default=4000)
    a = ap.parse_args()
    w, t = a.seq.split('/')
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    dd = Path(a.data_dir)

    ref = np.fromfile(REF / f'{w}__{t}.bin', '<f4').reshape(-1, 4)[:, :3].astype(np.float64)
    tree = cKDTree(ref)
    normals = map_normals(ref, tree, a.workers)
    print(f'{a.seq}: map {len(ref)} voxels, normals done', flush=True)

    sw = np.loadtxt(dd / 'sweeps.txt', comments='#')
    stamps = sw[:, 1]
    counts = sw[:, 2].astype(np.int64)
    offs = np.concatenate([[0], np.cumsum(counts)])
    pts = np.memmap(dd / 'lidar.txt.bin', REC, 'r')
    assert len(pts) == offs[-1], (len(pts), offs[-1])
    lt, lp, lq = read_tum(DATA / w / t / 'os1_32-clean-global.lidar1.gt.tum')
    assert len(lt) == len(stamps) and np.allclose(lt, stamps, atol=1e-6)
    gt_t, gt_p, _ = read_tum(DATA / w / t / 'gt_200hz.tum')
    dirs = travel_dirs(stamps, gt_t, gt_p)
    rot = Rotation.from_quat(lq)
    zup = np.array([0, 0, 1.0])

    rows = []
    label_info = {}
    shift_rows = []
    shifts = np.round(np.arange(-15.0, 15.0001, 0.1), 3)
    rng = np.random.default_rng(0)
    for k in range(len(stamps)):
        p = pts[offs[k]:offs[k + 1]]
        ps = np.stack([p['x'], p['y'], p['z']], 1).astype(np.float64)
        R = rot[k].as_matrix()
        c = lp[k]
        pw = ps @ R.T + c
        dist, idx = tree.query(pw, distance_upper_bound=0.15, workers=a.workers)
        ok = np.isfinite(dist)
        n = normals[idx[ok]]
        q = pw[ok]
        J = np.hstack([n, np.cross(q - c, n)])
        H = J.T @ J / a.sigma ** 2
        d = dirs[k]
        lat = np.cross(zup, d)
        (s_ax, s_lat, s_v), wv, V = marginal_sigma(H, [d, lat, zup])
        weakest = V[:, 0]
        rows.append((stamps[k], *c, *d, int(ok.sum()), s_ax, s_lat, s_v,
                     float(np.sqrt(1.0 / wv[0])), float(abs(weakest @ d))))
        lab = p['label'][ok]
        proj = (n @ d) ** 2
        for L in np.unique(lab):
            label_info[int(L)] = label_info.get(int(L), 0.0) + float(proj[lab == L].sum())
        if k % a.every_shift == 0:
            sel = rng.choice(len(pw), min(a.shift_points, len(pw)), replace=False)
            base = pw[sel]
            mis = []
            for s in shifts:
                dd_, _ = tree.query(base + s * d, distance_upper_bound=0.3, workers=a.workers)
                mis.append(int((dd_ > 0.1).sum()))
            mis = np.array(mis)
            scale = len(pw) / len(sel)
            far = np.abs(shifts) >= 0.5
            j = np.argmin(np.where(far, mis, 1 << 30))
            m0 = mis[np.argmin(np.abs(shifts))]
            shift_rows.append((stamps[k], float(shifts[j]), float((mis[j] - m0) * scale),
                               float((mis[np.argmin(np.abs(shifts - 1.0))] - m0) * scale)))
        if k % 500 == 0:
            print(f'  sweep {k}/{len(stamps)}', flush=True)

    cols = ['stamp', 'x', 'y', 'z', 'dx', 'dy', 'dz', 'n_matched', 'sig_axial', 'sig_lateral',
            'sig_vertical', 'sig_weakest', 'weakest_dot_axial']
    A = np.array(rows)
    np.savetxt(out / 'sweeps.tsv', A, delimiter='\t', header='\t'.join(cols), comments='',
               fmt='%.6g')
    np.savetxt(out / 'shifts.tsv', np.array(shift_rows), delimiter='\t',
               header='stamp\tbest_alt_shift\tdiscriminating_pts\tdiscriminating_pts_at_1m',
               comments='', fmt='%.6g')
    drift = drift_bound(stamps, A[:, 8])
    np.savetxt(out / 'drift.tsv', np.stack([stamps, drift], 1), delimiter='\t',
               header='stamp\tsig_axial_lio', comments='', fmt='%.6g')
    s_ax = A[:, 8]
    tot = sum(label_info.values())
    S = np.array(shift_rows)
    summary = {
        'seq': a.seq, 'sweeps': len(stamps), 'sigma_range_m': a.sigma,
        'sig_axial_pct': {str(q): float(np.percentile(s_ax, q)) for q in (5, 25, 50, 75, 95, 99)},
        'sig_axial_max': float(s_ax.max()),
        'sig_lateral_median': float(np.median(A[:, 9])),
        'sig_vertical_median': float(np.median(A[:, 10])),
        'share_sweeps_axial_over': {str(th): float((s_ax > th).mean())
                                    for th in (0.001, 0.01, 0.1, 1.0)},
        'weakest_dir_is_axial_share': float((A[:, 12] > 0.9).mean()),
        'axial_info_share_by_label': {LABELS.get(L, str(L)): v / tot
                                      for L, v in sorted(label_info.items(), key=lambda x: -x[1])},
        'ambiguity': {
            'sweeps_checked': len(S),
            'discriminating_pts_pct': {str(q): float(np.percentile(S[:, 2], q))
                                       for q in (0, 5, 25, 50)},
            'share_below_100_pts': float((S[:, 2] < 100).mean()),
            'share_below_20_pts': float((S[:, 2] < 20).mean()),
        },
        'lio_axial_drift_end': float(drift[-1]),
        'lio_axial_drift_max': float(drift.max()),
        'lo_axial_random_walk_end': float(np.sqrt(np.sum(np.minimum(s_ax, SIGMA_CAP) ** 2))),
    }
    (out / 'summary.json').write_text(json.dumps(summary, indent=1) + '\n')
    print(json.dumps(summary, indent=1), flush=True)


if __name__ == '__main__':
    main()
