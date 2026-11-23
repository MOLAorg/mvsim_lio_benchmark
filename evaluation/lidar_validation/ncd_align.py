#!/usr/bin/env python3
"""Per-sweep pose refinement of the Newer College sweeps against the TLS cloud, and the
rays of every pixel for the reference cast.

The ground truth of the 2020 collection is a 135 deg yaw off the os1_lidar frame
(measured locally; see the dataset notes). Each sweep is de-skewed by linear
interpolation of the ground truth at the per-point times, then refined by
point-to-plane ICP against the 1 cm TLS cloud with normals.

Usage: ncd_align.py <ncd_still.npz> <gt.tum> <tls_cloud.ply> <out_dir>
Writes: poses.npz (refined lidar poses at each column time), dirs.npy (unit
pixel directions in the lidar frame), rays.bin (for lidar_tool cast).
"""
import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

sys.path.insert(0, str(Path(__file__).resolve().parent))
from prep_mesh import PLY_TYPES  # noqa: E402,F401

YAW_OFFSET_DEG = -135.0


def read_cloud(path):
    with open(path, 'rb') as fh:
        props = []
        count = 0
        while True:
            line = fh.readline().decode('ascii').strip()
            if line.startswith('element vertex'):
                count = int(line.split()[2])
            elif line.startswith('property'):
                _, t, name = line.split()
                props.append((name, '<' + PLY_TYPES[t]))
            elif line == 'end_header':
                break
        v = np.fromfile(fh, dtype=np.dtype(props), count=count)
    P = np.stack([v['x'], v['y'], v['z']], 1).astype(np.float64)
    N = np.stack([v['nx'], v['ny'], v['nz']], 1).astype(np.float64)
    return P, N


def to_T(p, q):
    T = np.tile(np.eye(4), (len(p), 1, 1))
    T[:, :3, :3] = R.from_quat(q).as_matrix()
    T[:, :3, 3] = p
    return T


class GtInterp:
    def __init__(self, tum, body_to_lidar):
        g = np.loadtxt(tum)
        self.t = g[:, 0]
        self.p = g[:, 1:4]
        self.slerp = Slerp(self.t, R.from_quat(g[:, 4:8]))
        self.X = body_to_lidar

    def __call__(self, ts):
        p = np.stack([np.interp(ts, self.t, self.p[:, i]) for i in range(3)], 1)
        T = to_T(p, self.slerp(ts).as_quat())
        return T @ self.X


def pixel_dirs(xyz, rng):
    """Unit directions of every (ring, column) pixel in the lidar frame.

    Fitted from the returns: per ring a constant elevation and an azimuth that
    decreases linearly with the column; the fit is checked against the returns.
    """
    H, W = rng.shape[1:]
    ok = rng > 0
    el = np.zeros(H)
    a0 = np.zeros(H)
    cols = np.arange(W)
    dirs = np.zeros((H, W, 3))
    worst = 0.0
    for r in range(H):
        m = ok[:, r, :]
        v = xyz[:, r, :, :][m]
        c = np.broadcast_to(cols, m.shape)[m]
        n = np.linalg.norm(v, axis=1)
        el[r] = np.median(np.arcsin(v[:, 2] / n))
        az = np.arctan2(v[:, 1], v[:, 0]) + 2 * np.pi * c / W
        a0[r] = np.angle(np.mean(np.exp(1j * az)))
        azc = a0[r] - 2 * np.pi * cols / W
        dirs[r] = np.stack([np.cos(el[r]) * np.cos(azc), np.cos(el[r]) * np.sin(azc),
                            np.full(W, np.sin(el[r]))], 1)
        dev = np.degrees(np.arccos(np.clip(np.sum(dirs[r][c] * v / n[:, None], 1), -1, 1)))
        worst = max(worst, np.percentile(dev, 99.9))
    print(f'pixel directions: 99.9th percentile deviation from the returns {worst:.4f} deg')
    return dirs


def icp(P, tree, Q, N, iters=15, trims=(0.5, 0.2, 0.1, 0.05)):
    """Left correction D (map frame) that aligns points P to the cloud (point-to-plane)."""
    D = np.eye(4)
    for it in range(iters):
        X = P @ D[:3, :3].T + D[:3, 3]
        d, j = tree.query(X, k=1, workers=8)
        trim = trims[min(it // 4, len(trims) - 1)]
        n = N[j]
        e = np.sum((X - Q[j]) * n, axis=1)
        m = (d < trim) & (np.abs(e) < trim)
        A = np.hstack([np.cross(X[m], n[m]), n[m]])
        b = -e[m]
        x = np.linalg.lstsq(A, b, rcond=None)[0]
        dT = np.eye(4)
        dT[:3, :3] = R.from_rotvec(x[:3]).as_matrix()
        dT[:3, 3] = x[3:]
        D = dT @ D
    X = P @ D[:3, :3].T + D[:3, 3]
    d, j = tree.query(X, k=1, workers=8)
    e = np.abs(np.sum((X - Q[j]) * N[j], axis=1))
    m = d < 0.05
    return D, np.median(e[m]), m.mean()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scans')
    ap.add_argument('gt')
    ap.add_argument('cloud')
    ap.add_argument('out')
    a = ap.parse_args()
    out = Path(a.out)
    S = np.load(a.scans)
    rng = S['range'].astype(np.float64) * 1e-3
    xyz = S['xyz'].astype(np.float64)
    tns = S['tns'].astype(np.float64) * 1e-9
    stamps = S['stamp']
    n, H, W = rng.shape
    print(f'{n} sweeps of {H} x {W}', flush=True)
    dirs = pixel_dirs(xyz, rng)
    # Column times: the median per-point time of each column.
    tcol = np.median(tns.reshape(-1, W), axis=0)
    print(f'column times: {tcol[0]:.4f} .. {tcol[-1]:.4f} s')
    Q, N = read_cloud(a.cloud)
    lo = np.array([-25.0, -80.0, -5.0])
    hi = np.array([40.0, 25.0, 30.0])
    keep = np.all((Q > lo) & (Q < hi), axis=1)
    Q = Q[keep]
    N = N[keep]
    print(f'TLS cloud: {len(Q)} points', flush=True)
    tree = cKDTree(Q)
    # The body-to-lidar yaw: check its sign on the first sweep.
    choice = None
    for sign in (1, -1):
        X = np.eye(4)
        X[:3, :3] = R.from_euler('z', sign * YAW_OFFSET_DEG, degrees=True).as_matrix()
        gi = GtInterp(a.gt, X)
        T = gi(stamps[:1] + tcol.mean())[0]
        m = rng[0] > 0.5
        P = xyz[0][m] @ T[:3, :3].T + T[:3, 3]
        d, _ = tree.query(P[::7], k=1, workers=8)
        print(f'yaw {sign * YAW_OFFSET_DEG:+.0f} deg: median distance to TLS {np.median(d):.3f} m')
        if choice is None or np.median(d) < choice[0]:
            choice = (np.median(d), X)
    gi = GtInterp(a.gt, choice[1])
    poses = np.zeros((n, W, 4, 4))
    stats = []
    for i in range(n):
        Tc = gi(stamps[i] + tcol)
        m = (rng[i] > 1.0) & (rng[i] < 40.0)
        rr, cc = np.nonzero(m)
        sub = slice(None, None, 3)
        rr = rr[sub]
        cc = cc[sub]
        Tp = Tc[cc]
        P = np.einsum('nij,nj->ni', Tp[:, :3, :3], xyz[i, rr, cc]) + Tp[:, :3, 3]
        D, med, frac = icp(P, tree, Q, N)
        poses[i] = D @ Tc
        ang = np.degrees(R.from_matrix(D[:3, :3]).magnitude())
        stats.append((np.linalg.norm(D[:3, 3]), ang, med, frac))
        if i % 20 == 0:
            print(f'  sweep {i}: correction {stats[-1][0] * 100:.1f} cm {ang:.3f} deg, '
                  f'median |point-to-plane| {med * 1000:.1f} mm, inliers {frac:.0%}', flush=True)
    st = np.array(stats)
    print(f'corrections: median {np.median(st[:, 0]) * 100:.1f} cm / {np.median(st[:, 1]):.3f} deg, '
          f'max {st[:, 0].max() * 100:.1f} cm / {st[:, 1].max():.3f} deg; '
          f'median residual {np.median(st[:, 2]) * 1000:.1f} mm')
    np.savez(out / 'poses.npz', poses=poses, stamps=stamps, tcol=tcol, icp=st)
    np.save(out / 'dirs.npy', dirs)
    # Rays of every pixel, origin at the lidar origin at its column time.
    org = poses[:, None, :, :3, 3].repeat(H, axis=1)
    dw = np.einsum('ncij,rcj->nrci', poses[:, :, :3, :3], dirs)
    rays = np.concatenate([org, dw], axis=-1).reshape(-1, 6)
    rays.astype('<f8').tofile(out / 'rays.bin')
    print(f'{len(rays)} rays -> {out / "rays.bin"}')


if __name__ == '__main__':
    main()
