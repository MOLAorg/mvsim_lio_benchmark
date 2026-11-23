#!/usr/bin/env python3
"""Axial information by semantic label, tunnel sweeps in the straight sections far from portals.

Usage: interior_labels.py <world/traj> <data_dir> <obs_out_dir> [--every 10]
"""
import argparse
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

import tunnel_profile as tp
from obs import DATA, LABELS, REC, REF, map_normals, read_tum


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('seq')
    ap.add_argument('data_dir')
    ap.add_argument('obs_dir')
    ap.add_argument('--every', type=int, default=10)
    ap.add_argument('--workers', type=int, default=7)
    a = ap.parse_args()
    w, t = a.seq.split('/')
    dd = Path(a.data_dir)
    ref = np.fromfile(REF / f'{w}__{t}.bin', '<f4').reshape(-1, 4)[:, :3].astype(np.float64)
    tree = cKDTree(ref)
    normals = map_normals(ref, tree, a.workers)
    sw = np.loadtxt(dd / 'sweeps.txt', comments='#')
    offs = np.concatenate([[0], np.cumsum(sw[:, 2].astype(np.int64))])
    pts = np.memmap(dd / 'lidar.txt.bin', REC, 'r')
    lt, lp, lq = read_tum(DATA / w / t / 'os1_32-clean-global.lidar1.gt.tum')
    rot = Rotation.from_quat(lq)
    A = np.genfromtxt(Path(a.obs_dir) / 'sweeps.tsv', names=True, delimiter='\t')
    dirs = np.stack([A['dx'], A['dy'], A['dz']], 1)
    ss = np.arange(0.0, tp.LENGTH, 0.25)
    cl = np.array([tp.centerline(s) for s in ss])
    info = {}
    npts = {}
    n_sw = 0
    for k in range(0, len(lt), a.every):
        j = np.argmin(np.hypot(cl[:, 0] - lp[k, 0], cl[:, 1] - lp[k, 1]))
        s = ss[j]
        straight = (70 < s < tp.L1) or (tp.L1 + tp.R * tp.ANG + 70 < s < tp.LENGTH - 70)
        if not straight:
            continue
        n_sw += 1
        p = pts[offs[k]:offs[k + 1]]
        ps = np.stack([p['x'], p['y'], p['z']], 1).astype(np.float64)
        pw = ps @ rot[k].as_matrix().T + lp[k]
        dist, idx = tree.query(pw, distance_upper_bound=0.15, workers=a.workers)
        ok = np.isfinite(dist)
        proj = (normals[idx[ok]] @ dirs[k]) ** 2
        lab = p['label'][ok]
        for L in np.unique(lab):
            m = lab == L
            info[int(L)] = info.get(int(L), 0.0) + float(proj[m].sum())
            npts[int(L)] = npts.get(int(L), 0) + int(m.sum())
    tot = sum(info.values())
    allp = sum(npts.values())
    print(f'{a.seq}: {n_sw} straight interior sweeps')
    for L, v in sorted(info.items(), key=lambda x: -x[1]):
        print(f'  {LABELS.get(L, L):14s} axial info {100 * v / tot:5.1f} %   points {100 * npts[L] / allp:5.2f} %')


if __name__ == '__main__':
    main()
