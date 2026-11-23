#!/usr/bin/env python3
"""Points that a shift along the direction of travel moves off the exact map, all points.

For every --every-th clean sweep at its true pose: the number of points farther than 10 cm from
the reference map when the sweep is shifted by s along the direction of travel, minus that
number at s = 0, for s in SHIFTS. Uses the outputs of obs.py (directions) and its data dir.

Usage: shift_test.py <world/traj> <data_dir> <obs_out_dir> [--every 5] [--workers 7]
"""
import argparse
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from obs import DATA, REC, REF, read_tum

SHIFTS = [0.0, -20, -10, -5, -2, -1, -0.5, 0.5, 1, 2, 5, 10, 20]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('seq')
    ap.add_argument('data_dir')
    ap.add_argument('obs_dir')
    ap.add_argument('--every', type=int, default=5)
    ap.add_argument('--workers', type=int, default=7)
    a = ap.parse_args()
    w, t = a.seq.split('/')
    dd = Path(a.data_dir)
    ref = np.fromfile(REF / f'{w}__{t}.bin', '<f4').reshape(-1, 4)[:, :3].astype(np.float64)
    tree = cKDTree(ref)
    sw = np.loadtxt(dd / 'sweeps.txt', comments='#')
    offs = np.concatenate([[0], np.cumsum(sw[:, 2].astype(np.int64))])
    pts = np.memmap(dd / 'lidar.txt.bin', REC, 'r')
    lt, lp, lq = read_tum(DATA / w / t / 'os1_32-clean-global.lidar1.gt.tum')
    rot = Rotation.from_quat(lq)
    A = np.genfromtxt(Path(a.obs_dir) / 'sweeps.tsv', names=True, delimiter='\t')
    dirs = np.stack([A['dx'], A['dy'], A['dz']], 1)
    rows = []
    for k in range(0, len(lt), a.every):
        p = pts[offs[k]:offs[k + 1]]
        ps = np.stack([p['x'], p['y'], p['z']], 1).astype(np.float64)
        pw = ps @ rot[k].as_matrix().T + lp[k]
        off = []
        for s in SHIFTS:
            d, _ = tree.query(pw + s * dirs[k], distance_upper_bound=0.3, workers=a.workers)
            off.append(int((d > 0.1).sum()))
        rows.append([lt[k], len(pw)] + [o - off[0] for o in off[1:]])
    hdr = 'stamp\tn_points\t' + '\t'.join(f'off_{s:g}' for s in SHIFTS[1:])
    np.savetxt(Path(a.obs_dir) / 'shifts_all.tsv', np.array(rows), delimiter='\t', header=hdr,
               comments='', fmt='%.6g')


if __name__ == '__main__':
    main()
