#!/usr/bin/env python3
"""Map quality of an estimated point-cloud map against the exact reference map.

Both maps are bench-map-export outputs (float32 x y z t records, voxel
centroids). Distances are nearest-neighbor distances between voxel centroids,
so the reference resolution sets the floor of what can be measured (about a
third of the voxel size for a well-sampled surface).

  acc_mean / acc_med / acc_p95   estimate -> reference distances [m] (accuracy)
  comp_mean                      reference -> estimate distances [m] (completeness)
  chamfer                        (acc_mean + comp_mean) / 2 [m]
  prec_X / rec_X / f_X           fraction of estimate / reference points within
                                 X m of the other map, and their F-score

Usage: map_metrics.py est.bin ref.bin [--tau 0.1,0.2] [--max-dist 2.0]
"""
import argparse
import json

import numpy as np
from scipy.spatial import cKDTree


def load(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)[:, :3].astype(np.float64)


def metrics(est, ref, taus=(0.1, 0.2), max_dist=2.0, workers=1):
    d_er, _ = cKDTree(ref).query(est, k=1, distance_upper_bound=max_dist, workers=workers)
    d_re, _ = cKDTree(est).query(ref, k=1, distance_upper_bound=max_dist, workers=workers)
    d_er = np.minimum(d_er, max_dist)
    d_re = np.minimum(d_re, max_dist)
    out = {'n_est': int(len(est)), 'n_ref': int(len(ref)),
           'acc_mean': float(d_er.mean()), 'acc_med': float(np.median(d_er)),
           'acc_p95': float(np.percentile(d_er, 95)), 'comp_mean': float(d_re.mean())}
    out['chamfer'] = (out['acc_mean'] + out['comp_mean']) / 2
    for tau in taus:
        p = float((d_er < tau).mean())
        r = float((d_re < tau).mean())
        k = f'{tau:g}'
        out[f'prec_{k}'] = p
        out[f'rec_{k}'] = r
        out[f'f_{k}'] = 2 * p * r / (p + r) if p + r > 0 else 0.0
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('est')
    ap.add_argument('ref')
    ap.add_argument('--tau', default='0.1,0.2')
    ap.add_argument('--max-dist', type=float, default=2.0)
    a = ap.parse_args()
    r = metrics(load(a.est), load(a.ref), [float(x) for x in a.tau.split(',')], a.max_dist)
    print(json.dumps({k: (round(v, 5) if isinstance(v, float) else v) for k, v in r.items()}))


if __name__ == '__main__':
    main()
