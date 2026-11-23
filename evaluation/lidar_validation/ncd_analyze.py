#!/usr/bin/env python3
"""Real against simulated OS1-64 sweeps at the same poses: per-pixel range residuals
against the TLS reference, as functions of range, incidence, depth edges and ring.

Usage: ncd_analyze.py <work_dir> <lidar_tool>
Writes summary.json in the work dir and prints the comparison.
"""
import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import dump, pixel_table, read_sim, sim_geometry, summarize  # noqa: E402
from ncd_sim import CONFIGS  # noqa: E402


def cast(tool, work, org, dirs_w, name):
    n, H, W, _ = dirs_w.shape
    o = np.broadcast_to(org[:, None, :, :], (n, H, W, 3))
    np.concatenate([o, dirs_w], axis=-1).reshape(-1, 6).astype('<f8').tofile(work / f'rays_{name}.bin')
    subprocess.run([tool, 'cast', str(work / 'quad_mesh_clean.ply'), str(work / f'rays_{name}.bin'),
                    str(work / f'hits_{name}.bin'), '8'], check=True, stderr=subprocess.DEVNULL)
    h = np.fromfile(work / f'hits_{name}.bin', dtype='<f4').reshape(n, H, W, 4)
    (work / f'rays_{name}.bin').unlink()
    return h[..., 0].astype(np.float64), h[..., 1:].astype(np.float64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('work')
    ap.add_argument('tool')
    a = ap.parse_args()
    w = Path(a.work)
    S = np.load(w / 'ncd_still.npz')
    meas = S['range'].astype(np.float64) * 1e-3
    n, H, W = meas.shape
    chk = np.abs(meas - np.linalg.norm(S['xyz'], axis=-1))[meas > 0]
    print(f'range field against |xyz|: median {np.median(chk) * 1000:.2f} mm')
    P = np.load(w / 'poses.npz')
    org = P['poses'][:, :, :3, 3]
    dirs = np.load(w / 'dirs.npy')
    dirs_w = np.einsum('ncij,rcj->nrci', P['poses'][:, :, :3, :3], dirs)
    h = np.fromfile(w / 'hits.bin', dtype='<f4').reshape(n, H, W, 4)
    ref = h[..., 0].astype(np.float64)
    nrm = h[..., 1:].astype(np.float64)
    summary = {}
    t = pixel_table(meas, ref, nrm, dirs_w, org)
    summary['real'] = summarize(t)
    del t, h, dirs_w
    org_s, dirs_s = sim_geometry(w, H)
    ref_s, nrm_s = cast(a.tool, w, org_s, dirs_s, 'sim')
    for name in CONFIGS:
        rng, lab = read_sim(w / f'sim_{name}.bin', n, H)
        if name == 'clean':
            both = (rng > 0) & (ref_s > 0)
            d = np.abs(rng - ref_s)[both]
            print(f'noise-free simulation against the cast: {both.sum()} pixels, '
                  f'99.9th percentile {np.percentile(d, 99.9) * 1000:.2f} mm, '
                  f'returns without a cast hit {int(((rng > 0) & (ref_s == 0)).sum())}')
        t = pixel_table(rng, ref_s, nrm_s, dirs_s, org_s)
        summary[name] = summarize(t)
        summary[name]['frac_mixed_label'] = float((lab[ref_s > 0] == 1).mean())
    dump(summary, w / 'summary.json')
    names = ['real'] + list(CONFIGS)
    print('\nRobust std of the range residual [mm], surfaces within 30 deg of normal incidence')
    print('range [m]   ' + ''.join(f'{k:>16}' for k in names))
    for i, b in enumerate(summary['real']['sigma_vs_range']):
        print(f'{b["lo"]:>3}-{b["hi"]:<3}     ' + ''.join(
            f'{summary[k]["sigma_vs_range"][i]["sigma"] * 1000:9.1f} ({summary[k]["sigma_vs_range"][i]["median"] * 1000:+5.1f})'
            if summary[k]['sigma_vs_range'][i]['n'] >= 30 else f'{"-":>16}' for k in names))
    print('\nAgainst incidence (2-30 m): return probability / robust std [mm] / median [mm]')
    for i, b in enumerate(summary['real']['vs_incidence']):
        print(f'{b["lo"]:>2}-{b["hi"]:<2} deg  ' + ''.join(
            f'{summary[k]["vs_incidence"][i]["p_return"]:6.2f}/{(summary[k]["vs_incidence"][i]["sigma"] or np.nan) * 1000:5.1f}/{(summary[k]["vs_incidence"][i]["median"] or np.nan) * 1000:+5.1f}'
            for k in names))
    print('\nDepth edges (jump > 0.5 m): pixels, share between the surfaces, share at the other surface')
    for k in names:
        e = summary[k]['edge']
        print(f'  {k:>6}: {e["n_big"]:8d}  between {e["frac_between"]:.3f}  other {e["frac_other"]:.3f}')
    print('\nPer-ring median residual: std over rings [mm]')
    for k in names:
        print(f'  {k:>6}: {summary[k]["ring_bias"]["std"] * 1000:.2f} (spread {summary[k]["ring_bias"]["spread"] * 1000:.1f})')
    print('\nOverall: return probability, inliers (|e| < 0.3 m), outliers')
    for k in names:
        o = summary[k]['overall']
        print(f'  {k:>6}: {o["p_return"]:.3f} {o["p_inlier"]:.3f} {o["p_outlier"]:.3f}')


if __name__ == '__main__':
    main()
