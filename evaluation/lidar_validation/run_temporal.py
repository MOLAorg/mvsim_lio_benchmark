#!/usr/bin/env python3
"""Temporal statistics of static sweeps: Husky OS0-32 (30 stationary intervals),
Newer College OS1-64 (static run), three more Ouster units of public datasets in
their still intervals (bag_still.py, bag_static_extract.py), and the benchmark's
error model simulated at the Newer College poses (low, middle and high end of its
parameter ranges).

Usage: run_temporal.py <work_dir>   (work_dir holds husky/*.npz, the NCD files and
the directories of EXTRA)
Writes temporal_summary.json.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import read_sim  # noqa: E402
from ncd_sim import CONFIGS  # noqa: E402
from ncd_static import static_run  # noqa: E402
from temporal import model_sigma, pixel_dirs_from_xyz, pool, summarize, temporal_stats  # noqa: E402,F401

# Ends of the noise parameter ranges: the current model (sigma0, k, divergence, clamp) and the
# first release's (v1_*), each evaluated per pixel from its range and incidence.
MODEL_POINTS = dict(low=(0.005, 5e-4, 0.1, 80.0), high=(0.02, 1.5e-3, 0.3, 80.0),
                    v1_low=(0.005, 0, 0.05), v1_mid=(0.0125, 1.5e-4, 0.175), v1_high=(0.02, 3e-4, 0.3))
FIT_CLAMP_DEG = 80.0
# More Ouster units: summary key -> directory of *_still*.npz in the work dir.
EXTRA = {'geode_os1_64': 'geode', 'tiers_os1_64': 'tiers_os1_64', 'tiers_os0_128': 'tiers_os0_128'}
MIN_SWEEPS = 50


def dir_stats(d):
    """Temporal statistics of every still interval of a directory with enough sweeps."""
    stats = []
    for f in sorted(Path(d).glob('*_still*.npz')):
        z = np.load(f)
        if len(z['stamp']) < MIN_SWEEPS:
            continue
        stats.append(temporal_stats(z['range'].astype(np.float64) * 1e-3,
                                    pixel_dirs_from_xyz(z['xyz'].astype(np.float64))))
    return stats


def compensate(rng, ref):
    """Remove the change of each pixel's reference range between sweeps (the sensor
    drifts slightly), where the reference changes smoothly; ranges are kept otherwise."""
    ref = ref.astype(np.float64)
    with np.errstate(all='ignore'):
        ref_med = np.nanmedian(np.where(ref > 0, ref, np.nan), axis=0)
    d = ref - ref_med[None]
    ok = (rng > 0) & (ref > 0) & np.isfinite(d) & (np.abs(d) < 0.1)
    return np.where(ok, rng - d, rng)


def fit_noise(p):
    """Least-squares fit of sigma0, k and the footprint divergence (spread clamped at
    FIT_CLAMP_DEG) to the per-pixel temporal noise of surface pixels (1-30 m), in bins of
    range and incidence."""
    from scipy.optimize import least_squares
    surf = p['geo'] & ~p['edge'] & (p['flat'] < 0.01) & (p['inl'] > 0.9) & (p['n_ret'] >= 30) & (p['med'] >= 1) & (p['med'] < 30)
    rb = np.array([1, 2, 3, 4, 6, 8, 11, 16, 22, 30])
    tb = np.array([0, 15, 30, 45, 60, 70, 75, 80, 85])
    rows = []
    for i in range(len(rb) - 1):
        for j in range(len(tb) - 1):
            m = surf & (p['med'] >= rb[i]) & (p['med'] < rb[i + 1]) & (p['theta'] >= tb[j]) & (p['theta'] < tb[j + 1])
            if m.sum() >= 50:
                rows.append((np.median(p['med'][m]), np.median(p['theta'][m]), np.median(p['sig'][m]), m.sum()))
    rows = np.array(rows)
    f = lambda x: (model_sigma(rows[:, 0], rows[:, 1], abs(x[0]), abs(x[1]), abs(x[2]), FIT_CLAMP_DEG)
                   - rows[:, 2]) / rows[:, 2]
    sol = least_squares(f, [0.01, 5e-4, 0.1])
    s0, k, dv = np.abs(sol.x)
    return dict(sigma0=float(s0), k=float(k), divergence_deg=float(dv), clamp_deg=FIT_CLAMP_DEG, bins=int(len(rows)),
                rel_rms=float(np.sqrt(np.mean(sol.fun ** 2))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('work')
    a = ap.parse_args()
    w = Path(a.work)
    res = {}
    stats = dir_stats(w / 'husky')
    res['husky_os0_32'] = summarize(pool(stats), models=MODEL_POINTS)
    res['husky_fit'] = fit_noise(pool(stats))
    res['husky_os0_32']['intervals'] = len(stats)
    S = np.load(w / 'ncd_still.npz')
    n, H, W = S['range'].shape
    run = static_run(S['stamp'])
    rng = compensate(S['range'][run].astype(np.float64) * 1e-3,
                     np.fromfile(w / 'hits.bin', dtype='<f4').reshape(n, H, W, 4)[run, ..., 0])
    ncd_short = temporal_stats(rng, np.load(w / 'dirs.npy'))
    res['ncd_os1_64'] = summarize(pool([ncd_short]), models=MODEL_POINTS)
    # The same unit also stands still at the start of 02_long (no TLS reference there).
    long_stats = dir_stats(w / 'ncd_long')
    if long_stats:
        res['ncd_os1_64_all'] = summarize(pool([ncd_short] + long_stats), models=MODEL_POINTS)
        res['ncd_os1_64_all']['intervals'] = 1 + len(long_stats)
    for key, sub in EXTRA.items():
        st = dir_stats(w / sub)
        if st:
            res[key] = summarize(pool(st), models=MODEL_POINTS)
            res[key]['intervals'] = len(st)
    alt = np.sort(np.loadtxt(w / 'sim_elev.txt'))
    el = np.radians(alt)
    az = -np.pi + np.arange(W) * (2 * np.pi / W)
    dirs_s = np.stack([np.cos(el)[:, None] * np.cos(az)[None, :], np.cos(el)[:, None] * np.sin(az)[None, :],
                       np.broadcast_to(np.sin(el)[:, None], (H, W))], -1)
    ref_s = np.fromfile(w / 'hits_sim.bin', dtype='<f4').reshape(n, H, W, 4)[run, ..., 0]
    for name in CONFIGS:
        r, _ = read_sim(w / f'sim_{name}.bin', n, H)
        r = compensate(r[run].astype(np.float64), ref_s)
        res[f'sim_{name}'] = summarize(pool([temporal_stats(r, dirs_s, observed=np.ones((len(run), W), bool))]),
                                       models=MODEL_POINTS)
    (w / 'temporal_summary.json').write_text(json.dumps(res, indent=1))
    report(res)


def fmt(x, scale=1000, nd=1):
    return '-' if x is None else f'{x * scale:.{nd}f}'


def report(res):
    cols = ['husky_os0_32', 'ncd_os1_64', 'sim_clean', 'sim_mid', 'sim_beam_d15_l25', 'sim_v2_low', 'sim_v2_high']
    print('Fitted to the Husky OS0-32 noise:', json.dumps(res['husky_fit']))
    print('\nTemporal noise within 30 deg of normal incidence, median [mm] (pixels); model low/mid/high for the same pixels')
    print('range [m] ' + ''.join(f'{c:>16}' for c in cols) + '   model on Husky pixels')
    for i, b in enumerate(res['husky_os0_32']['sigma_vs_range']):
        cells = [f'{fmt(res[c]["sigma_vs_range"][i]["sigma"])} ({res[c]["sigma_vs_range"][i]["n"]})' for c in cols]
        mdl = ' / '.join(fmt(b['model'][k]) for k in ('low', 'high', 'v1_low', 'v1_high'))
        print(f'{b["lo"]:>3}-{b["hi"]:<3}   ' + ''.join(f'{c:>16}' for c in cells) + f'   {mdl}')
    print('\nAgainst incidence, 2-20 m: return share | share of pixels always returning | noise [mm] (model low/mid/high on the same pixels)')
    for i, b in enumerate(res['husky_os0_32']['vs_incidence']):
        row = f'{b["lo"]:>2}-{b["hi"]:<2} deg'
        for c in ('husky_os0_32', 'ncd_os1_64', 'sim_beam_d15_l25', 'sim_v2_high'):
            x = res[c]['vs_incidence'][i]
            row += f' | {c[:5]} {fmt(x["p_ret"], 1, 3)} {fmt(x["frac_full"], 1, 2)} {fmt(x["sigma"])} (n={x["n"] + x["n_never"]})'
        mdl = ' / '.join(fmt(b['model'][k]) for k in ('low', 'high', 'v1_low', 'v1_high'))
        print(row + f' | Husky model {mdl}')
    print('\nDepth edges along a ring (jump > 0.5 m): pixels; share of returns between; pixels mixing (0 < between < 0.9);'
          ' per-pixel between-share histogram [0, <5%, 5-20%, 20-50%, 50-90%, >90%]')
    for c in cols:
        e = res[c]['edge']
        print(f'  {c:>13}: {e["n_pixels"]:7d}  {e["share_between"]:.3f}  {fmt(e["pixels_mixing"], 1, 3)}  {res[c]["edge_pixel_share_hist"]}')


if __name__ == '__main__':
    main()
