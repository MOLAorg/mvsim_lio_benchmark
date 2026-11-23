#!/usr/bin/env python3
"""Per-pixel temporal statistics of a static run of Newer College sweeps (the device
still for 15.6 s at the start of 01_short) against the same statistics of simulated
sweeps at the same poses.

For every pixel, over the sweeps of the run: the error e = measured - reference
range (reference: ray cast into the TLS mesh at the refined pose of that sweep),
its temporal median (systematic error) and robust temporal spread (noise), the
share of sweeps with a return, and at depth edges where the return falls between
the two surfaces. The temporal spread does not depend on static reference errors
(mesh, materials, per-ring bias), which cancel over time.

Usage: ncd_static.py <work_dir>
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import dump, edge_info, read_sim, sim_geometry  # noqa: E402
from temporal import observed_columns  # noqa: E402
from ncd_sim import CONFIGS  # noqa: E402

MIN_RUN = 50


def static_run(stamps):
    runs = np.split(np.arange(len(stamps)), np.flatnonzero(np.diff(stamps) > 0.15) + 1)
    run = max(runs, key=len)
    if len(run) < MIN_RUN:
        sys.exit('no static run long enough')
    return run


def mad_sigma(x, axis):
    med = np.nanmedian(x, axis=axis, keepdims=True)
    return 1.4826 * np.nanmedian(np.abs(x - med), axis=axis)


def pixel_stats(meas, ref, nrm, dirs_w, org, observed=None):
    """meas, ref: (T, H, W); nrm: (H, W, 3); dirs_w: (H, W, 3); org: (W, 3).

    observed: (T, W), the columns of each sweep that carry data; a column with
    no return at all (the operator closer than the minimum range, or lost
    packets) says nothing about dropouts and is not counted in the return share.
    """
    if observed is None:
        observed = np.ones((meas.shape[0], meas.shape[2]), bool)
    n_obs = np.broadcast_to(observed.sum(axis=0)[None, :], meas.shape[1:])
    hit_share = np.mean(ref > 0, axis=0)
    ref_med = np.median(np.where(ref > 0, ref, np.nan), axis=0)
    valid = (hit_share >= 0.9) & np.isfinite(ref_med)
    ref_med = np.where(valid, ref_med, 0.0)
    ret = (meas > 0) & (ref > 0) & observed[:, None, :]
    e = np.where(ret, meas - ref, np.nan)
    n_ret = ret.sum(axis=0)
    with np.errstate(all='ignore'):
        bias = np.nanmedian(e, axis=0)
        sig = mad_sigma(e, axis=0)
    hit = org[None, :, :] + ref_med[..., None] * dirs_w
    off, other = edge_info(hit[None], nrm[None], valid[None])
    off = off[0]
    other = other[0]
    edge = valid & (other[..., 0] >= 0) & (off > np.maximum(0.05, 0.02 * ref_med))
    rr = np.clip(other[..., 0], 0, None)
    cc = np.clip(other[..., 1], 0, None)
    r_other = ref_med[rr, cc]
    cosi = np.abs(np.sum(dirs_w * nrm, axis=-1))
    theta = np.degrees(np.arccos(np.clip(cosi, 0, 1)))
    # Share of the returns between both surfaces at each edge pixel.
    jump = r_other - ref_med
    big = edge & (np.abs(jump) > 0.5)
    with np.errstate(all='ignore'):
        u = (meas - ref_med[None]) / np.where(big, jump, np.nan)[None]
    u = np.where(ret, u, np.nan)
    between = np.nansum((u > 0.1) & (u < 0.9), axis=0)
    at_other = np.nansum(u >= 0.9, axis=0)
    valid = valid & (n_obs >= MIN_RUN)
    return dict(valid=valid, ref=ref_med, theta=theta, edge=edge & valid, big=big & valid, n_ret=n_ret,
                p_ret=n_ret / np.maximum(n_obs, 1), bias=bias, sigma=sig, between=between,
                at_other=at_other, nz=np.abs(nrm[..., 2]), T=meas.shape[0])


def summarize(s, inlier=0.3):
    out = {}
    surf = s['valid'] & ~s['edge']
    on_surface = surf & (s['n_ret'] >= 20) & (np.abs(s['bias']) < inlier)
    rb = [2, 4, 8, 16, 32]
    out['sigma_vs_range'] = []
    for lo, hi in zip(rb[:-1], rb[1:]):
        m = on_surface & (s['theta'] < 30) & (s['ref'] >= lo) & (s['ref'] < hi)
        out['sigma_vs_range'].append(dict(lo=lo, hi=hi, n=int(m.sum()),
                                          sigma=float(np.median(s['sigma'][m])) if m.sum() >= 30 else np.nan,
                                          sigma_q=[float(q) for q in np.percentile(s['sigma'][m], [25, 75])] if m.sum() >= 30 else None))
    tb = [0, 15, 30, 45, 60, 70, 75, 80, 85, 90]
    out['vs_incidence'] = []
    # Dropout statistics: pixels that see the reference surface whenever they return.
    seen = surf & (s['ref'] >= 2) & (s['ref'] < 20) & ((s['n_ret'] == 0) | (np.abs(s['bias']) < inlier))
    for lo, hi in zip(tb[:-1], tb[1:]):
        base = seen & (s['theta'] >= lo) & (s['theta'] < hi)
        m = base & on_surface
        p = s['p_ret'][base]
        out['vs_incidence'].append(dict(
            lo=lo, hi=hi, n=int(base.sum()), p_ret=float(p.mean()) if len(p) else np.nan,
            frac_never=float(np.mean(p == 0)) if len(p) else np.nan,
            frac_partial=float(np.mean((p > 0) & (p < 1))) if len(p) else np.nan,
            sigma=float(np.median(s['sigma'][m])) if m.sum() >= 30 else np.nan,
            bias=float(np.median(s['bias'][m])) if m.sum() >= 30 else np.nan))
    m = on_surface & (s['theta'] < 30) & (s['ref'] >= 3) & (s['ref'] < 20)
    b = s['bias'][m]
    out['bias'] = dict(n=int(m.sum()), median=float(np.median(b)), spread=float(1.4826 * np.median(np.abs(b - np.median(b)))))
    rings = np.broadcast_to(np.arange(s['valid'].shape[0])[:, None], s['valid'].shape)
    meds = [np.median(s['bias'][m & (rings == k)]) for k in range(s['valid'].shape[0]) if (m & (rings == k)).sum() >= 50]
    out['ring_bias'] = dict(n_rings=len(meds), std=float(np.std(meds)) if meds else np.nan)
    big = s['big'] & (s['n_ret'] > 0)
    nr = s['n_ret'][big].sum()
    out['edge'] = dict(n_pixels=int(big.sum()),
                       share_between=float(s['between'][big].sum() / max(nr, 1)),
                       share_at_other=float(s['at_other'][big].sum() / max(nr, 1)),
                       pixels_ever_between=float(np.mean(s['between'][big] > 0)) if big.any() else np.nan)
    out['overall'] = dict(valid=int(s['valid'].sum()), on_surface=int(on_surface.sum()),
                          p_ret_valid=float(s['p_ret'][s['valid']].mean()))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('work')
    a = ap.parse_args()
    w = Path(a.work)
    S = np.load(w / 'ncd_still.npz')
    n, H, W = S['range'].shape
    run = static_run(S['stamp'])
    print(f'static run: {len(run)} sweeps, {S["stamp"][run[-1]] - S["stamp"][run[0]]:.1f} s')
    P = np.load(w / 'poses.npz')
    pose0 = P['poses'][run[0]]
    dirs = np.load(w / 'dirs.npy')
    dirs_w = np.einsum('cij,rcj->rci', pose0[:, :3, :3], dirs)
    h = np.fromfile(w / 'hits.bin', dtype='<f4').reshape(n, H, W, 4)[run]
    meas = S['range'][run].astype(np.float64) * 1e-3
    res = {}
    stats = {}
    observed = observed_columns(meas)
    print(f'columns with no return at all: median {np.median((~observed).sum(axis=1)):.0f} of {W} per sweep')
    stats['real'] = pixel_stats(meas, h[..., 0].astype(np.float64), h[0, ..., 1:].astype(np.float64), dirs_w,
                                pose0[:, :3, 3], observed)
    res['real'] = summarize(stats['real'])
    del h
    org_s, dirs_s = sim_geometry(w, H)
    hs = np.fromfile(w / 'hits_sim.bin', dtype='<f4').reshape(n, H, W, 4)[run]
    for name in CONFIGS:
        rng, _ = read_sim(w / f'sim_{name}.bin', n, H)
        stats[name] = pixel_stats(rng[run].astype(np.float64), hs[..., 0].astype(np.float64),
                                  hs[0, ..., 1:].astype(np.float64), dirs_s[run[0]], org_s[run[0]])
        res[name] = summarize(stats[name])
    dump(res, w / 'static_summary.json')
    np.savez_compressed(w / 'static_pixels.npz', **{f'{k}_{q}': v[q] for k, v in stats.items()
                                                      for q in ('valid', 'ref', 'theta', 'edge', 'big', 'n_ret', 'p_ret', 'bias', 'sigma', 'between', 'at_other')})
    names = ['real'] + list(CONFIGS)
    print('\nTemporal noise (median over pixels of the robust temporal std) [mm], within 30 deg of normal')
    print('range [m] ' + ''.join(f'{k:>10}' for k in names) + '   pixels (real)')
    for i, b in enumerate(res['real']['sigma_vs_range']):
        print(f'{b["lo"]:>3}-{b["hi"]:<3}   ' + ''.join(f'{(res[k]["sigma_vs_range"][i]["sigma"] or np.nan) * 1000:10.1f}' for k in names) + f'   {b["n"]}')
    print('\nAgainst incidence (2-20 m): mean return share | never | partial | noise [mm] | systematic [mm]')
    for i, b in enumerate(res['real']['vs_incidence']):
        row = f'{b["lo"]:>2}-{b["hi"]:<2} deg '
        for k in names:
            v = res[k]['vs_incidence'][i]
            row += f' | {v["p_ret"]:.2f} {v["frac_never"]:.2f} {v["frac_partial"]:.2f} {(v["sigma"] or np.nan) * 1000:4.1f} {(v["bias"] or np.nan) * 1000:+5.1f}'
        print(row + f'  n={b["n"]}')
    print('\nSystematic error (temporal median of e), 3-20 m, within 30 deg: median, spread [mm]; std of ring medians [mm]')
    for k in names:
        print(f'  {k:>6}: {res[k]["bias"]["median"] * 1000:+6.1f} {res[k]["bias"]["spread"] * 1000:6.1f}   rings {res[k]["ring_bias"]["std"] * 1000:5.2f}')
    print('\nDepth edges (jump > 0.5 m): pixels, share of returns between, at the other surface, pixels ever between')
    for k in names:
        e = res[k]['edge']
        print(f'  {k:>6}: {e["n_pixels"]:6d}  {e["share_between"]:.3f}  {e["share_at_other"]:.3f}  {e["pixels_ever_between"]:.3f}')


if __name__ == '__main__':
    main()
