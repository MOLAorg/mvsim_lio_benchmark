#!/usr/bin/env python3
"""Per-pixel temporal statistics of the Husky OS0-32 while the robot stands still, and
the predictions of the benchmark's error model for the same statistics.

For each stationary interval, from the per-pixel median range image: 3D points,
normals (local PCA), incidence angles and depth edges. Over the sweeps of the
interval: per-pixel robust temporal spread of the range (noise), share of sweeps
with a return, and at depth edges where each return falls between the near and
the far surface. No reference map is needed: every statistic is temporal.

Usage: husky_static.py <dir with *_still*.npz> <out.json>
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

MIN_SWEEPS = 50


def pixel_dirs(xyz):
    n = np.linalg.norm(xyz, axis=-1, keepdims=True)
    d = np.where(n > 0.3, xyz / np.maximum(n, 1e-9), np.nan)
    H, W, _ = d.shape
    # Fill pixels without a return in the reference sweep: per ring, constant
    # elevation and azimuth linear in the column, fitted to the returns.
    el = np.nanmedian(np.arcsin(d[..., 2]), axis=1)
    cols = np.arange(W)
    out = np.zeros_like(d)
    for r in range(H):
        ok = np.isfinite(d[r, :, 0])
        az = np.arctan2(d[r, ok, 1], d[r, ok, 0]) + 2 * np.pi * cols[ok] / W
        a0 = np.angle(np.mean(np.exp(1j * az)))
        azc = a0 - 2 * np.pi * cols / W
        out[r] = np.stack([np.cos(el[r]) * np.cos(azc), np.cos(el[r]) * np.sin(azc), np.full(W, np.sin(el[r]))], 1)
    return out


def normals(P, valid, half=(1, 2), gate=0.05):
    """Per-pixel normal and planarity from the neighbors within +-half (rings, cols)."""
    H, W, _ = P.shape
    N = np.full((H, W, 3), np.nan)
    flat = np.zeros((H, W))
    rng = np.linalg.norm(P, axis=-1)
    for r in range(H):
        for c in range(W):
            if not valid[r, c]:
                continue
            rs = slice(max(0, r - half[0]), min(H, r + half[0] + 1))
            cs = [(c + k) % W for k in range(-half[1], half[1] + 1)]
            Q = P[rs][:, cs].reshape(-1, 3)
            qv = valid[rs][:, cs].reshape(-1)
            qr = rng[rs][:, cs].reshape(-1)
            m = qv & (np.abs(qr - rng[r, c]) < gate * rng[r, c] + 0.05)
            if m.sum() < 6:
                continue
            X = Q[m] - Q[m].mean(0)
            w, v = np.linalg.eigh(X.T @ X)
            N[r, c] = v[:, 0]
            flat[r, c] = w[0] / max(w.sum(), 1e-12)
    return N, flat


def interval_stats(f):
    z = np.load(f)
    rng = z['range'].astype(np.float64) * 1e-3
    T, H, W = rng.shape
    if T < MIN_SWEEPS:
        return None
    observed = (rng > 0).any(axis=1)
    n_obs = np.broadcast_to(observed.sum(0)[None, :], (H, W))
    ret = rng > 0
    n_ret = ret.sum(0)
    r = np.where(ret, rng, np.nan)
    with np.errstate(all='ignore'):
        med = np.nanmedian(r, axis=0)
        sig = 1.4826 * np.nanmedian(np.abs(r - med[None]), axis=0)
        inl = np.nanmean(np.abs(r - med[None]) < np.maximum(0.1, 5 * sig[None]), axis=0)
    D = pixel_dirs(z['xyz'].astype(np.float64))
    valid = np.isfinite(med) & (n_ret >= 0.5 * n_obs)
    P = np.where(valid[..., None], med[..., None] * D, 0.0)
    N, flat = normals(P, valid)
    cosi = np.abs(np.sum(N * D, axis=-1))
    theta = np.degrees(np.arccos(np.clip(cosi, 0, 1)))
    refl = np.nanmedian(np.where(ret, z['refl'].astype(np.float64), np.nan), axis=0)
    # Depth edges: near and far surface among the pixel and its 4-neighbors.
    stack = [med]
    for dr, dc in ((0, -1), (0, 1), (-1, 0), (1, 0)):
        q = np.roll(med, shift=(-dr, -dc), axis=(0, 1))
        if dr == -1:
            q[0, :] = np.nan
        if dr == 1:
            q[-1, :] = np.nan
        stack.append(q)
    S = np.stack(stack)
    with np.errstate(all='ignore'):
        near = np.nanmin(S, axis=0)
        far = np.nanmax(S, axis=0)
    edge = np.isfinite(near) & (far - near > 0.5) & (n_ret > 0)
    with np.errstate(all='ignore'):
        u = (rng - near[None]) / (far - near)[None]
    u = np.where(ret & edge[None], u, np.nan)
    between = np.nansum((u > 0.1) & (u < 0.9), axis=0)
    at_far = np.nansum(u >= 0.9, axis=0)
    at_near = np.nansum(u <= 0.1, axis=0)
    # Never-returning pixels on a surface: both ring neighbors return steadily and agree.
    up = np.roll(med, 1, axis=0)
    dn = np.roll(med, -1, axis=0)
    up[0, :] = np.nan
    dn[-1, :] = np.nan
    th_up = np.roll(theta, 1, axis=0)
    th_dn = np.roll(theta, -1, axis=0)
    gap = (n_ret == 0) & np.isfinite(up) & np.isfinite(dn) & (np.abs(up - dn) < 0.1 * np.fmin(up, dn) + 0.1)
    gap_theta = 0.5 * (th_up + th_dn)
    gap_range = 0.5 * (up + dn)
    return dict(T=T, med=med, sig=sig, inl=inl, n_ret=n_ret, n_obs=n_obs, valid=valid, theta=theta,
                flat=flat, refl=refl, edge=edge, between=between, at_far=at_far, at_near=at_near,
                gap=gap, gap_theta=gap_theta, gap_range=gap_range)


def pool(stats):
    keys = ('med', 'sig', 'inl', 'n_ret', 'n_obs', 'valid', 'theta', 'flat', 'refl', 'edge', 'between', 'at_far',
            'at_near', 'gap', 'gap_theta', 'gap_range')
    return {k: np.concatenate([s[k].ravel() for s in stats]) for k in keys}


def model_sigma(r, theta, sigma0, k, div_deg):
    t = np.radians(np.minimum(theta, 84.3))
    fp = r * np.radians(div_deg) * np.tan(t)
    return np.sqrt(sigma0 ** 2 + (k * r) ** 2 + fp ** 2 / 12.0)


def summarize(p):
    out = {}
    surf = p['valid'] & ~p['edge'] & (p['flat'] < 0.01) & (p['inl'] > 0.9) & (p['n_ret'] >= 30)
    rb = [1, 2, 4, 8, 16, 32, 64]
    out['sigma_vs_range'] = []
    for lo, hi in zip(rb[:-1], rb[1:]):
        m = surf & (p['theta'] < 30) & (p['med'] >= lo) & (p['med'] < hi)
        s = p['sig'][m]
        out['sigma_vs_range'].append(dict(lo=lo, hi=hi, n=int(m.sum()),
                                          sigma=float(np.median(s)) if len(s) >= 30 else None,
                                          q25=float(np.percentile(s, 25)) if len(s) >= 30 else None,
                                          q75=float(np.percentile(s, 75)) if len(s) >= 30 else None))
    tb = [0, 15, 30, 45, 60, 70, 75, 80, 85, 90]
    out['vs_incidence'] = []
    for lo, hi in zip(tb[:-1], tb[1:]):
        m = surf & (p['theta'] >= lo) & (p['theta'] < hi) & (p['med'] >= 2) & (p['med'] < 20)
        g = p['gap'] & (p['gap_theta'] >= lo) & (p['gap_theta'] < hi) & (p['gap_range'] >= 2) & (p['gap_range'] < 20)
        share = p['n_ret'][m] / np.maximum(p['n_obs'][m], 1)
        n_all = m.sum() + g.sum()
        out['vs_incidence'].append(dict(
            lo=lo, hi=hi, n=int(m.sum()), n_never=int(g.sum()),
            p_ret=float((share.sum()) / max(n_all, 1)),
            frac_never=float(g.sum() / max(n_all, 1)),
            frac_partial=float(np.mean(share < 0.99)) if m.any() else None,
            sigma=float(np.median(p['sig'][m])) if m.sum() >= 30 else None))
    rb2 = [0, 20, 50, 100, 200, 1e9]
    out['sigma_vs_reflectivity'] = []
    for lo, hi in zip(rb2[:-1], rb2[1:]):
        m = surf & (p['theta'] < 30) & (p['med'] >= 4) & (p['med'] < 16) & (p['refl'] >= lo) & (p['refl'] < hi)
        out['sigma_vs_reflectivity'].append(dict(lo=lo, hi=hi, n=int(m.sum()),
                                                 sigma=float(np.median(p['sig'][m])) if m.sum() >= 30 else None))
    e = p['edge'] & (p['n_ret'] >= 30)
    nr = p['n_ret'][e].sum()
    ever = p['between'][e] > 0
    out['edge'] = dict(n_pixels=int(e.sum()), share_between=float(p['between'][e].sum() / max(nr, 1)),
                       share_far=float(p['at_far'][e].sum() / max(nr, 1)),
                       share_near=float(p['at_near'][e].sum() / max(nr, 1)),
                       pixels_ever_between=float(ever.mean()),
                       share_between_when_ever=float(p['between'][e][ever].sum() / max(p['n_ret'][e][ever].sum(), 1)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dir')
    ap.add_argument('out')
    a = ap.parse_args()
    files = sorted(Path(a.dir).glob('*_still*.npz'))
    stats = []
    for f in files:
        s = interval_stats(f)
        if s is None:
            continue
        stats.append(s)
        print(f'{f.name}: {s["T"]} sweeps', flush=True)
    p = pool(stats)
    res = summarize(p)
    res['intervals'] = len(stats)
    res['sweeps'] = int(sum(s['T'] for s in stats))
    # Model predictions at the low, middle and high end of the parameter ranges.
    res['model'] = {}
    for name, (s0, k, dv) in dict(low=(0.005, 0, 0.05), mid=(0.0125, 1.5e-4, 0.175), high=(0.02, 3e-4, 0.3)).items():
        res['model'][name] = dict(
            sigma_vs_range=[float(model_sigma(np.sqrt(b['lo'] * b['hi']), 15, s0, k, dv)) for b in res['sigma_vs_range']],
            sigma_vs_incidence=[float(model_sigma(8.0, 0.5 * (b['lo'] + b['hi']), s0, k, dv)) for b in res['vs_incidence']])
    Path(a.out).write_text(json.dumps(res, indent=1))
    print(f'\n{res["intervals"]} intervals, {res["sweeps"]} sweeps')
    print('Temporal noise vs range (within 30 deg): median [q25, q75] mm | model low/mid/high')
    for i, b in enumerate(res['sigma_vs_range']):
        mm = ' / '.join(f'{res["model"][k]["sigma_vs_range"][i] * 1000:.1f}' for k in ('low', 'mid', 'high'))
        if b['sigma'] is None:
            print(f'  {b["lo"]:>2}-{b["hi"]:<3} m: n={b["n"]}')
            continue
        print(f'  {b["lo"]:>2}-{b["hi"]:<3} m: {b["sigma"] * 1000:5.1f} [{b["q25"] * 1000:.1f}, {b["q75"] * 1000:.1f}]  n={b["n"]:6d} | {mm}')
    print('Against incidence (2-20 m): return share, never, partial, noise [mm] | model noise low/mid/high at 8 m')
    for i, b in enumerate(res['vs_incidence']):
        mm = ' / '.join(f'{res["model"][k]["sigma_vs_incidence"][i] * 1000:.1f}' for k in ('low', 'mid', 'high'))
        sg = f'{b["sigma"] * 1000:5.1f}' if b['sigma'] is not None else '   - '
        print(f'  {b["lo"]:>2}-{b["hi"]:<2} deg: {b["p_ret"]:.3f} {b["frac_never"]:.3f} {b["frac_partial"] if b["frac_partial"] is None else round(b["frac_partial"], 3)} {sg}  n={b["n"]}+{b["n_never"]} | {mm}')
    print('Noise vs reflectivity (4-16 m, within 30 deg):', [(b['lo'], b['n'], None if b['sigma'] is None else round(b['sigma'] * 1000, 1)) for b in res['sigma_vs_reflectivity']])
    print('Depth edges:', json.dumps(res['edge']))


if __name__ == '__main__':
    sys.exit(main())
