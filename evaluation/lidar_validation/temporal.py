"""Per-pixel temporal statistics of a static LiDAR (a stack of sweeps from a still sensor),
identical for real and simulated sweeps.

- noise: robust temporal spread of each pixel's range;
- geometry from the per-pixel median range: normals by local PCA, incidence angle;
- return share: returns over the sweeps in which the pixel's column carries data;
- depth edges along a ring: a pixel whose two horizontal neighbors lie on smooth
  surfaces more than `jump` apart; where its returns fall between both depths.
"""
import numpy as np

MIN_GEOM_RETURNS = 5


def pixel_dirs_from_xyz(xyz):
    """Unit pixel directions: per ring constant elevation, azimuth linear in the column,
    fitted to the returns of an (H, W, 3) organized cloud."""
    n = np.linalg.norm(xyz, axis=-1, keepdims=True)
    d = np.where(n > 0.3, xyz / np.maximum(n, 1e-9), np.nan)
    H, W, _ = d.shape
    el = np.nanmedian(np.arcsin(d[..., 2]), axis=1)
    cols = np.arange(W)
    out = np.zeros((H, W, 3))
    for r in range(H):
        ok = np.isfinite(d[r, :, 0])
        az = np.arctan2(d[r, ok, 1], d[r, ok, 0]) + 2 * np.pi * cols[ok] / W
        a0 = np.angle(np.mean(np.exp(1j * az)))
        azc = a0 - 2 * np.pi * cols / W
        out[r] = np.stack([np.cos(el[r]) * np.cos(azc), np.cos(el[r]) * np.sin(azc), np.full(W, np.sin(el[r]))], 1)
    return out


def shift(a, dr, dc, fill=np.nan):
    """b[r, c] = a[r + dr, c + dc]: columns wrap around, rings do not."""
    b = np.roll(a, shift=(-dr, -dc), axis=(0, 1)).astype(float)
    if dr > 0:
        b[-dr:] = fill
    elif dr < 0:
        b[:-dr] = fill
    return b


def local_normals(P, valid, half=(1, 2), gate=0.05):
    """Normals and planarity (smallest eigenvalue over the sum) from the neighbors
    within +-half rings and columns whose range is within gate*r + 5 cm."""
    rng = np.where(valid, np.linalg.norm(P, axis=-1), np.nan)
    n = np.zeros(valid.shape)
    s1 = np.zeros(valid.shape + (3,))
    s2 = np.zeros(valid.shape + (3, 3))
    for dr in range(-half[0], half[0] + 1):
        for dc in range(-half[1], half[1] + 1):
            q = np.stack([shift(P[..., i], dr, dc) for i in range(3)], -1)
            qr = shift(rng, dr, dc)
            m = valid & np.isfinite(qr) & (np.abs(qr - rng) < gate * rng + 0.05)
            q = np.where(m[..., None], q, 0.0)
            n += m
            s1 += q
            s2 += q[..., :, None] * q[..., None, :]
    ok = valid & (n >= 6)
    nn = np.maximum(n, 1)[..., None]
    mu = s1 / nn
    C = s2 / nn[..., None] - mu[..., :, None] * mu[..., None, :]
    C = np.where(ok[..., None, None], C, np.eye(3))
    w, v = np.linalg.eigh(C)
    N = np.where(ok[..., None], v[..., :, 0], np.nan)
    flat = np.where(ok, w[..., 0] / np.maximum(w.sum(-1), 1e-12), np.nan)
    return N, flat


def observed_columns(rng, share=0.8):
    """(T, W): a column of a sweep counts as observed if it holds at least `share` of its
    median number of returns over the run; columns blocked by something close to the
    sensor (an operator, a passer-by) or lost in transmission say nothing about dropouts."""
    n = (rng > 0).sum(axis=1)
    return n >= share * np.median(n, axis=0)[None, :]


def temporal_stats(rng, dirs, observed=None, jump=0.5):
    """rng: (T, H, W) ranges [m], 0 = no return; dirs: (H, W, 3) unit directions."""
    T, H, W = rng.shape
    if observed is None:
        observed = observed_columns(rng)
    n_obs = np.broadcast_to(observed.sum(0)[None, :], (H, W))
    ret = (rng > 0) & observed[:, None, :]
    n_ret = ret.sum(0)
    r = np.where(ret, rng, np.nan)
    with np.errstate(all='ignore'):
        med = np.nanmedian(r, axis=0)
        sig = 1.4826 * np.nanmedian(np.abs(r - med[None]), axis=0)
        inl = np.nanmean(np.abs(r - med[None]) < np.maximum(0.1, 5 * sig[None]), axis=0)
    geo = np.isfinite(med) & (n_ret >= MIN_GEOM_RETURNS)
    P = np.where(geo[..., None], med[..., None] * dirs, 0.0)
    N, flat = local_normals(P, geo)
    cosi = np.abs(np.sum(N * dirs, axis=-1))
    theta = np.degrees(np.arccos(np.clip(cosi, 0, 1)))
    # Depth edges along the ring: neighbors at c-1 and c+1 on two smooth surfaces.
    ml = shift(np.where(geo, med, np.nan), 0, -1)
    mr = shift(np.where(geo, med, np.nan), 0, 1)
    ml2 = shift(np.where(geo, med, np.nan), 0, -2)
    mr2 = shift(np.where(geo, med, np.nan), 0, 2)
    with np.errstate(all='ignore'):
        near = np.fmin(ml, mr)
        far = np.fmax(ml, mr)
        smooth = (np.abs(ml - ml2) < 0.05 * ml + 0.05) & (np.abs(mr - mr2) < 0.05 * mr + 0.05)
    edge = np.isfinite(near) & np.isfinite(far) & (far - near > jump) & smooth & (n_ret > 0)
    with np.errstate(all='ignore'):
        u = (rng - near[None]) / (far - near)[None]
    u = np.where(ret & edge[None], u, np.nan)
    between = np.nansum((u > 0.1) & (u < 0.9), axis=0)
    at_far = np.nansum(u >= 0.9, axis=0)
    at_near = np.nansum(u <= 0.1, axis=0)
    hist = np.histogram(u[np.isfinite(u)].clip(-0.25, 1.25), bins=30, range=(-0.25, 1.25))[0]
    # Pixels that never return although both ring neighbors return steadily on one surface.
    up = shift(np.where(geo, med, np.nan), -1, 0)
    dn = shift(np.where(geo, med, np.nan), 1, 0)
    with np.errstate(all='ignore'):
        gap = (n_ret == 0) & (n_obs > 0) & np.isfinite(up) & np.isfinite(dn) & (np.abs(up - dn) < 0.1 * np.fmin(up, dn) + 0.1)
        gap_theta = 0.5 * (shift(theta, -1, 0) + shift(theta, 1, 0))
    gap_range = 0.5 * (up + dn)
    return dict(T=T, med=med, sig=sig, inl=inl, n_ret=n_ret, n_obs=n_obs, geo=geo, theta=theta, flat=flat,
                edge=edge, edge_gap=np.where(edge, far - near, np.nan), between=between, at_far=at_far,
                at_near=at_near, u_hist=hist,
                gap=gap, gap_theta=gap_theta, gap_range=gap_range)


def pool(stats):
    keys = [k for k in stats[0] if k not in ('T', 'u_hist')]
    out = {k: np.concatenate([s[k].ravel() for s in stats]) for k in keys}
    out['u_hist'] = np.sum([s['u_hist'] for s in stats], axis=0)
    out['T'] = int(sum(s['T'] for s in stats))
    return out


def _med(x, n=30):
    return float(np.median(x)) if len(x) >= n else None


def summarize(p, max_range=20.0, models=None):
    """models: {name: (sigma0, k, divergence_deg)}; their noise is predicted per pixel
    from its own range and incidence, and summarized over the same pixels."""
    models = models or {}
    out = {}
    surf = p['geo'] & ~p['edge'] & (p['flat'] < 0.01) & (p['inl'] > 0.9) & (p['n_ret'] >= 30)
    rb = [1, 2, 4, 8, 16, 32, 64]
    out['sigma_vs_range'] = []
    for lo, hi in zip(rb[:-1], rb[1:]):
        m = surf & (p['theta'] < 30) & (p['med'] >= lo) & (p['med'] < hi)
        s = p['sig'][m]
        pred = {k: _med(model_sigma(p['med'][m], p['theta'][m], *v)) for k, v in models.items()}
        out['sigma_vs_range'].append(dict(lo=lo, hi=hi, n=int(m.sum()), sigma=_med(s), model=pred,
                                          q25=float(np.percentile(s, 25)) if len(s) >= 30 else None,
                                          q75=float(np.percentile(s, 75)) if len(s) >= 30 else None))
    tb = [0, 15, 30, 45, 60, 70, 75, 80, 85, 90]
    out['vs_incidence'] = []
    # Return share: every pixel with a known geometry, plus never-returning gaps on a surface.
    cand = p['geo'] & ~p['edge'] & (p['flat'] < 0.01) & (p['inl'] > 0.9) & (p['med'] >= 2) & (p['med'] < max_range)
    for lo, hi in zip(tb[:-1], tb[1:]):
        m = cand & (p['theta'] >= lo) & (p['theta'] < hi)
        g = p['gap'] & (p['gap_theta'] >= lo) & (p['gap_theta'] < hi) & (p['gap_range'] >= 2) & (p['gap_range'] < max_range)
        share = p['n_ret'][m] / np.maximum(p['n_obs'][m], 1)
        n_all = int(m.sum() + g.sum())
        ms = surf & (p['theta'] >= lo) & (p['theta'] < hi) & (p['med'] >= 2) & (p['med'] < max_range)
        pred = {k: _med(model_sigma(p['med'][ms], p['theta'][ms], *v)) for k, v in models.items()}
        out['vs_incidence'].append(dict(lo=lo, hi=hi, n=int(m.sum()), n_never=int(g.sum()), model=pred,
                                        frac_full=float(np.mean(share >= 0.99)) if m.any() else None,
                                        p_ret=float(share.sum() / max(n_all, 1)) if n_all else None,
                                        frac_partial=float(np.mean(share < 0.99)) if m.any() else None,
                                        sigma=_med(p['sig'][ms])))
    e = p['edge'] & (p['n_ret'] >= 30)
    nr = max(int(p['n_ret'][e].sum()), 1)
    ever = p['between'][e] > 0
    often = p['between'][e] / np.maximum(p['n_ret'][e], 1) > 0.5
    fb = p['between'][e] / np.maximum(p['n_ret'][e], 1)
    out['edge_pixel_share_hist'] = np.histogram(fb, bins=[0, 1e-9, 0.05, 0.2, 0.5, 0.9, 1.0001])[0].tolist()
    # Share of the returns between both surfaces against the depth gap of the edge:
    out['edge_by_gap'] = []
    for lo, hi in [(0.5, 1), (1, 2), (2, 4), (4, 8), (8, 1000)]:
        g = e & (p['edge_gap'] >= lo) & (p['edge_gap'] < hi)
        out['edge_by_gap'].append(dict(lo=lo, hi=hi, n=int(g.sum()),
                                       share_between=float(p['between'][g].sum() / max(int(p['n_ret'][g].sum()), 1))))
    out['edge'] = dict(n_pixels=int(e.sum()), share_between=float(p['between'][e].sum() / nr),
                       pixels_mixing=float(np.mean((fb > 0) & (fb < 0.9))) if e.any() else None,
                       share_far=float(p['at_far'][e].sum() / nr), share_near=float(p['at_near'][e].sum() / nr),
                       pixels_ever_between=float(ever.mean()) if e.any() else None,
                       pixels_mostly_between=float(often.mean()) if e.any() else None,
                       u_hist=[int(x) for x in p['u_hist']])
    out['sweeps'] = p['T']
    return out


def model_sigma(r, theta, sigma0, k, div_deg, clamp_deg=84.3):
    """Std of the benchmark's range error on a plane: Gaussian plus the uniform footprint
    term, whose spread stops growing beyond `clamp_deg` (84.3 deg in the first release's
    model, 80 deg in the beam model)."""
    t = np.radians(np.minimum(theta, clamp_deg))
    fp = r * np.radians(div_deg) * np.tan(t)
    return np.sqrt(sigma0 ** 2 + (k * r) ** 2 + fp ** 2 / 12.0)
