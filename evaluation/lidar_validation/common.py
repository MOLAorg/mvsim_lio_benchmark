"""Shared readers and the per-pixel statistics, applied identically to real and simulated sweeps."""
import json

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

COLS = 1024
PERIOD = 0.1


def read_sim(path, n_sweeps, n_rings):
    """Simulated sweeps as range and label images (ring order ascending in elevation)."""
    raw = np.fromfile(path, dtype=np.uint8)
    rec = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('t', '<f4'), ('ring', '<u2'), ('label', '<u2')])
    rng = np.zeros((n_sweeps, n_rings, COLS), np.float32)
    lab = np.zeros((n_sweeps, n_rings, COLS), np.uint8)
    pos = 0
    for i in range(n_sweeps):
        count = int(raw[pos:pos + 4].view('<i4')[0])
        pos += 4
        p = raw[pos:pos + count * rec.itemsize].view(rec)
        pos += count * rec.itemsize
        col = np.rint(p['t'] / (PERIOD / COLS)).astype(int)
        r = np.sqrt(p['x'].astype(np.float64) ** 2 + p['y'] ** 2 + p['z'] ** 2)
        rng[i, p['ring'], col] = r
        lab[i, p['ring'], col] = p['label']
    if pos != len(raw):
        raise ValueError(f'{path}: {len(raw) - pos} trailing bytes')
    return rng, lab


def sim_geometry(work, n_rings):
    """Ray origins and world directions of the simulated grid (MVSim's Lidar3DModel)."""
    alt = np.sort(np.loadtxt(work / 'sim_elev.txt'))
    el = np.radians(alt)
    az = -np.pi + np.arange(COLS) * (2 * np.pi / COLS)
    d = np.stack([np.cos(el)[:, None] * np.cos(az)[None, :], np.cos(el)[:, None] * np.sin(az)[None, :],
                  np.broadcast_to(np.sin(el)[:, None], (n_rings, COLS))], -1)
    k = np.loadtxt(work / 'sim_poses.tum')
    n = len(k) // 2
    t_fire = np.arange(COLS) * (PERIOD / COLS)
    org = np.zeros((n, COLS, 3))
    rot = np.zeros((n, COLS, 3, 3))
    for i in range(n):
        a = k[2 * i]
        b = k[2 * i + 1]
        f = np.clip(t_fire / (b[0] - a[0]), 0, 1)
        org[i] = a[1:4] + f[:, None] * (b[1:4] - a[1:4])
        rot[i] = Slerp([0, 1], R.from_quat([a[4:8], b[4:8]]))(f).as_matrix()
    dw = np.einsum('ncij,rcj->nrci', rot, d)
    return org, dw


def edge_info(hit, normal, ok):
    """MVSim's mixed-return criterion on the reference hits: the 4-neighbor farthest
    off this pixel's tangent plane, if farther than max(5 cm, 2% of the range)."""
    n, H, W = ok.shape
    other_idx = np.full(ok.shape + (2,), -1, np.int64)
    best = np.zeros(ok.shape)
    for dr, dc in ((0, -1), (0, 1), (-1, 0), (1, 0)):
        q = np.roll(hit, shift=(-dr, -dc), axis=(1, 2))
        qok = np.roll(ok, shift=(-dr, -dc), axis=(1, 2))
        if dr == -1:
            qok[:, 0, :] = False
        if dr == 1:
            qok[:, -1, :] = False
        off = np.abs(np.sum((q - hit) * normal, axis=-1))
        m = ok & qok & (off > best)
        best = np.where(m, off, best)
        rr = (np.arange(H)[None, :, None] + dr) % H
        cc = (np.arange(W)[None, None, :] + dc) % W
        other_idx[..., 0] = np.where(m, np.broadcast_to(rr, ok.shape), other_idx[..., 0])
        other_idx[..., 1] = np.where(m, np.broadcast_to(cc, ok.shape), other_idx[..., 1])
    return best, other_idx


def pixel_table(meas, ref, normal, dirs_w, org):
    """Per-pixel quantities of every pixel whose reference ray hit the mesh.

    meas: measured range (0: no return); ref: reference range (0: no hit);
    normal: reference unit normal; dirs_w: unit ray directions (world);
    org: ray origins (world), per sweep and column.
    """
    ok = ref > 0
    hit = org[:, None, :, :] + ref[..., None] * dirs_w
    cosi = np.abs(np.sum(dirs_w * normal, axis=-1))
    off, other = edge_info(hit, normal, ok)
    rr = np.clip(other[..., 0], 0, None)
    cc = np.clip(other[..., 1], 0, None)
    si = np.arange(ok.shape[0])[:, None, None]
    r_other = ref[si, rr, cc]
    edge = ok & (other[..., 0] >= 0) & (off > np.maximum(0.05, 0.02 * ref))
    sel = ok
    return dict(ref=ref[sel], meas=meas[sel], cosi=cosi[sel], edge=edge[sel],
                r_other=np.where(edge, r_other, 0)[sel], nz=np.abs(normal[..., 2])[sel],
                ring=np.broadcast_to(np.arange(ok.shape[1])[None, :, None], ok.shape)[sel])


def robust_sigma(e):
    if len(e) < 30:
        return np.nan
    return 1.4826 * np.median(np.abs(e - np.median(e)))


def summarize(t, inlier=0.3):
    """The statistics compared between real and simulated sweeps."""
    ret = t['meas'] > 0
    e = np.where(ret, t['meas'] - t['ref'], np.nan)
    theta = np.degrees(np.arccos(np.clip(t['cosi'], 0, 1)))
    surf = ~t['edge']
    out = {}
    rbins = [1, 2, 4, 8, 16, 32, 60]
    head_on = surf & (theta < 30)
    out['sigma_vs_range'] = []
    for lo, hi in zip(rbins[:-1], rbins[1:]):
        m = head_on & ret & (t['ref'] >= lo) & (t['ref'] < hi) & (np.abs(e) < inlier)
        out['sigma_vs_range'].append(dict(lo=lo, hi=hi, n=int(m.sum()), sigma=robust_sigma(e[m]),
                                          median=float(np.median(e[m])) if m.any() else np.nan))
    tb = list(range(0, 91, 10))
    out['vs_incidence'] = []
    for lo, hi in zip(tb[:-1], tb[1:]):
        base = surf & (theta >= lo) & (theta < hi) & (t['ref'] >= 2) & (t['ref'] < 30)
        m = base & ret & (np.abs(e) < inlier)
        out['vs_incidence'].append(dict(
            lo=lo, hi=hi, n=int(base.sum()), p_return=float(m.sum() / max(base.sum(), 1)),
            p_any=float((base & ret).sum() / max(base.sum(), 1)),
            sigma=robust_sigma(e[m]), median=float(np.median(e[m])) if m.any() else np.nan))
    big = t['edge'] & ret & (np.abs(t['r_other'] - t['ref']) > 0.5)
    u = (t['meas'][big] - t['ref'][big]) / (t['r_other'][big] - t['ref'][big])
    out['edge'] = dict(n_edge=int(t['edge'].sum()), n_big=int(big.sum()),
                       frac_between=float(np.mean((u > 0.1) & (u < 0.9))) if len(u) else np.nan,
                       frac_other=float(np.mean(u >= 0.9)) if len(u) else np.nan,
                       hist=np.histogram(np.clip(u, -0.5, 1.5), bins=40, range=(-0.5, 1.5))[0].tolist())
    rm = head_on & ret & (t['ref'] >= 3) & (t['ref'] < 20) & (np.abs(e) < inlier)
    meds = [np.median(e[rm & (t['ring'] == k)]) for k in np.unique(t['ring']) if (rm & (t['ring'] == k)).sum() > 200]
    out['ring_bias'] = dict(n_rings=len(meds), std=float(np.std(meds)), spread=float(np.ptp(meds)) if meds else np.nan)
    allm = ret & (np.abs(e) < inlier)
    out['overall'] = dict(n_ref=int(len(e)), p_return=float(ret.mean()), p_inlier=float(allm.mean()),
                          p_outlier=float((ret & (np.abs(e) >= inlier)).mean()),
                          sigma_inlier=robust_sigma(e[allm & surf]))
    return out


def dump(obj, path):
    def conv(x):
        if isinstance(x, (np.floating, float)):
            return None if np.isnan(x) else float(x)
        if isinstance(x, np.integer):
            return int(x)
        raise TypeError(type(x))
    path.write_text(json.dumps(obj, indent=1, default=conv))
