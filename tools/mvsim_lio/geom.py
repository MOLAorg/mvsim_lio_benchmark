"""Small 2D geometry helpers for world layout."""
import math

import numpy as np


def dist_to_polyline(x, y, pts):
    """Distance from point(s) (x, y) to a polyline given as [(x, y), ...]."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    d = np.full(x.shape, np.inf)
    for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        u = np.clip(((x - ax) * dx + (y - ay) * dy) / max(L2, 1e-12), 0.0, 1.0)
        d = np.minimum(d, np.hypot(x - (ax + u * dx), y - (ay + u * dy)))
    return d


def poisson_disk(rng, xmin, xmax, ymin, ymax, r, keep=None, k=20):
    """Bridson's Poisson-disk sampling with minimum distance r.
    keep(x, y) -> bool filters candidate points (e.g. clearings, trails)."""
    cell = r / math.sqrt(2)
    nx = int((xmax - xmin) / cell) + 1
    ny = int((ymax - ymin) / cell) + 1
    grid = -np.ones((nx, ny), dtype=int)
    pts = []
    active = []

    def fits(px, py):
        if not (xmin <= px < xmax and ymin <= py < ymax):
            return False
        gi, gj = int((px - xmin) / cell), int((py - ymin) / cell)
        for i in range(max(0, gi - 2), min(nx, gi + 3)):
            for j in range(max(0, gj - 2), min(ny, gj + 3)):
                q = grid[i, j]
                if q >= 0 and (pts[q][0] - px) ** 2 + (pts[q][1] - py) ** 2 < r * r:
                    return False
        return True

    def add(px, py):
        grid[int((px - xmin) / cell), int((py - ymin) / cell)] = len(pts)
        pts.append((px, py))
        active.append(len(pts) - 1)

    add(rng.uniform(xmin, xmax), rng.uniform(ymin, ymax))
    while active:
        i = int(rng.integers(len(active)))
        bx, by = pts[active[i]]
        for _ in range(k):
            a = rng.uniform(0, 2 * math.pi)
            rr = rng.uniform(r, 2 * r)
            px, py = bx + rr * math.cos(a), by + rr * math.sin(a)
            if fits(px, py):
                add(px, py)
                break
        else:
            active.pop(i)
    if keep is not None:
        pts = [p for p in pts if keep(*p)]
    return pts


def smooth_closed(pts, n_out, sigma=2.0):
    """Resample a closed polygon to n_out points and smooth it (for trails)."""
    a = np.array(pts + pts[:1], dtype=float)
    seg = np.hypot(*np.diff(a, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    si = np.linspace(0, s[-1], n_out, endpoint=False)
    x = np.interp(si, s, a[:, 0])
    y = np.interp(si, s, a[:, 1])
    from scipy.ndimage import gaussian_filter1d
    x = gaussian_filter1d(x, sigma, mode='wrap')
    y = gaussian_filter1d(y, sigma, mode='wrap')
    return list(zip(x, y))
