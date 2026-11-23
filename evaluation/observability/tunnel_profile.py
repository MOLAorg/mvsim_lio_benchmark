#!/usr/bin/env python3
"""Per-sweep tunnel arc length and portal distance for the observability outputs.

Usage: tunnel_profile.py <out_dir_of_obs.py>
Prints the statistics of the sweeps inside the tunnel, farther than 70 m from both portals.
"""
import math
import sys
from pathlib import Path

import numpy as np

L1, R, ANG, L2 = 250.0, 300.0, math.radians(45.0), 250.0
X_START = -300.0
LENGTH = L1 + R * ANG + L2


def centerline(s):
    if s <= L1:
        return X_START + s, 0.0
    if s <= L1 + R * ANG:
        a = (s - L1) / R
        return X_START + L1 + R * math.sin(a), R * (1 - math.cos(a))
    x1, y1 = X_START + L1 + R * math.sin(ANG), R * (1 - math.cos(ANG))
    d = s - L1 - R * ANG
    return x1 + d * math.cos(ANG), y1 + d * math.sin(ANG)


def main():
    out = Path(sys.argv[1])
    A = np.genfromtxt(out / 'sweeps.tsv', names=True, delimiter='\t')
    S = np.genfromtxt(out / 'shifts.tsv', names=True, delimiter='\t')
    ss = np.arange(-120.0, LENGTH + 120.0, 0.25)
    cl = np.array([centerline(min(max(s, 0.0), LENGTH)) for s in ss])
    # Outside the tunnel the centerline is extended straight.
    for i, s in enumerate(ss):
        if s < 0:
            cl[i] = (X_START + s, 0.0)
        elif s > LENGTH:
            x1, y1 = centerline(LENGTH)
            cl[i] = (x1 + (s - LENGTH) * math.cos(ANG), y1 + (s - LENGTH) * math.sin(ANG))

    def arc(x, y):
        d = np.hypot(cl[:, 0] - x, cl[:, 1] - y)
        j = np.argmin(d)
        return ss[j], d[j]

    s_sw = np.array([arc(x, y)[0] for x, y in zip(A['x'], A['y'])])
    inner = (s_sw > 70) & (s_sw < LENGTH - 70)
    bend = (s_sw > L1) & (s_sw < L1 + R * ANG)
    st = {round(t, 3): s for t, s in zip(A['stamp'], s_sw)}
    s_sh = np.array([st.get(round(t, 3), np.nan) for t in S['stamp']])
    inner_sh = (s_sh > 70) & (s_sh < LENGTH - 70)
    sa = A['sig_axial']
    print(f'{out.name}: {len(A)} sweeps, {inner.sum()} inside (>70 m from portals), '
          f'{(inner & bend).sum()} of them in the bend')
    for name, m in (('inside', inner), ('inside, straight', inner & ~bend), ('inside, bend', inner & bend),
                    ('near/outside portals', ~inner)):
        if m.sum():
            print(f'  {name:22s} sig_axial p50 {1e3 * np.median(sa[m]):7.2f} mm  p95 '
                  f'{1e3 * np.percentile(sa[m], 95):7.2f} mm  max {1e3 * sa[m].max():8.2f} mm; '
                  f'weakest dir axial {100 * (A["weakest_dot_axial"][m] > 0.9).mean():3.0f} %; '
                  f'lateral p50 {1e3 * np.median(A["sig_lateral"][m]):.3f} mm')
    for name, m in (('inside', inner_sh), ('outside', ~inner_sh)):
        if m.sum():
            dp = S['discriminating_pts'][m]
            print(f'  shifts {name:8s}: {m.sum()} sweeps; discriminating pts p0 {dp.min():.0f} '
                  f'p5 {np.percentile(dp, 5):.0f} p25 {np.percentile(dp, 25):.0f} '
                  f'p50 {np.median(dp):.0f}; best alternative shifts '
                  f'{np.unique(np.round(np.abs(S["best_alt_shift"][m])), return_counts=True)}')


if __name__ == '__main__':
    main()
