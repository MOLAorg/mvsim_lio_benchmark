#!/usr/bin/env python3
"""Summary of the observability outputs: results/observability.tsv and tables/observability.tex.

Tunnel sequences are summarized inside the tunnel, farther than 70 m from both portals.

Usage: summarize.py <obs_out_root> <repo_root>
"""
import json
import shutil
import sys
from pathlib import Path

import numpy as np

import tunnel_profile as tp

SEQS = [('urban_street__car_loops', 'urban street, car loops'),
        ('greenhouse__aisles', 'greenhouse, aisles'),
        ('rough_terrain__hill_loop', 'rough terrain, hill loop'),
        ('campus__ring_drive', 'campus, ring drive'),
        ('campus__long_tour', 'campus, long tour'),
        ('open_field__lap', 'open field, lap'),
        ('open_field__zigzag', 'open field, zigzag'),
        ('tunnel__car_through', 'tunnel, car$^\\ast$'),
        ('tunnel__ugv_inspection', 'tunnel, UGV$^\\ast$')]


def load(p):
    lines = p.read_text().splitlines()
    return lines[0].split('\t'), np.loadtxt(lines[1:], delimiter='\t', ndmin=2)


def inside_mask(stamps, pos):
    ss = np.arange(-120.0, tp.LENGTH + 120.0, 0.25)
    cl = np.array([tp.centerline(min(max(s, 0.0), tp.LENGTH)) for s in ss])
    s = np.array([ss[np.argmin(np.hypot(cl[:, 0] - pos[round(t, 3)][0],
                                         cl[:, 1] - pos[round(t, 3)][1]))] for t in stamps])
    return (s > 70) & (s < tp.LENGTH - 70)


def main():
    root = Path(sys.argv[1])
    repo = Path(sys.argv[2])
    res = repo / 'results' / 'observability'
    res.mkdir(parents=True, exist_ok=True)
    rows = []
    for key, name in SEQS:
        d = root / key
        h, A = load(d / 'sweeps.tsv')
        c = {k: i for i, k in enumerate(h)}
        hs, S = load(d / 'shifts_all.tsv')
        cs = {k: i for i, k in enumerate(hs)}
        _, D = load(d / 'drift.tsv')
        m = np.ones(len(A), bool)
        ms = np.ones(len(S), bool)
        if key.startswith('tunnel'):
            pos = {round(t, 3): (x, y) for t, x, y in zip(A[:, c['stamp']], A[:, c['x']], A[:, c['y']])}
            m = inside_mask(A[:, c['stamp']], pos)
            ms = inside_mask(S[:, cs['stamp']], pos)

        def off(s):
            return np.minimum(S[ms, cs[f'off_-{s}']], S[ms, cs[f'off_{s}']])

        ax = A[m, c['sig_axial']] * 1e3
        r = {'seq': key, 'sweeps': int(m.sum()),
             'ax_p50_mm': np.median(ax), 'ax_max_mm': ax.max(),
             'lat_p50_mm': np.median(A[m, c['sig_lateral']]) * 1e3,
             'vert_p50_mm': np.median(A[m, c['sig_vertical']]) * 1e3,
             'points_p50': np.median(S[ms, cs['n_points']]),
             'off05_p50': np.median(off('0.5')), 'off05_min': off('0.5').min(),
             'off10_p50': np.median(off('10')), 'off10_min': off('10').min(),
             'lio_drift_end_m': D[-1, 1]}
        rows.append((name, r))
        dst = res / key
        dst.mkdir(exist_ok=True)
        for f in ('sweeps.tsv', 'shifts_all.tsv', 'drift.tsv', 'summary.json'):
            shutil.copy(d / f, dst / f)
    keys = list(rows[0][1].keys())
    with open(res / 'summary.tsv', 'w') as f:
        f.write('\t'.join(keys) + '\n')
        for _, r in rows:
            f.write('\t'.join(f'{r[k]:.4g}' if isinstance(r[k], float) else str(r[k]) for k in keys) + '\n')
    tex = [r'\begin{table}[t]', r'\centering',
           r'\caption{Observability of the clean OS1-32 sweeps at their true poses, independent of '
           r'any method: Cram\'er-Rao bound of the translation along and across the direction of '
           r'travel (range noise \SI{2}{\centi\meter}; along: median and maximum over the sweeps, across: median), and the '
           r'points that end farther than \SI{10}{\centi\meter} from the reference map when a sweep is '
           r'shifted along the direction of travel (median and minimum over every fifth sweep; '
           r'\num{16000} to \num{33000} points per sweep). '
           r'$^\ast$Inside the tunnel, farther than \SI{70}{\meter} from the portals.}',
           r'\label{tab:observability}', r'\footnotesize', r'\setlength{\tabcolsep}{3pt}',
           r'\begin{tabular}{@{}lrrrr@{}}', r'\toprule',
           r' & \multicolumn{2}{c}{Bound [mm]} & \multicolumn{2}{c}{Points off the map} \\',
           r'\cmidrule(lr){2-3}\cmidrule(l){4-5}',
           r'Sequence & along & across & \SI{0.5}{\meter} & \SI{10}{\meter} \\', r'\midrule']
    for name, r in rows:
        tex.append(f"{name} & {r['ax_p50_mm']:.1f} / {r['ax_max_mm']:.1f} & {r['lat_p50_mm']:.2f} & "
                   f"{r['off05_p50']:.0f} / {r['off05_min']:.0f} & {r['off10_p50']:.0f} / "
                   f"{r['off10_min']:.0f} \\\\")
    tex += [r'\bottomrule', r'\end{tabular}', r'\end{table}']
    (repo / 'paper' / 'tables' / 'observability.tex').write_text('\n'.join(tex) + '\n')
    print((res / 'summary.tsv').read_text())


if __name__ == '__main__':
    main()
