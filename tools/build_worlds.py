#!/usr/bin/env python3
"""Builds every world: worlds/<name>/world.xml, trajectories/<traj>.tum (20 Hz
knots, the canonical trajectory definition), trajectories/stats.json and the
top-view plot trajectories.png. Also checks that no trajectory runs through
an obstacle.

Usage: tools/build_worlds.py [world_name ...]
"""
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'tools'))

from mvsim_lio import traj as T  # noqa: E402
from mvsim_lio.plot import plot_world  # noqa: E402

# Body clearance per platform: radius [m] and height band above the ground [m].
BODY = {'car': (0.95, 0.3, 1.6), 'ugv': (0.40, 0.1, 0.9), 'legged': (0.30, 0.1, 0.7),
        'handheld': (0.30, 0.3, 1.8), 'dog': (0.35, 0.15, 0.8)}


def load_world_module(name):
    path = ROOT / 'worlds' / name / 'world.py'
    spec = importlib.util.spec_from_file_location(f'world_{name}', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    u = np.clip(((px - ax) * dx + (py - ay) * dy) / max(L2, 1e-12), 0, 1)
    return np.hypot(px - (ax + u * dx), py - (ay + u * dy))


def _poly_dist(px, py, poly):
    """Distance from points to a polygon (0 inside)."""
    xs = np.array([p[0] for p in poly])
    ys = np.array([p[1] for p in poly])
    d = np.full(px.shape, np.inf)
    inside = np.zeros(px.shape, dtype=bool)
    n = len(xs)
    for i in range(n):
        j = (i + 1) % n
        d = np.minimum(d, _seg_dist(px, py, xs[i], ys[i], xs[j], ys[j]))
        cond = ((ys[i] > py) != (ys[j] > py)) & (
            px < (xs[j] - xs[i]) * (py - ys[i]) / (ys[j] - ys[i] + 1e-300) + xs[i])
        inside ^= cond
    d[inside] = 0.0
    return d


def clearance(world, tr, platform):
    """Min horizontal distance from the body axis to obstacles overlapping the
    body's height band, sampled at 2 Hz. Returns (min distance, where)."""
    r_body, z_lo, z_hi = BODY[platform]
    step = int(T.RATE_HZ / 2)
    x, y = tr['x'][::step], tr['y'][::step]
    zg = tr['z'][::step] - T.PLATFORMS[platform]['h_base']
    best = (np.inf, None)
    for fp in world.footprints + world.hidden:
        if fp.drivable:
            continue
        if fp.kind == 'circle':
            cx, cy, r = fp.data
            d = np.hypot(x - cx, y - cy) - r
        else:
            xs = [p[0] for p in fp.data]
            ys = [p[1] for p in fp.data]
            near = ((x > min(xs) - 3) & (x < max(xs) + 3) & (y > min(ys) - 3) &
                    (y < max(ys) + 3))
            if not near.any():
                continue
            d = np.full(x.shape, np.inf)
            d[near] = _poly_dist(x[near], y[near], fp.data)
        overlap = (fp.z_max > zg + z_lo) & (fp.z_min < zg + z_hi)
        d = np.where(overlap, d - r_body, np.inf)
        i = int(np.argmin(d))
        if d[i] < best[0]:
            best = (float(d[i]), (float(x[i]), float(y[i]), float(zg[i]), fp.color))
    return best


def build(name):
    mod = load_world_module(name)
    world = mod.make_world()
    wdir = ROOT / 'worlds' / name
    (wdir / 'world.xml').write_text(world.to_xml())
    tdir = wdir / 'trajectories'
    tdir.mkdir(exist_ok=True)
    trajs = mod.trajectories(world)
    stats = {}
    built = {}
    for tname, spec in trajs.items():
        tr = T.build(spec['items'], spec['platform'], terrain=world.terrain,
                     explicit_z=spec.get('explicit_z', False), seed=spec.get('seed', 0),
                     look_around_deg=spec.get('look_around_deg', 0.0),
                     pitch_scale=spec.get('pitch_scale', 1.0))
        # Knots at 20 Hz, or faster for carriers with a fast gait (the dog's
        # 4 Hz bounce needs 50 Hz knots to survive the spline).
        every = int(T.RATE_HZ / spec.get('knot_hz', 20))
        T.write_tum(tdir / f'{tname}.tum', tr, every=every)
        st = T.stats(tr)
        st['platform'] = spec['platform']
        st['description'] = spec.get('description', '')
        cl = clearance(world, tr, spec['platform'])
        st['min_clearance_m'] = round(cl[0], 3)
        stats[tname] = st
        built[tname] = tr
        flag = '' if cl[0] > 0 else f'  COLLISION near {cl[1]}'
        print(f'  {name}/{tname}: {st["duration"]:.0f} s, {st["length"]:.0f} m, '
              f'vmax {st["vmax"]:.1f} m/s, clearance {cl[0]:.2f} m{flag}')
    (tdir / 'stats.json').write_text(json.dumps(
        {k: {kk: (round(vv, 3) if isinstance(vv, float) else vv) for kk, vv in v.items()}
         for k, v in stats.items()}, indent=1) + '\n')
    plot_world(world, built, wdir / 'trajectories.png')
    print(f'{name}: {world.n_prims} primitives, {len(world.classes)} block classes, '
          f'world.xml {(wdir / "world.xml").stat().st_size / 1e6:.1f} MB')


def main():
    names = sys.argv[1:] or sorted(p.parent.name for p in (ROOT / 'worlds').glob('*/world.py'))
    for n in names:
        build(n)


if __name__ == '__main__':
    main()
