"""rough_terrain: 260 x 260 m of steep, bumpy hills and valleys (+-10 m,
slopes up to ~25 deg, 7 m bumps) with two craters, ~400 boulders of 0.3-1.6 m
and a few dead trees. Stresses large, fast-changing pitch and roll, and a
scene made almost only of terrain and rocks (no planes or poles)."""
import math

import numpy as np

from mvsim_lio.geom import dist_to_polyline
from mvsim_lio.traj import P
from mvsim_lio.world import Terrain, World

SEED = 1212

PATH_A = [(-100, -80), (-70, -40), (-60, 10), (-20, 30), (10, -10), (40, -30), (70, 0), (60, 50),
          (20, 80), (-30, 70), (-70, 60), (-100, 20)]
PATH_B = [(-110, 100), (-50, 95), (0, 60), (30, 20), (60, -40), (90, -90), (110, -110)]


def ground(x, y):
    z = (7.0 * np.sin(x / 30.0) * np.cos(y / 35.0) + 3.0 * np.sin((x - y) / 20.0)
         + 0.6 * np.sin(x / 7.1) * np.sin(y / 6.3))
    for cx, cy, r, d in ((40, -50, 14, 3.0), (-60, 60, 10, 2.0)):
        z = z - d * np.exp(-((x - cx) ** 2 + (y - cy) ** 2) / (2 * (r / 2) ** 2))
    return z


def make_world():
    rng = np.random.default_rng(SEED)
    w = World('rough_terrain', __doc__, extent=(-130, 130, -130, 130))
    w.set_terrain(Terrain(ground, -130, 130, -130, 130, 0.5, color='#a09070'))
    g = lambda x, y: float(ground(x, y))  # noqa: E731
    pa, pb = PATH_A + PATH_A[:1], PATH_B
    n = 0
    while n < 400:
        x, y = rng.uniform(-125, 125, size=2)
        r = float(rng.choice([rng.uniform(0.3, 0.7), rng.uniform(0.7, 1.6)], p=[0.7, 0.3]))
        if dist_to_polyline(x, y, pa) < 3.0 + r or dist_to_polyline(x, y, pb) < 3.0 + r:
            continue
        w.sphere(x, y, g(x, y) + 0.3 * r, r, color='#8a8680')
        n += 1
        if rng.uniform() < 0.2:  # small cluster around big rocks
            for _ in range(3):
                a = rng.uniform(0, 2 * math.pi)
                rr = r * rng.uniform(0.3, 0.6)
                qx, qy = x + math.cos(a) * (r + rr), y + math.sin(a) * (r + rr)
                if dist_to_polyline(qx, qy, pa) > 3.0 and dist_to_polyline(qx, qy, pb) > 3.0:
                    w.sphere(qx, qy, g(qx, qy) + 0.2 * rr, rr, color='#7a7670')
    for _ in range(15):
        x, y = rng.uniform(-120, 120, size=2)
        if dist_to_polyline(x, y, pa) < 5 or dist_to_polyline(x, y, pb) < 5:
            continue
        w.cylinder(x, y, g(x, y) - 0.3, 0.2, rng.uniform(3, 6), color='#5a4a3a')
        for _ in range(3):
            w.cylinder(x, y, g(x, y) + rng.uniform(1.5, 3.0), 0.07, rng.uniform(1, 2),
                       yaw=rng.uniform(-math.pi, math.pi), pitch=math.radians(rng.uniform(30, 70)),
                       color='#5a4a3a')
    return w


def trajectories(world):
    a = [P(*PATH_A[0], v=1.5)] + [P(*p) for p in PATH_A[1:]] + [P(*PATH_A[0])]
    b = [P(*PATH_B[0], v=3.0)] + [P(*p) for p in PATH_B[1:]]
    return {
        'hill_loop': dict(items=a, platform='ugv', seed=1,
                          description='UGV, 1.5 m/s, a loop over ridges and through a crater'),
        'traverse_fast': dict(items=b, platform='ugv', seed=2,
                              description='UGV, 3 m/s, diagonal traverse across the hills'),
    }
