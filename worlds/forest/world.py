"""forest: 200 x 200 m of hilly woodland (+-5 m, slopes up to ~20%) with
about 1700 trees (porous sphere-cluster crowns, branches, some conifers),
bushes and fallen logs. Two cleared trails. Stresses unstructured,
vegetation-like clutter with few planar surfaces, plus pitch/roll on slopes."""
import math

import numpy as np

from mvsim_lio import props
from mvsim_lio.geom import dist_to_polyline, poisson_disk, smooth_closed
from mvsim_lio.traj import P
from mvsim_lio.world import Terrain, World

SEED = 202


def ground(x, y):
    return (3.0 * np.sin(x / 30.0) * np.cos(y / 25.0) + 1.5 * np.sin((x + 2 * y) / 40.0)
            + 0.25 * np.sin(x / 5.3) * np.cos(y / 4.7))


def trail_a():
    ctrl = [(-60, -10), (-50, -50), (-10, -65), (35, -55), (65, -25), (60, 20), (40, 55),
            (0, 62), (-35, 50), (-65, 25)]
    return smooth_closed(ctrl, 160, sigma=3.0)


def trail_b():
    xs = np.linspace(-88, 88, 60)
    return [(float(x), float(25 * math.sin(x / 35.0) + 0.15 * x)) for x in xs]


def make_world():
    rng = np.random.default_rng(SEED)
    w = World('forest', __doc__, extent=(-100, 100, -100, 100))
    w.set_terrain(Terrain(ground, -100, 100, -100, 100, 0.5, color='#6f8a4a'))
    g = lambda x, y: float(ground(x, y))  # noqa: E731
    ta = trail_a() + trail_a()[:1]
    tb = trail_b()

    def off_trail(x, y, m):
        return dist_to_polyline(x, y, ta) > m and dist_to_polyline(x, y, tb) > m

    for (x, y) in poisson_disk(rng, -98, 98, -98, 98, 4.0, keep=lambda x, y: off_trail(x, y, 3.2)):
        if rng.uniform() < 0.15:
            props.conifer(w, rng, x, y, g(x, y))
        else:
            props.tree(w, rng, x, y, g(x, y), leaves=30, branches=3)
    for _ in range(250):
        x, y = rng.uniform(-98, 98, size=2)
        if off_trail(x, y, 2.2):
            props.bush(w, rng, x, y, g(x, y))
    for _ in range(60):
        x, y = rng.uniform(-95, 95, size=2)
        if off_trail(x, y, 6.0):
            props.log(w, rng, x, y, g(x, y))
    return w


def trajectories(world):
    ta = trail_a()
    k0 = 0
    loop = [P(*ta[k0], v=1.2)] + [P(*ta[(k0 + i) % len(ta)]) for i in range(4, len(ta), 4)]
    loop.append(P(*ta[k0]))
    tb = trail_b()
    cross = [P(*tb[0], v=1.5)] + [P(*p) for p in tb[2::2]]
    return {
        'trail_loop': dict(items=loop, platform='legged', seed=1,
                           description='Legged robot, one lap of the main trail, 1.2 m/s'),
        'crossing': dict(items=cross, platform='ugv', seed=2,
                         description='UGV across the forest on the second trail, 1.5 m/s'),
    }
