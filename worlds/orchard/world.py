"""orchard: 23 rows x 31 fruit trees (5 m between rows, 4 m within a row)
on a gently sloped field (3%), with trellis posts, a perimeter farm track, a
hedge, a fence, a shed and a water tank. Highly repetitive along and across
the rows (aliasing); the LiDAR sees trunks under the crowns."""
import math

import numpy as np

from mvsim_lio import props
from mvsim_lio.traj import P
from mvsim_lio.world import Terrain, World

SEED = 707
ROW_Y = [-55.0 + 5.0 * k for k in range(23)]
TREE_X = [-60.0 + 4.0 * k for k in range(31)]


def ground(x, y):
    return 0.03 * x + 0.6 * np.sin(y / 25.0) + 0.2 * np.sin(x / 13.0) * np.cos(y / 9.0)


def make_world():
    rng = np.random.default_rng(SEED)
    w = World('orchard', __doc__, extent=(-85, 85, -75, 75))
    w.set_terrain(Terrain(ground, -90, 90, -80, 80, 1.0, color='#8aa860'))
    g = lambda x, y: float(ground(x, y))  # noqa: E731
    for y in ROW_Y:
        for k, x in enumerate(TREE_X):
            xx, yy = x + rng.uniform(-0.15, 0.15), y + rng.uniform(-0.15, 0.15)
            if rng.uniform() < 0.04:
                continue  # missing tree
            props.tree(w, rng, xx, yy, g(xx, yy), height=rng.uniform(3.4, 4.2),
                       crown=rng.uniform(1.3, 1.6), trunk_r=rng.uniform(0.08, 0.14), leaves=25,
                       branches=3)
            if k % 3 == 1:
                w.cylinder(x + 2.0, y, g(x + 2.0, y) - 0.3, 0.05, 2.8, color='#a09070')  # post
    # Perimeter: hedge (east), fence (west), shed and water tank (north-east)
    for y in np.arange(-70.0, 70.0, 1.2):
        props.bush(w, rng, 80.0, y, g(80.0, y), size=1.2)
    props.fence(w, -80, -70, -80, 70, g, spacing=3.0)
    w.box(73.0, 70.5, g(73, 70.5) - 0.3, 8.0, 6.0, 4.0, 0.0, '#a04838')
    w.cylinder(60.0, 71.0, g(60, 71) - 0.2, 2.5, 4.0, color='#8090a0')
    return w


def trajectories(world):
    xe, xw = 66.0, -66.0
    rows = []
    gaps = [ROW_Y[k] + 2.5 for k in (2, 4, 6, 8, 10)]
    for i, y in enumerate(gaps):
        if i % 2 == 0:
            rows += [P(xw, y), P(xe, y)]
        else:
            rows += [P(xe, y), P(xw, y)]
    rows[0].v = 1.5
    headland = [P(-68, -64, v=2.5), P(68, -64), P(68, 64), P(-68, 64), P(-68, -62)]
    return {
        'rows': dict(items=rows, platform='ugv', seed=1,
                     description='UGV, 1.5 m/s, along five inter-row lanes with headland turns'),
        'headland': dict(items=headland, platform='ugv', seed=2,
                         description='UGV, 2.5 m/s, one lap of the perimeter track'),
    }
