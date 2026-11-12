"""parking_lot: a 160 x 120 m open-air parking lot in front of a 120 m long
supermarket: five double rows of perpendicular bays (~70% occupied by cars of
four sizes), curbed end islands with trees, tall light poles, cart corrals,
hedges and a fence. Repetitive rows of similar cars; large open aisles."""
import math

import numpy as np

from mvsim_lio import props
from mvsim_lio.traj import P, Spin
from mvsim_lio.world import World

SEED = 808
ROWS = [23.0 - 17.0 * k for k in range(5)]  # double-row center lines (y)
BAY_X = [-55.0 + 2.5 * (k + 0.5) for k in range(44)]

SCREENSHOTS = [
    dict(point_to=(0, -5, 0), distance=150, azimuth=-70, elevation=40, fov=50),
    dict(point_to=(-20, -2.5, 1.0), distance=22, azimuth=-150, elevation=15, fov=65),
]


def make_world():
    rng = np.random.default_rng(SEED)
    w = World('parking_lot', __doc__, extent=(-80, 80, -65, 62))
    with w.semantic('parking'):
        w.hplane(-85, 85, -70, 65, 0.0, color='#484848')
    props.building(w, rng, 0.0, 46.0, 120.0, 20.0, 8.0, pilaster=5.0, color='#c8c0b0',
                   entrance_side=0)
    with w.semantic('sidewalk'):
        w.box(0.0, 34.5, 0.0, 120.0, 3.0, 0.15, 0.0, '#a8a8a0')  # store front sidewalk
    for x in (-40.0, -10.0, 20.0, 45.0):
        with w.instance(), w.semantic('other-structure'):
            w.box(x, 34.5, 0.15, 4.0, 1.0, 1.0, 0.0, '#909090')  # cart corral by the store
    for yc in ROWS:
        for side in (-1, 1):
            for x in BAY_X:
                if rng.uniform() < 0.7:
                    yy = yc + side * 2.6 + rng.uniform(-0.15, 0.15)
                    props.car(w, rng, x + rng.uniform(-0.1, 0.1), yy, 0.0,
                              side * math.pi / 2 + math.pi + rng.uniform(-0.04, 0.04))
        for xi in (-58.0, 58.0):
            with w.semantic('other-ground'):
                w.box(xi, yc, 0.0, 4.0, 10.0, 0.15, 0.0, '#a8a8a0')  # end island
            props.tree(w, rng, xi, yc - 2.5, 0.15, height=rng.uniform(6, 8), crown=2.0)
            props.lamp_post(w, xi, yc + 2.5, 0.15, 0.0, height=9.0, arm=0.8)
        for x in (-25.0, 5.0, 35.0):
            with w.instance(), w.semantic('pole'):
                w.cylinder(x, yc, 0.0, 0.15, 10.0, color=props.C_METAL)  # light pole
                w.box(x, yc, 9.8, 1.2, 0.6, 0.3, 0.0, '#e8e8d0')
    # Perimeter
    for x in np.arange(-78.0, 78.0, 1.3):
        props.bush(w, rng, x, -62.0, 0.0, size=0.9)
    for y in np.arange(-60.0, 30.0, 1.3):
        props.bush(w, rng, -76.0, y, 0.0, size=0.9)
    props.fence(w, 76, -60, 76, 30, lambda x, y: 0.0, spacing=2.5, height=1.8, rails=3)
    return w


def trajectories(world):
    xe, xw = 64.0, -64.0
    aisles = [31.5] + [yc - 8.5 for yc in ROWS]
    search = []
    for i, y in enumerate(aisles):
        search += [P(xw, y), P(xe, y)] if i % 2 == 0 else [P(xe, y), P(xw, y)]
    search += [P(xw, 14.5), P(-20, 14.5)]
    search[0].v = 4.5
    ugv = [P(-50, 31.5, v=2.0), P(50, 31.5), P(xe, 31.5), P(xe, -2.5), Spin(360, 5.0),
           P(-30, -2.5), Spin(-180, 3.0), P(xe - 2, -2.5),
           P(xe - 2, -36.5), P(xw + 2, -36.5), P(xw + 2, 31.5), P(-50, 31.5)]
    return {
        'search': dict(items=search, platform='car', seed=1,
                       description='Car, 4.5 m/s, snaking through all six aisles, then a revisit'),
        'ugv_loop': dict(items=ugv, platform='ugv', seed=2,
                         description='UGV, 2 m/s, store front and aisles, with in-place spins'),
    }
