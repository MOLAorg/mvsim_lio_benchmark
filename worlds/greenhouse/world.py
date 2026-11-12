"""greenhouse: a 96 x 40 m multi-span greenhouse (5 spans of 8 m, gutters at
4.5 m, gable roofs up to 5.8 m seen from inside), a post grid every 4 m,
trusses, and long crop rows: porous tomato-like hedges in three spans and
benches with pots in two. Repetitive along the 87 m aisles, where every
section looks the same; indoor with a sloped ceiling."""
import math

import numpy as np

from mvsim_lio import props
from mvsim_lio.traj import P, Stop
from mvsim_lio.world import World

SEED = 606
X0, X1 = -48.0, 48.0
SPANS = [-16.0, -8.0, 0.0, 8.0, 16.0]  # span centers (y)
HG, HR = 4.5, 5.8  # gutter and ridge heights
CROP_X0, CROP_X1 = -43.0, 46.0

SCREENSHOTS = [
    dict(point_to=(0, 0, 0), distance=90, azimuth=-65, elevation=45, fov=50, cut_z=4.4),
    dict(point_to=(-30, -16, 1.5), distance=12, azimuth=-170, elevation=8, fov=75),
]


def hedge(w, rng, y, x0, x1):
    with w.instance(), w.semantic('vegetation'):
        for x in np.arange(x0, x1, 0.35):
            for zc in (0.45, 1.05, 1.65, 2.25):
                w.sphere(x + rng.uniform(-0.1, 0.1), y + rng.uniform(-0.12, 0.12),
                         zc + rng.uniform(-0.15, 0.15), rng.uniform(0.18, 0.3),
                         color=props.C_LEAF)
        for x in np.arange(x0, x1, 2.0):
            w.cylinder(x, y, 0.0, 0.02, 2.8, color='#a0a0a0')  # support strings / stems


def make_world():
    rng = np.random.default_rng(SEED)
    w = World('greenhouse', __doc__, extent=(-55, 55, -26, 26))
    with w.semantic('floor'):
        w.hplane(-60, 60, -30, 30, 0.0, color='#a09880')
        w.box(-46.0, 0.0, 0.0, 4.0, 40.0, 0.05, 0.0, '#c0c0c0')  # concrete main corridor
    gutters = [c - 4.0 for c in SPANS] + [SPANS[-1] + 4.0]
    # Shell
    with w.semantic('wall'):
        w.wall(X0, gutters[0], X1, gutters[0], 0.0, HG, 0.1, color='#d8e8e8')
        w.wall(X0, gutters[-1], X1, gutters[-1], 0.0, HG, 0.1, color='#d8e8e8')
        w.wall(X0, gutters[0], X0, gutters[-1], 0.0, HG, 0.1,
               [dict(kind='door', pos=0.5, width=3.0, z_min=0.0, z_max=3.0)], color='#d8e8e8')
        w.wall(X1, gutters[0], X1, gutters[-1], 0.0, HG, 0.1, color='#d8e8e8')
    for yg in gutters:
        with w.semantic('other-structure'):
            w.box(0.0, yg, HG, X1 - X0, 0.3, 0.2, 0.0, '#b0b0b0')  # gutter
        for x in np.arange(X0, X1 + 0.1, 4.0):
            with w.instance(), w.semantic('column'):
                w.box(x, yg, 0.0, 0.12, 0.12, HG, 0.0, '#b8b8b8')  # posts
    with w.semantic('other-structure'):
        for x in np.arange(X0 + 4.0, X1, 4.0):
            w.box(x, 0.0, HG - 0.3, 0.08, gutters[-1] - gutters[0], 0.15, 0.0, '#b8b8b8',
                  plot=False)  # trusses
    # Gable roofs: per span, two inverted wedges whose sloped faces look down.
    with w.semantic('ceiling'):
        for yc in SPANS:
            for yaw in (-math.pi / 2, math.pi / 2):
                w.ramp(0.0, yc, HR, 4.0, X1 - X0, HR - HG, yaw=yaw, roll=math.pi,
                       color='#e8f0f0', plot=False)
    # Crops
    for k, yc in enumerate(SPANS):
        for side in (-1, 1):
            yr = yc + side * 2.0
            if k in (1, 3):
                # Benches with pots
                for x in np.arange(CROP_X0, CROP_X1 - 3.9, 4.2):
                    with w.instance(), w.semantic('table'):
                        w.box(x + 2.0, yr, 0.0, 4.0, 1.4, 0.8, 0.0, '#909aa0')
                    for px in np.arange(x + 0.3, x + 3.9, 0.45):
                        for py in (-0.45, 0.0, 0.45):
                            with w.instance(), w.semantic('other-object'):  # pot and plant
                                w.cylinder(px, yr + py, 0.8, 0.12, 0.18, color='#a05030')
                                if rng.uniform() < 0.8:
                                    with w.semantic('vegetation'):
                                        w.sphere(px, yr + py, 1.05 + rng.uniform(0, 0.1),
                                                 rng.uniform(0.1, 0.2), color=props.C_LEAF)
            else:
                for dy in (-0.45, 0.45):
                    hedge(w, rng, yr + dy, CROP_X0, CROP_X1)
        for dy in (-0.9, 0.9):
            # Heating pipe rails along the center path
            with w.semantic('other-structure'):
                w.cylinder(CROP_X0, yc + dy, 0.06, 0.025, CROP_X1 - CROP_X0, yaw=0.0,
                           pitch=math.pi / 2, color='#c0c0c0')
    return w


def trajectories(world):
    xc = -46.0
    xe = CROP_X1 - 1.0

    def visit(yc, x_end):
        return [P(xc, yc), P(-40.0, yc), P(x_end, yc), Stop(1.0), P(-40.0, yc), P(xc, yc)]

    aisles = ([P(xc, -18.0, v=1.2)] + visit(SPANS[0], xe) + visit(SPANS[3], xe) +
              [P(xc, -18.0)])
    walk = [P(xc, 18.0, v=1.1)] + visit(SPANS[4], 0.0) + visit(SPANS[1], 20.0) + [P(xc, -2.0)]
    return {
        'aisles': dict(items=aisles, platform='ugv', seed=1,
                       description='UGV, 1.2 m/s, down and back two 87 m aisles'),
        'walk': dict(items=walk, platform='handheld', explicit_z=True, seed=2,
                     look_around_deg=15.0,
                     description='Handheld rig, partial walks along two other aisles'),
    }
