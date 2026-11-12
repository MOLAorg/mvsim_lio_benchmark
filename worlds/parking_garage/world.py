"""parking_garage: a three-level parking structure (64 x 40 m, levels at 0,
3.2 and 6.4 m, 2.9 m clearance) linked by two 20 m ramps (16% grade), with a
column grid, parapets, an open roof level, ~60% occupied bays, and a few
buildings and trees around it. Stresses multi-level geometry with ceilings,
ramps (pitch), repetitive columns, and vertical drift between floors."""
import math

import numpy as np

from mvsim_lio import props
from mvsim_lio.traj import P
from mvsim_lio.world import World

SEED = 1010
X0, X1, Y0, Y1 = -32.0, 32.0, -20.0, 20.0
LEVELS = [0.0, 3.2, 6.4]
SLAB = 0.3
RAMP_X0, RAMP_X1 = -10.0, 10.0
AISLE_Y, END_X = 10.5, 28.5

SCREENSHOTS = [
    dict(point_to=(0, 0, 3), distance=95, azimuth=-60, elevation=35, fov=50),
    dict(point_to=(0, -5, 0.5), distance=55, azimuth=-60, elevation=50, fov=55, cut_z=2.85),
]


def slab(w, z, holes):
    """Level slab (top at z) over the footprint, minus rectangular holes
    given as (x0, x1, y0, y1) strips touching the south or north edge."""
    # Split along x at the hole bounds, then cut each x-band in y.
    xs = sorted({X0, X1} | {h[0] for h in holes} | {h[1] for h in holes})
    for a, b in zip(xs[:-1], xs[1:]):
        ys = [Y0, Y1]
        for h in holes:
            if h[0] <= (a + b) / 2 <= h[1]:
                ys = [h[3], Y1] if h[2] <= Y0 + 1e-6 else [Y0, h[2]]
        with w.semantic('parking'):
            w.box((a + b) / 2, (ys[0] + ys[1]) / 2, z - SLAB, b - a, ys[1] - ys[0], SLAB, 0.0,
                  '#b0aca4', drivable=True, plot=False)


def parapets(w, z):
    for (xa, ya, xb, yb) in ((X0, Y0, X1, Y0), (X0, Y1, X1, Y1), (X0, Y0, X0, Y1), (X1, Y0, X1, Y1)):
        with w.semantic('wall'):
            w.box_between(xa, ya, xb, yb, z, 0.2, 1.1, '#c0bcb4')


def make_world():
    rng = np.random.default_rng(SEED)
    w = World('parking_garage', __doc__, extent=(-60, 60, -45, 45))
    with w.semantic('parking'):
        w.hplane(-70, 70, -55, 55, 0.0, color='#585858')
    hole_a = (RAMP_X0, RAMP_X1, Y0, -14.0)  # over ramp A (L0 -> L1), south strip
    hole_b = (RAMP_X0, RAMP_X1, 14.0, Y1)  # over ramp B (L1 -> L2), north strip
    slab(w, LEVELS[1], [hole_a])
    slab(w, LEVELS[2], [hole_b])
    with w.semantic('road'):
        w.ramp(RAMP_X0, -17.0, 0.0, RAMP_X1 - RAMP_X0, 6.0, LEVELS[1], color='#a8a49c',
               drivable=True)
        w.ramp(RAMP_X1, 17.0, LEVELS[1], RAMP_X1 - RAMP_X0, 6.0, LEVELS[2] - LEVELS[1],
               yaw=math.pi, color='#a8a49c', drivable=True)
    # Ground level walls: entrance on the west side, ventilation openings
    vents = [dict(kind='window', pos=p, width=6.0, z_min=1.2, z_max=2.4) for p in (0.2, 0.5, 0.8)]
    with w.semantic('wall'):
        w.wall(X0, Y0, X1, Y0, 0.0, LEVELS[1] - SLAB, 0.25, vents)
        w.wall(X0, Y1, X1, Y1, 0.0, LEVELS[1] - SLAB, 0.25, vents)
        w.wall(X1, Y0, X1, Y1, 0.0, LEVELS[1] - SLAB, 0.25, vents[:1])
        w.wall(X0, Y0, X0, Y1, 0.0, LEVELS[1] - SLAB, 0.25,
               [dict(kind='door', pos=(-10.5 - Y0) / (Y1 - Y0), width=7.0, z_min=0.0,
                     z_max=2.9)])
    for z in LEVELS[1:]:
        parapets(w, z)
    # Barriers along the slab openings
    with w.semantic('fence'):
        w.box_between(RAMP_X0 - 1, -14.0, RAMP_X1, -14.0, LEVELS[1], 0.2, 1.1, '#d0a030')
        w.box_between(RAMP_X0, 14.0, RAMP_X1 + 1, 14.0, LEVELS[2], 0.2, 1.1, '#d0a030')
    # Columns between levels
    for k, z in enumerate(LEVELS[:2]):
        for x in np.arange(-24.0, 24.1, 8.0):
            for y in (-7.0, 7.0):
                with w.instance(), w.semantic('column'):
                    w.box(x, y, z, 0.6, 0.6, LEVELS[k + 1] - SLAB - z, 0.0, '#c8c4bc')
    # Parked cars: central double row and the outer bays
    for k, z in enumerate(LEVELS):
        for x in np.arange(-23.75, 24.0, 2.5):
            for side in (-1, 1):
                if rng.uniform() < 0.6:
                    props.car(w, rng, x, side * 4.5, z, side * math.pi / 2 + math.pi)
                outer_blocked = abs(x) < 16.5 and (
                    (side < 0 and k in (0, 1)) or (side > 0 and k in (1, 2)))
                if not outer_blocked and rng.uniform() < 0.6:
                    props.car(w, rng, x, side * 16.8, z, side * math.pi / 2)
    for z in LEVELS[2:]:
        for x in (-20.0, 0.0, 20.0):
            props.lamp_post(w, x, 0.0, z, 0.0, height=5.0, arm=0.5)
    # Surroundings
    for (x, y, lx, ly, h) in ((0, 38, 50, 10, 12), (-50, 0, 10, 40, 18), (50, -5, 12, 30, 9)):
        props.building(w, rng, x, y, lx, ly, h)
    for x in np.arange(-30.0, 31.0, 10.0):
        props.tree(w, rng, x, -30.0, 0.0)
    return w


def trajectories(world):
    z0, z1, z2 = LEVELS
    ya = AISLE_Y

    def lap(z):
        # One loop around the level's aisle ring, counter-clockwise.
        return [P(END_X, -ya, z), P(END_X, ya, z), P(-END_X, ya, z), P(-END_X, -ya, z)]

    up = [P(-42.0, -10.5, z0, v=3.5), P(-36.0, -10.5, z0), P(-END_X, -ya, z0)] + lap(z0)
    # From the SW corner, go onto ramp A (south strip), up to level 1
    up += [P(-16.0, -ya, z0), P(RAMP_X0 - 3.0, -17.0, z0), P(RAMP_X0, -17.0, z0),
           P(RAMP_X1, -17.0, z1), P(RAMP_X1 + 3.0, -17.0, z1), P(16.0, -ya, z1)]
    up += [P(END_X, -ya, z1), P(END_X, ya, z1), P(16.0, ya, z1), P(RAMP_X1 + 3.0, 17.0, z1),
           P(RAMP_X1, 17.0, z1), P(RAMP_X0, 17.0, z2), P(RAMP_X0 - 3.0, 17.0, z2),
           P(-16.0, ya, z2), P(-END_X, ya, z2), P(-END_X, -ya, z2), P(END_X, -ya, z2),
           P(END_X, ya, z2), P(-16.0, ya, z2), P(RAMP_X0 - 3.0, 17.0, z2), P(RAMP_X0, 17.0, z2)]
    # Down ramp B to level 1, across to ramp A and down to the ground level
    up += [P(RAMP_X1, 17.0, z1), P(RAMP_X1 + 3.0, 17.0, z1), P(16.0, ya, z1), P(END_X, ya, z1),
           P(END_X, -ya, z1), P(16.0, -ya, z1), P(RAMP_X1 + 3.0, -17.0, z1),
           P(RAMP_X1, -17.0, z1), P(RAMP_X0, -17.0, z0), P(RAMP_X0 - 3.0, -17.0, z0),
           P(-16.0, -ya, z0), P(-END_X, -ya, z0), P(-36.0, -10.5, z0), P(-42.0, -10.5, z0)]
    walk = [P(20.0, -ya, z2, v=1.3), P(-END_X + 2, -ya, z2), P(-END_X + 2, ya, z2),
            P(-16.0, ya, z2), P(RAMP_X0 - 3.0, 17.0, z2), P(RAMP_X0, 17.0, z2),
            P(RAMP_X1, 17.0, z1), P(RAMP_X1 + 3.0, 17.0, z1), P(16.0, ya, z1),
            P(END_X - 2, ya, z1), P(END_X - 2, -ya, z1), P(16.0, -ya, z1),
            P(RAMP_X1 + 3.0, -17.0, z1), P(RAMP_X1, -17.0, z1), P(RAMP_X0, -17.0, z0),
            P(RAMP_X0 - 3.0, -17.0, z0), P(-16.0, -ya, z0), P(-END_X + 2, -ya, z0),
            P(-END_X + 2, ya, z0), P(END_X - 2, ya, z0), P(END_X - 2, -ya + 1, z0)]
    return {
        'walk_down': dict(items=walk, platform='handheld', explicit_z=True, seed=2,
                          look_around_deg=15.0,
                          description='Handheld rig from the roof down both ramps to the ground'),
        'up_down': dict(items=up, platform='car', explicit_z=True, seed=1,
                        description='Car, 3.5 m/s: ground lap, up two ramps, roof lap, back down'),
    }
