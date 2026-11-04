"""warehouse: a 90 x 60 m logistics hall, 10 m high with a ceiling and roof
trusses, six double rows of 8 m pallet racks (5 levels, randomly filled)
separated by 3.6 m aisles, loading docks with door openings, staging areas
with pallets and an office cabin. Indoor and repetitive: the aisles look
alike, with long, nearly identical views along them."""
import math

import numpy as np

from mvsim_lio import props
from mvsim_lio.traj import P
from mvsim_lio.world import World

SEED = 404
X0, X1, Y0, Y1, H = -45.0, 45.0, -30.0, 30.0, 10.0
RACK_X0 = -38.0
RACK_LEN = 25 * 2.7
ROWS_Y = [-12.0 + k * (2.3 + 3.6) for k in range(6)]  # south edge of each double row

SCREENSHOTS = [
    dict(point_to=(-5, 0, 0), distance=95, azimuth=-70, elevation=55, fov=50, cut_z=8.9),
    dict(point_to=(-10, -5, 2), distance=25, azimuth=-20, elevation=25, fov=70, cut_z=8.9),
]


def make_world():
    rng = np.random.default_rng(SEED)
    w = World('warehouse', __doc__, extent=(-55, 55, -40, 40))
    w.hplane(-60, 60, -45, 45, 0.0, color='#9a9a94')
    # Hall shell. The ceiling is left out of the screenshots' way by drawing
    # it last; it is a plane at z = H.
    docks = [dict(kind='door', pos=(x - X0) / (X1 - X0), width=3.2, z_min=0.0, z_max=4.0)
             for x in (-20.0, -12.0, -4.0, 4.0, 12.0, 20.0)]
    docks.append(dict(kind='door', pos=(36.0 - X0) / (X1 - X0), width=5.0, z_min=0.0, z_max=5.0))
    win = [dict(kind='window', pos=(x - X0) / (X1 - X0), width=6.0, z_min=7.0, z_max=8.6)
           for x in (-30.0, -15.0, 0.0, 15.0, 30.0)]
    w.wall(X0, Y0, X1, Y0, 0.0, H, 0.3, docks)
    w.wall(X0, Y1, X1, Y1, 0.0, H, 0.3, win)
    w.wall(X0, Y0, X0, Y1, 0.0, H, 0.3)
    w.wall(X1, Y0, X1, Y1, 0.0, H, 0.3)
    for x in np.arange(X0 + 6, X1, 6.0):
        w.box(x, 0, H - 1.0, 0.25, Y1 - Y0, 0.5, 0.0, '#707880', plot=False)  # roof trusses
    for x in np.arange(X0 + 15, X1 - 1, 15.0):
        for y in (-21.0, 25.0):
            w.box(x, y, 0.0, 0.45, 0.45, H, 0.0, '#c0c0b8')  # columns
    # Racks: six double rows (two back-to-back single rows each)
    for y in ROWS_Y:
        props.rack_row(w, rng, RACK_X0, y, RACK_LEN, 1.1, 5, 1.6, fill=0.7)
        props.rack_row(w, rng, RACK_X0, y + 1.2, RACK_LEN, 1.1, 5, 1.6, fill=0.7)
    # Staging area pallets (south, by the docks) and north
    for _ in range(40):
        x, y = rng.uniform(-30, 25), rng.uniform(-28, -23)
        w.box(x, y, 0.0, 1.2, 1.0, rng.uniform(0.3, 1.6), rng.uniform(-0.1, 0.1), props.C_PALLET)
    for _ in range(25):
        x, y = rng.uniform(-35, 25), rng.uniform(26.5, 28.8)
        w.box(x, y, 0.0, 1.2, 1.0, rng.uniform(0.3, 1.6), 0.0, props.C_PALLET)
    # Office cabin and a parked forklift
    w.box(-40.5, -25.5, 0.0, 8.0, 5.0, 3.0, 0.0, '#d0d8e0')
    w.box(-32.0, -16.0, 0.0, 2.4, 1.2, 2.1, 0.3, '#e0b020')
    w.box(-30.6, -15.6, 0.0, 1.0, 1.2, 0.2, 0.3, '#303030')
    w.hplane(X0, X1, Y0, Y1, H, color='#d8d8d0')
    return w


def aisle_y(k):
    """Center line of the aisle north of double row k."""
    return ROWS_Y[k] + 2.3 + 1.8


def trajectories(world):
    xe = RACK_X0 + RACK_LEN + 4.0  # east main aisle
    xw = RACK_X0 - 3.5  # west main aisle
    snake = [P(-38, -19, v=2.0), P(xw, -19), P(xw, aisle_y(0)), P(xe, aisle_y(0)),
             P(xe, aisle_y(2)), P(xw, aisle_y(2)), P(xw, aisle_y(4)), P(xe, aisle_y(4)),
             P(xe, -19), P(xw, -19), P(xw, aisle_y(0)), P(-10, aisle_y(0))]
    walk = [P(30, -19, v=1.3), P(xe, -19), P(xe, aisle_y(1)), P(xw, aisle_y(1)),
            P(xw, aisle_y(3)), P(xe, aisle_y(3)), P(xe, 22.5), P(-20, 22.5), P(-22, 21.0)]
    return {
        'aisles': dict(items=snake, platform='ugv', seed=1,
                       description='UGV, 2 m/s, through three aisles and around, with a revisit'),
        'walk': dict(items=walk, platform='handheld', explicit_z=True, seed=2,
                     look_around_deg=15.0,
                     description='Handheld rig through two other aisles and the north staging area'),
    }
