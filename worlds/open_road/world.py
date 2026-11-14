"""open_road: a 1.8 km two-lane road through open farmland at 9-14 m/s, with
little beside it: reflector posts every 30 m, utility poles, two guardrail
stretches, and lone trees, sheds, hedges and billboards far apart. The ground
constrains z, roll and pitch, but along-track motion rests on a handful of
posts and distant objects: the weak along-track constraint of real open roads.
A farmyard at the start gives the first 100 m more structure. Sparse moving
traffic, except in one sequence: a vehicle ahead and behind in the car's lane
and oncoming vehicles."""
import math

import numpy as np

from mvsim_lio import props, traffic
from mvsim_lio.traj import P
from mvsim_lio.world import Terrain, World

SCREENSHOTS = [
    dict(point_to=(40, -4, 1.0), distance=110, azimuth=-150, elevation=18, fov=60),
    dict(point_to=(420, 0, 1.0), distance=160, azimuth=-160, elevation=14, fov=60),
]
# Not part of the hosted data: rendered only when named (tools/generate.py --worlds).
HOSTED = False
SEED = 131
POST_SPACING = 30.0
TREE_SPACING = 90.0
BUILDING_SPACING = 120.0
X0, X1, Y0, Y1 = -60.0, 1860.0, -140.0, 140.0
LANE_Y = -2.0  # the driven lane, relative to the road axis; the oncoming one is at +2.0
POOL = ['car'] * 5 + ['van'] * 2 + ['truck'] * 2
TRAFFIC = {
    'drive_fast': dict(ahead=['car'], behind=['van'], headway=(2.5, 4.0), lanes=dict(
        on1=dict(mode='oncoming', kinds=['car', 'truck', 'car'], v=14.0))),
    'drive_slow': dict(ahead=['truck'], behind=['car', 'car'], headway=(2.0, 3.0), lanes=dict(
        on1=dict(mode='oncoming', kinds=['car', 'van', 'car', 'car'], v=13.0))),
}


def ground(x, y):
    return (0.8 * np.sin(x / 120.0) + 0.4 * np.sin(x / 31.0 + 0.7) * np.cos(y / 40.0)
            + 0.5 * np.sin(y / 33.0))


def road_y(x):
    # Gentle S bend between x = 900 and 1500, straight elsewhere.
    s = np.clip((x - 900.0) / 600.0, 0.0, 1.0)
    return 35.0 * (np.sin(2.0 * np.pi * s))


def make_world():
    rng = np.random.default_rng(SEED)
    w = World('open_road', __doc__, extent=(X0, X1, Y0, Y1))
    w.set_terrain(Terrain(ground, X0, X1, Y0, Y1, 2.0, color='#8fae6e'))
    g = lambda x, y: float(ground(x, y))  # noqa: E731
    # Reflector posts on both edges, utility poles every 70 m
    for x in np.arange(0.0, 1800.0, POST_SPACING):
        for side in (-1, 1):
            y = float(road_y(x)) + side * 5.0
            with w.instance(), w.semantic('pole'):
                w.cylinder(x, y, g(x, y) - 0.3, 0.05, 1.4, color='#d0d0d0')
    for x in np.arange(35.0, 1800.0, 70.0):
        y = float(road_y(x)) - 9.0
        with w.instance(), w.semantic('pole'):
            w.cylinder(x, y, g(x, y) - 0.3, 0.14, 8.0, color='#806a50')
    # Guardrail stretches on the left
    xs = np.concatenate([np.arange(300.0, 520.0, 4.0), np.arange(1100.0, 1300.0, 4.0)])
    for xa, xb in zip(xs[:-1], xs[1:]):
        if xb - xa > 5.0:
            continue
        ya, yb = float(road_y(xa)) + 5.5, float(road_y(xb)) + 5.5
        with w.semantic('fence'):
            w.box_between(xa, ya, xb, yb, g(xa, ya) + 0.4, 0.15, 0.6, '#b0b0b0')
    # Lone trees, far from the road
    for x in np.arange(40.0, 1800.0, TREE_SPACING):
        side = rng.choice((-1, 1))
        y = float(road_y(x)) + side * rng.uniform(25, 70)
        props.tree(w, rng, x + rng.uniform(-15, 15), y, g(x, y), height=9, crown=3.2)
    # Sheds / houses beside the road, and hedge blocks and billboards
    for x in np.arange(60.0, 1800.0, BUILDING_SPACING):
        side = rng.choice((-1, 1))
        y = float(road_y(x)) + side * rng.uniform(22, 45)
        with w.instance(), w.semantic('building'):
            w.box(x, y, g(x, y) - 0.3, rng.uniform(8, 16), rng.uniform(6, 10), rng.uniform(3.5, 6.0),
                  rng.uniform(-0.3, 0.3), '#a05040')
    for x in np.arange(20.0, 1800.0, 2 * BUILDING_SPACING):
        y = float(road_y(x)) + rng.choice((-1, 1)) * rng.uniform(9, 14)
        with w.instance(), w.semantic('vegetation'):
            w.box(x, y, g(x, y) - 0.3, rng.uniform(15, 30), 1.2, 2.0, 0.0, '#3a6a30')  # hedge
    for x in np.arange(150.0, 1800.0, 3 * BUILDING_SPACING):
        y = float(road_y(x)) + rng.choice((-1, 1)) * 8.0
        with w.instance(), w.semantic('other-structure'):
            w.box(x, y, g(x, y) - 0.3, 0.3, 6.0, 3.0, 0.0, '#c0c040')  # billboard facing the road
    # A farmyard beside the road where the sequences start from rest: the scans
    # of a start hold enough fixed structure along the road to follow it.
    rs = np.random.default_rng(SEED + 1)
    for x in np.arange(-45.0, 111.0, 12.0):
        y = float(road_y(x)) - 7.5
        props.lamp_post(w, x, y, g(x, y) - 0.3, math.pi / 2, height=7.0)
        y = float(road_y(x + 6.0)) - 11.5 + rs.uniform(-1.0, 1.0)
        props.tree(w, rs, x + 6.0, y, g(x + 6.0, y), leaves=25)
        y = float(road_y(x + 3.0)) + 6.5
        props.sign_post(w, x + 3.0, y, g(x + 3.0, y) - 0.3, math.pi / 2)
    y = float(road_y(5.0)) - 28.0
    props.building(w, rs, 5.0, y, 26.0, 10.0, 5.0, z=g(5.0, y))
    traffic.add_pool(w, POOL, X0 + 20.0, Y0 + 15.0)
    return w


def moving_blocks(tname, t, x):
    """Trajectories of the traffic blocks of a sequence (tools/generate.py);
    t, x: the knots of the car's trajectory."""
    if tname not in TRAFFIC:
        return {}
    road = dict(road_y=road_y, ground=ground, lanes=dict(ego=LANE_Y, on1=2.0), x_range=(X0 + 5.0, X1 - 5.0))
    return traffic.tracks(POOL, TRAFFIC[tname], road, tname, t, x)


def trajectories(world):
    straight = [P(0, road_y(0) - 2.0, v=14.0)] + [P(x, road_y(x) - 2.0) for x in range(100, 900, 100)]
    straight += [P(x, road_y(x) - 2.0) for x in range(900, 1810, 30)]
    slow = [P(0, road_y(0) - 2.0, v=9.0)] + [P(x, road_y(x) - 2.0) for x in range(60, 1810, 60)]
    return {
        'drive_fast': dict(items=straight, platform='car', seed=1,
                           description='Car along the road at 14 m/s, gentle S bend'),
        'drive_slow': dict(items=slow, platform='car', seed=2,
                           description='Same road at 9 m/s'),
        'drive_empty': dict(items=straight, platform='car', seed=3,
                            description='Car along the road at 14 m/s, no other vehicle'),
    }
