"""open_highway: 7 km of divided highway at 14-25 m/s. Ordinary stretches
(median barrier, reflector posts, lamp posts, sign gantries, overpasses,
trees, buildings, gentle bends) alternate with stretches whose only geometry
is invariant along the road, as on a long bridge or between noise walls: flat
deck, continuous barriers or walls, nothing else in range. Along-track motion
is unobservable there and LiDAR odometry under-estimates the distance
traveled. The first 4.2 km are for training trajectories; the last 2.6 km,
with other kinds of invariant stretch, for a held-out one. Every trajectory
but one has moving traffic: vehicles ahead of and behind the car that stop and
start with it, the two lanes beside it as queues or as faster and slower
streams, and oncoming vehicles on the other carriageway. A vehicle that keeps
its place near the car stalls LiDAR odometry that has no other motion source,
in the ordinary stretches too."""
import math
import sys

import numpy as np

from mvsim_lio import props, traffic
from mvsim_lio import traj as T
from mvsim_lio.traj import P, Stop
from mvsim_lio.world import Terrain, World, paint_road

# Highway speeds: the stock 'car' carrier is capped at 15 m/s. The carrier is
# registered here (with its body for the clearance check of
# tools/build_worlds.py) so that no shared file changes.
T.PLATFORMS.setdefault('car_highway', dict(T.PLATFORMS['car'], vmax=30.0))
for _m in list(sys.modules.values()):
    if isinstance(getattr(_m, 'BODY', None), dict) and 'car' in _m.BODY:
        _m.BODY.setdefault('car_highway', _m.BODY['car'])

SCREENSHOTS = [
    dict(point_to=(45, 0, 2.0), distance=110, azimuth=-150, elevation=18, fov=60),
    dict(point_to=(1040, 1, 1.0), distance=90, azimuth=-165, elevation=12, fov=60),
]
# Not part of the hosted data: rendered only when named (tools/generate.py --worlds).
HOSTED = False
SEED = 977
# (x0, x1, kind). Kinds other than 'rich' are invariant along the road:
#   bridge        low outer barriers and a median barrier
#   walls         4.5 m noise walls on both sides and a median barrier
#   bridge_lamps  bridge with a lamp post on the median every 50 m
#   railposts     railings (posts every 2 m, two rails) and a median barrier
SECTIONS = [
    (0, 900, 'rich'), (900, 1180, 'bridge'), (1180, 2600, 'rich'), (2600, 2820, 'walls'),
    (2820, 4200, 'rich'),
    (4200, 5200, 'rich'), (5200, 5450, 'bridge_lamps'), (5450, 6300, 'rich'),
    (6300, 6450, 'railposts'), (6450, 7000, 'rich'),
]
TRAIN_X = (0.0, 4200.0)
HELDOUT_X = (4400.0, 7000.0)
# Gentle bends (x0, x1, lateral amplitude), inside the ordinary stretches only.
BENDS = [(200, 800, 25.0), (1400, 2100, -35.0), (2950, 3450, 20.0), (3700, 4150, -15.0),
         (4500, 5100, -30.0), (5600, 6200, 30.0), (6500, 6950, 15.0)]
X_END = SECTIONS[-1][1]
X0, X1, Y0, Y1 = -60.0, X_END + 60.0, -130.0, 130.0
LANE_Y = -3.5  # the driven lane, relative to the road axis
Y_OUT_R, Y_MED, Y_OUT_L = -8.0, 1.0, 10.0  # outer barriers and median, relative to the road axis
RAMP = 120.0  # the terrain flattens over this length before an invariant stretch
# Lanes, relative to the road axis: the driven one, the two beside it (narrow,
# 2.9 m apart) and the two of the other carriageway.
LANES = dict(ego=LANE_Y, left=-0.6, right=-6.4, on1=3.6, on2=6.8)
# Vehicles available to the traffic of a trajectory (blocks parked below the
# terrain when not in use).
POOL = ['car'] * 18 + ['van'] * 5 + ['truck'] * 4 + ['boxtruck'] * 3
_ONCOMING = dict(on1=dict(mode='oncoming', kinds=['car', 'truck', 'car', 'van'], v=26.0),
                 on2=dict(mode='oncoming', kinds=['truck', 'car', 'car'], v=23.0))
# Trucks are too wide for the lanes beside the car.
TRAFFIC = {
    'cruise_25': dict(ahead=['car'], behind=['van'], headway=(1.8, 3.0), lanes=dict(
        left=dict(mode='flow', kinds=['car', 'car', 'van'], kappa=0.15),
        right=dict(mode='flow', kinds=['car', 'van', 'car'], kappa=-0.2), **_ONCOMING)),
    'varied_18': dict(ahead=['truck', 'car'], behind=['car', 'car'], lanes=dict(
        left=dict(mode='flow', kinds=['car'] * 4, kappa=0.2),
        right=dict(mode='flow', kinds=['van', 'car', 'car', 'car'], kappa=-0.15), **_ONCOMING)),
    'jam_14': dict(ahead=[('boxtruck', 1.0, 6.0), 'car', 'truck'], behind=['car', 'car', 'van'], lanes=dict(
        left=dict(mode='platoon', kinds=['car'] * 6),
        right=dict(mode='platoon', kinds=['car', 'van', 'car', 'car', 'car', 'car']), **_ONCOMING)),
    'bridge_start_20': dict(ahead=['truck', 'car'], behind=['car', 'car'], lanes=dict(
        left=dict(mode='platoon', kinds=['car'] * 4),
        right=dict(mode='flow', kinds=['car', 'van', 'car'], kappa=-0.2), **_ONCOMING)),
    'walls_start_14': dict(ahead=['car', 'van'], behind=['truck', 'car'], lanes=dict(
        left=dict(mode='flow', kinds=['car'] * 3, kappa=0.25),
        right=dict(mode='platoon', kinds=['car', 'car', 'van', 'car']), **_ONCOMING)),
    'heldout_21': dict(ahead=['car'], behind=['car', 'truck'], lanes=dict(
        left=dict(mode='flow', kinds=['van', 'car', 'car'], kappa=0.18),
        right=dict(mode='flow', kinds=['car'] * 3, kappa=-0.25), **_ONCOMING)),
    'heldout_start_17': dict(ahead=['van', 'truck'], behind=['car', 'van'], lanes=dict(
        left=dict(mode='platoon', kinds=['car'] * 5),
        right=dict(mode='platoon', kinds=['car', 'car', 'van', 'car', 'car']), **_ONCOMING)),
}


def road_y(x):
    x = np.asarray(x, dtype=float)
    y = np.zeros_like(x)
    for xa, xb, amp in BENDS:
        u = np.clip((x - xa) / (xb - xa), 0.0, 1.0)
        y = y + amp * np.sin(np.pi * u) ** 2
    return y


def _flat_weight(x):
    """1 in ordinary stretches, 0 in the invariant ones, smooth in between."""
    x = np.asarray(x, dtype=float)
    w = np.ones_like(x)
    for x0, x1, kind in SECTIONS:
        if kind == 'rich':
            continue
        u = np.clip(np.minimum(x - (x0 - RAMP), (x1 + RAMP) - x) / RAMP, 0.0, 1.0)
        w = np.minimum(w, 0.5 * (1.0 + np.cos(np.pi * u)))
    return w


def ground(x, y):
    und = (0.8 * np.sin(x / 120.0) + 0.4 * np.sin(x / 31.0 + 0.7) * np.cos(y / 40.0)
           + 0.5 * np.sin(y / 33.0))
    return _flat_weight(x) * und


def _barrier(w, g, x0, x1, off, height, width, color, step):
    """Continuous barrier at lateral offset `off` from the road axis."""
    xs = np.arange(x0, x1 + 1e-6, step)
    for xa, xb in zip(xs[:-1], xs[1:]):
        ya, yb = float(road_y(xa)) + off, float(road_y(xb)) + off
        with w.semantic('fence'):
            w.box_between(xa, ya, xb, yb, min(g(xa, ya), g(xb, yb)) - 0.3, width, height + 0.3, color)


def _gantry(w, g, x):
    yr, ym = float(road_y(x)) + Y_OUT_R - 1.5, float(road_y(x)) + Y_MED
    z = g(x, ym)
    with w.instance(), w.semantic('other-structure'):
        for y in (yr, ym):
            w.cylinder(x, y, g(x, y) - 0.3, 0.15, 7.3, color=props.C_METAL)
        w.box(x, (yr + ym) / 2, z + 6.4, 0.4, abs(ym - yr), 0.5, 0.0, props.C_METAL, plot=False)
        with w.semantic('traffic-sign'):
            w.box(x, (yr + ym) / 2 - 2.0, z + 4.9, 0.15, 3.5, 1.5, 0.0, '#3060a0', plot=False)


def _overpass(w, g, x):
    yc = float(road_y(x))
    z = g(x, yc)
    with w.instance(), w.semantic('other-structure'):
        for off in (Y_OUT_R - 4.0, Y_MED, Y_OUT_L + 4.0):
            w.box(x, yc + off, g(x, yc + off) - 0.3, 1.2, 0.5 if off == Y_MED else 1.2, 6.0, 0.0, props.C_CONCRETE)
        w.box(x, yc + 1.0, z + 5.6, 12.0, 70.0, 1.2, 0.0, props.C_CONCRETE, plot=False)
        for off, s in ((Y_OUT_R - 4.6, -1), (Y_OUT_L + 4.6, 1)):
            # Embankment abutments beside the road.
            with w.semantic('terrain'):
                w.box(x, yc + off + s * 14.0, g(x, yc + off) - 0.3, 12.0, 28.0, 5.9, 0.0, '#7f9a66')


def _rich(w, g, rng, x0, x1):
    yr = lambda x, off: float(road_y(x)) + off  # noqa: E731
    _barrier(w, g, x0, x1, Y_MED, 0.8, 0.6, props.C_CONCRETE, 4.0)
    for x in np.arange(x0 + 15.0, x1, 30.0):
        for off in (Y_OUT_R + 0.4, Y_OUT_L - 1.0):
            with w.instance(), w.semantic('pole'):
                w.cylinder(x, yr(x, off), g(x, yr(x, off)) - 0.3, 0.05, 1.4, color='#d0d0d0')
    for x in np.arange(x0 + 25.0, x1 - 10.0, 50.0):
        props.lamp_post(w, x, yr(x, Y_MED), g(x, yr(x, Y_MED)) + 0.8, math.pi / 2, height=8.0)
    for x in np.arange(x0 + 35.0, x1, 70.0):
        with w.instance(), w.semantic('pole'):
            w.cylinder(x, yr(x, Y_OUT_R - 3.0), g(x, yr(x, Y_OUT_R - 3.0)) - 0.3, 0.14, 8.0, color='#806a50')
    for x in np.arange(x0 + 180.0, x1 - 60.0, 420.0):
        _gantry(w, g, x + rng.uniform(-30, 30))
    for x in np.arange(x0 + 330.0, x1 - 150.0, 700.0):
        _overpass(w, g, x + rng.uniform(-40, 40))
    for x in np.arange(x0 + 20.0, x1, 40.0):
        for side in (-1, 1):
            if rng.uniform() < 0.3:
                continue
            xt = x + rng.uniform(-15, 15)
            y = yr(xt, side * rng.uniform(16, 60))
            if rng.uniform() < 0.3:
                props.conifer(w, rng, xt, y, g(xt, y))
            else:
                props.tree(w, rng, xt, y, g(xt, y), leaves=25)
    for x in np.arange(x0 + 60.0, x1, 150.0):
        side = rng.choice((-1, 1))
        y = yr(x, side * rng.uniform(28, 55))
        with w.instance(), w.semantic('building'):
            w.box(x, y, g(x, y) - 0.3, rng.uniform(8, 16), rng.uniform(6, 10), rng.uniform(3.5, 6.0),
                  rng.uniform(-0.3, 0.3), '#a05040')
    for x in np.arange(x0 + 100.0, x1 - 40.0, 240.0):
        y = yr(x, rng.choice((Y_OUT_R - 5.0, Y_OUT_L + 5.0)) + rng.uniform(-2, 2))
        with w.instance(), w.semantic('vegetation'):
            w.box(x, y, g(x, y) - 0.3, rng.uniform(15, 30), 1.2, 2.0, 0.0, '#3a6a30')  # hedge
    for x in np.arange(x0 + 140.0, x1 - 40.0, 360.0):
        y = yr(x, rng.choice((Y_OUT_R - 4.0, Y_OUT_L + 4.0)))
        with w.instance(), w.semantic('other-structure'):
            w.box(x, y, g(x, y) - 0.3, 0.3, 6.0, 3.0, 0.0, '#c0c040')  # billboard facing the road


def _start_area(w, g, rng, x0):
    """A rest area beside the road, where sequences start from rest: the scans
    of a start hold enough fixed structure along the road to follow it."""
    yr = lambda x, off: float(road_y(x)) + off  # noqa: E731
    for x in np.arange(x0 - 45.0, x0 + 111.0, 12.0):
        y = yr(x, Y_OUT_R - 2.5)
        props.lamp_post(w, x, y, g(x, y) - 0.3, math.pi / 2, height=7.0)
        y = yr(x + 6.0, Y_OUT_R - 5.5 + rng.uniform(-1.0, 1.0))
        props.tree(w, rng, x + 6.0, y, g(x + 6.0, y), leaves=25)
        y = yr(x + 3.0, Y_OUT_L + 1.5)
        props.sign_post(w, x + 3.0, y, g(x + 3.0, y) - 0.3, math.pi / 2)
    y = yr(x0 + 5.0, Y_OUT_R - 22.0)
    props.building(w, rng, x0 + 5.0, y, 26.0, 10.0, 5.0, z=g(x0 + 5.0, y))
    _gantry(w, g, x0 + 45.0)


def _paint(X, Y, rgb):
    """The two carriageways in the ground texture: the three lanes of the
    driven one and the two of the other, with edge lines and dashed lane
    lines; the median barrier stands between them."""
    paint_road(rgb, X, Y - road_y(X), Y_OUT_R + 0.2, Y_OUT_L - 0.2,
               solid=(Y_OUT_R + 0.45, Y_MED - 0.55, Y_MED + 0.55, Y_OUT_L - 1.5),
               dashed=(LANE_Y - 1.45, LANE_Y + 1.45, 5.2))


def make_world():
    rng = np.random.default_rng(SEED)
    w = World('open_highway', __doc__, extent=(X0, X1, Y0, Y1))
    w.set_terrain(Terrain(ground, X0, X1, Y0, Y1, 2.0, color='#8fae6e', paint=_paint))
    g = lambda x, y: float(ground(x, y))  # noqa: E731
    for x0, x1, kind in SECTIONS:
        if kind == 'rich':
            _rich(w, g, rng, x0, x1)
            continue
        _barrier(w, g, x0, x1, Y_MED, 0.8, 0.6, props.C_CONCRETE, 50.0)
        if kind in ('bridge', 'bridge_lamps'):
            for off in (Y_OUT_R, Y_OUT_L):
                _barrier(w, g, x0, x1, off, 0.9, 0.4, props.C_CONCRETE, 50.0)
            if kind == 'bridge_lamps':
                for x in np.arange(x0 + 25.0, x1, 50.0):
                    props.lamp_post(w, x, Y_MED, 0.8, math.pi / 2, height=9.0)
        elif kind == 'walls':
            for off in (Y_OUT_R - 1.0, Y_OUT_L + 1.0):
                _barrier(w, g, x0, x1, off, 4.5, 0.3, '#90a0a8', 50.0)
        elif kind == 'railposts':
            for off in (Y_OUT_R, Y_OUT_L):
                with w.semantic('fence'):
                    for x in np.arange(x0, x1, 2.0):
                        w.cylinder(x, off, -0.3, 0.05, 1.4, color='#a0a0a0')
                    for z in (0.5, 1.0):
                        w.box_between(x0, off, x1, off, z, 0.08, 0.1, '#a0a0a0')
    _start_area(w, g, np.random.default_rng(SEED + 1), TRAIN_X[0])
    traffic.add_pool(w, POOL, X0 + 20.0, Y0 + 15.0)
    return w


def moving_blocks(tname, t, x):
    """Trajectories of the traffic blocks of a sequence (tools/generate.py);
    t, x: the knots of the car's trajectory."""
    if tname not in TRAFFIC:
        return {}
    road = dict(road_y=road_y, ground=ground, lanes=LANES, x_range=(X0 + 5.0, X1 - 5.0))
    return traffic.tracks(POOL, TRAFFIC[tname], road, tname, t, x)


def _lane(xa, xb, speeds):
    """Lane following from xa to xb. speeds: [(x, v)], the speed limit from x on."""
    xs = sorted(set(np.arange(xa, xb + 1e-6, 50.0).tolist()) | {float(x) for x, _ in speeds} | {float(xb)})
    v_at = dict((float(x), v) for x, v in speeds)
    return [P(x, float(road_y(x)) + LANE_Y, v=v_at.get(x)) for x in xs]


def trajectories(world):
    a, b = TRAIN_X
    c, d = HELDOUT_X
    return {
        'cruise_25': dict(items=_lane(a, b, [(a, 25.0)]), platform='car_highway', seed=1,
                          description='Training part of the highway at 25 m/s'),
        'varied_18': dict(items=_lane(a, b, [(a, 18.0), (1000, 14.0), (1150, 20.0), (2000, 16.0),
                                             (2650, 22.0), (2750, 17.0), (3300, 20.0)]),
                          platform='car_highway', seed=2,
                          description='Training part at 14-22 m/s, with speed changes inside the invariant stretches'),
        'jam_14': dict(items=(_lane(a, 1000, [(a, 14.0)]) + [Stop(30.0)] + _lane(1000, 2700, [(1000, 14.0)])
                              + [Stop(20.0)] + _lane(2700, b, [(2700, 14.0)])),
                       platform='car_highway', seed=4,
                       description='Training part at 14 m/s with a 30 s and a 20 s stop, one inside each invariant stretch'),
        'empty_14': dict(items=(_lane(a, 500, [(a, 14.0)]) + [Stop(25.0)] + _lane(500, 1000, [(500, 14.0)])
                                + [Stop(25.0)] + _lane(1000, 1800, [(1000, 14.0)])),
                         platform='car_highway', seed=8,
                         description='First 1.8 km at 14 m/s with no other vehicle; a 25 s stop in an ordinary '
                                     'stretch and one inside the bridge'),
        'bridge_start_20': dict(items=_lane(1030, b, [(1030, 20.0)]), platform='car_highway', seed=5,
                                description='Starts at rest inside the bridge (150 m of it ahead), then 20 m/s'),
        'walls_start_14': dict(items=_lane(2700, b, [(2700, 14.0)]), platform='car_highway', seed=6,
                               description='Starts at rest between the noise walls (120 m of them ahead), then 14 m/s'),
        'heldout_21': dict(items=_lane(c, d, [(c, 21.0), (5300, 16.0), (5400, 23.0), (6350, 19.0)]),
                           platform='car_highway', seed=3,
                           description='Held-out part of the highway (other kinds of invariant stretch) at 16-23 m/s'),
        'heldout_start_17': dict(items=_lane(5300, d, [(5300, 17.0)]), platform='car_highway', seed=7,
                                 description='Held-out part: starts at rest on the bridge with lamps (150 m of it ahead), then 17 m/s'),
    }
