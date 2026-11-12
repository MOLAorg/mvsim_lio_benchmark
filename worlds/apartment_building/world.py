"""apartment_building: a newly built three-storey apartment building (20 x 14 m,
3.0 m floor to floor, 2.75 m ceilings) with a dog-leg stair (two flights of 9
steps, 16.7 cm risers, 28 cm treads, a half landing and a central wall) in a
central core, and two empty apartments per floor (living room, corridor, two
bedrooms, kitchen, bathroom): bare walls, floors, ceilings, door and window
openings, no furniture. Stresses stairs for a legged robot, identical floors
(vertical ambiguity), small bare rooms with short ranges and door transitions."""
import math

import numpy as np

from mvsim_lio import props
from mvsim_lio.traj import P, Stop
from mvsim_lio.world import World

SEED = 1414
X0, X1, Y0, Y1 = -10.0, 10.0, -7.0, 7.0
FLOORS = [0.0, 3.0, 6.0]
HROOF = 9.0
SLAB = 0.25
H = 3.0 - SLAB  # wall height per storey (floor to ceiling)
CORE = 2.0  # core half width (walls at x = +-2)
FLIGHT_Y0 = 1.5  # first step of every flight (y)
NSTEP, RISER, TREAD = 9, 1.5 / 9, 0.28
FLIGHT_Y1 = FLIGHT_Y0 + NSTEP * TREAD  # 4.02: half landing edge
CORE_N = 5.5  # north wall of the stair core (y)
FX = 1.0  # flight centerlines at x = +-FX

SCREENSHOTS = [
    dict(point_to=(0, 0, 4), distance=45, azimuth=-60, elevation=30, fov=50),
    dict(point_to=(0, 0, 0.5), distance=30, azimuth=-120, elevation=55, fov=55, cut_z=2.6),
    dict(point_to=(0, 2.5, 1.0), distance=13, azimuth=-110, elevation=62, fov=60, cut_z=4.4),
]


def door(pos, width=1.0):
    return dict(kind='door', pos=pos, width=width, z_min=0.0, z_max=2.1)


def window(pos, width=1.4):
    return dict(kind='window', pos=pos, width=width, z_min=0.9, z_max=2.2)


def wall(w, x0, y0, x1, y1, z, openings_at=(), thick=0.12, kinds=None):
    """Wall with openings given by their center coordinate along the wall."""
    L = math.hypot(x1 - x0, y1 - y0)
    ops = []
    for i, c in enumerate(openings_at):
        u = (math.hypot(c[0] - x0, c[1] - y0)) / L
        k = kinds[i] if kinds else 'door'
        ops.append(door(u) if k == 'door' else door(u, 1.2) if k == 'passage' else window(u))
    with w.semantic('wall'):
        w.wall(x0, y0, x1, y1, z, H, thick, ops, color='#e4e0d8')


def apartment(w, z, sx):
    """Interior walls of one apartment, mirrored with sx = +1 (east) or -1."""
    X = lambda x: sx * x  # noqa: E731
    # Living room / bedrooms boundary, with the passage to the corridor
    wall(w, X(2.0), -0.5, X(10.0), -0.5, z, [(X(5.95), -0.5)], kinds=['passage'])
    # Corridor walls with the bedroom, bathroom and kitchen doors
    wall(w, X(5.3), -0.5, X(5.3), 7.0, z, [(X(5.3), 1.0), (X(5.3), 4.5)])
    wall(w, X(6.6), -0.5, X(6.6), 7.0, z, [(X(6.6), 1.5), (X(6.6), 5.0)])
    wall(w, X(2.0), 2.5, X(5.3), 2.5, z)  # bathroom | bedroom 2
    wall(w, X(6.6), 3.5, X(10.0), 3.5, z)  # bedroom 1 | kitchen


def slab(w, z, label='floor'):
    """Slab with its top at z and a hole over the stair."""
    parts = [(X0, X1, Y0, FLIGHT_Y0), (X0, -CORE + 0.05, FLIGHT_Y0, CORE_N),
             (CORE - 0.05, X1, FLIGHT_Y0, CORE_N), (X0, X1, CORE_N, Y1)]
    with w.semantic(label):
        for a, b, c, d in parts:
            w.box((a + b) / 2, (c + d) / 2, z - SLAB, b - a, d - c, SLAB, 0.0, '#c8c4bc',
                  drivable=True, plot=False)


def make_world():
    rng = np.random.default_rng(SEED)
    w = World('apartment_building', __doc__, extent=(-22, 22, -18, 16))
    with w.semantic('floor'):
        w.hplane(-30, 30, -25, 25, 0.0, color='#7a7a72')
    for k, z in enumerate(FLOORS):
        if k > 0:
            slab(w, z)
        # Facades with windows (and the street door on the ground floor)
        south = [(-8.0, Y0), (-4.0, Y0), (0.0, Y0), (4.0, Y0), (8.0, Y0)]
        kinds = ['window', 'window', 'door' if k == 0 else 'window', 'window', 'window']
        wall(w, X0, Y0, X1, Y0, z, south, 0.3, kinds)
        north = [(-8.5, Y1), (-3.6, Y1), (3.6, Y1), (8.5, Y1)]
        wall(w, X0, Y1, X1, Y1, z, north, 0.3, ['window'] * 4)
        for xs in (X0, X1):
            wall(w, xs, Y0, xs, Y1, z, [(xs, -4.0), (xs, 1.0), (xs, 5.0)], 0.3, ['window'] * 3)
        # Stair core: side walls with one apartment door each, north wall
        for sx in (-1, 1):
            wall(w, sx * CORE, Y0, sx * CORE, Y1, z, [(sx * CORE, -2.0)], 0.2)
            apartment(w, z, sx)
        wall(w, -CORE, CORE_N, CORE, CORE_N, z, (), 0.2)
        if k + 1 < len(FLOORS):
            # Dog-leg stair: flight up along +y (east half), half landing,
            # flight up along -y (west half), central wall between flights.
            zm = z + NSTEP * RISER
            props.stairs_flight(w, FX, FLIGHT_Y0, z, math.pi / 2, NSTEP, RISER, TREAD,
                                2 * FX - 0.1, thickness=0.3)
            with w.semantic('stairs'):  # half landing
                w.box(0.0, (FLIGHT_Y1 + CORE_N) / 2, zm - 0.2, 2 * CORE - 0.2,
                      CORE_N - FLIGHT_Y1, 0.2, 0.0, props.C_CONCRETE, drivable=True)
            props.stairs_flight(w, -FX, FLIGHT_Y1, zm, -math.pi / 2, NSTEP, RISER, TREAD,
                                2 * FX - 0.1, thickness=0.3)
    with w.semantic('wall'):
        w.box(0.0, (FLIGHT_Y0 + FLIGHT_Y1) / 2, 0.0, 0.1, FLIGHT_Y1 - FLIGHT_Y0, HROOF - SLAB,
              0.0, '#d8d4cc')  # central wall between the flights, all storeys
        # Top floor: guard wall where no flight continues upwards
        w.box(FX, FLIGHT_Y0, FLOORS[-1], 2 * FX - 0.1, 0.08, 1.0, 0.0, '#d8d4cc')
    slab(w, HROOF, 'ceiling')
    # Street context: neighbor buildings, sidewalk trees, lamp posts, cars
    for (x, y, lx, ly, h) in ((-17, 0, 8, 16, 12), (17, 0, 8, 16, 9), (0, 13, 30, 6, 15)):
        props.building(w, rng, x, y, lx, ly, h)
    for x in np.arange(-18.0, 19.0, 6.0):
        if abs(x) > 2:
            props.tree(w, rng, x, -10.5, 0.0, height=6.0, crown=1.6, leaves=30)
        props.lamp_post(w, x + 3.0, -10.2, 0.0, -math.pi / 2, height=6.0, arm=1.0)
    for x in (-12.0, 8.0, 14.0):
        props.car(w, rng, x, -13.0, 0.0, 0.0)
    return w


def climb(z, v_stairs):
    """From the core hall of the floor at z to the floor above (arrives at
    (-FX, 0) heading south)."""
    zm, zt = z + NSTEP * RISER, z + 2 * NSTEP * RISER
    return [P(FX, 0.0, z), P(FX, FLIGHT_Y0 - 0.3, z), P(FX, FLIGHT_Y0, z, v=v_stairs),
            P(FX, FLIGHT_Y1, zm), P(FX, 4.75, zm), P(-FX, 4.75, zm), P(-FX, FLIGHT_Y1, zm),
            P(-FX, FLIGHT_Y0, zt), P(-FX, FLIGHT_Y0 - 0.4, zt, v=0.9), P(-FX, 0.0, zt)]


def descend(z, v_stairs):
    """From the core hall of the floor at z down to the floor below."""
    pts = climb(z - 2 * NSTEP * RISER, 0.0)[::-1]
    out = [P(p.x, p.y, p.z) for p in pts]
    out[2].v = v_stairs
    out[-3].v = 0.9
    return out


def visit(z, sx, full=True):
    """From the core hall into the apartment on side sx, through its rooms,
    and back to the hall. Stops (in-place turns) at the end of each room."""
    X = lambda x: sx * x  # noqa: E731
    pts = [P(X(1.2), -2.0, z), P(X(2.0), -2.0, z), P(X(3.0), -2.0, z), P(X(5.0), -4.5, z),
           P(X(8.5), -5.6, z), Stop(1.0), P(X(8.5), -2.0, z), P(X(5.95), -1.3, z)]
    if full:
        corr = X(5.95)
        rooms = [((X(5.3), 1.0), (X(3.3), 1.3)), ((X(6.6), 1.5), (X(8.8), 0.6)),
                 ((X(5.3), 4.5), (X(3.3), 5.6)), ((X(6.6), 5.0), (X(8.8), 6.0))]
        for (dx, dy), (ex, ey) in rooms:
            ix = dx + 0.7 * math.copysign(1.0, ex - dx)  # straight through the door first
            pts += [P(corr, dy, z), P(dx, dy, z), P(ix, dy, z), P(ex, ey, z), Stop(1.0),
                    P(ix, dy, z), P(dx, dy, z), P(corr, dy, z)]
        pts += [P(corr, -1.2, z), P(X(4.5), -1.8, z)]
    pts += [P(X(3.0), -2.0, z), P(X(2.0), -2.0, z), P(X(1.2), -2.0, z), P(X(FX), -0.8, z)]
    return pts


def trajectories(world):
    z0, z1, z2 = FLOORS
    tour = ([P(0.0, -12.0, z0, v=1.0), P(0.0, -7.6, z0), P(0.0, -6.0, z0), P(0.0, -3.0, z0)] +
            visit(z0, 1) + climb(z0, 0.4)[1:] + visit(z1, -1) + [P(FX, -0.6, z1)] +
            climb(z1, 0.4)[1:] + visit(z2, 1) + visit(z2, -1, full=False) +
            [P(-FX, -0.3, z2)] + descend(z2, 0.35)[1:] + [P(FX, -0.6, z1), P(-FX, -0.6, z1)] +
            descend(z1, 0.35)[1:] + [P(0.0, -3.0, z0), P(0.0, -7.6, z0), P(0.0, -12.0, z0)])
    cycle = ([P(FX, -0.6, z0)] + climb(z0, 0.6)[1:] + [P(FX, -0.6, z1)] +
             climb(z1, 0.6)[1:] + [P(-FX, -0.6, z2), Stop(1.0)] + descend(z2, 0.5) +
             [P(FX, -0.6, z1), P(-FX, -0.6, z1)] + descend(z1, 0.5)[1:] +
             [P(0.0, -3.0, z0), Stop(1.0)])
    fast = [P(0.0, -3.0, z0, v=1.2)] + cycle + cycle
    return {
        'dog_floors': dict(items=tour, platform='dog', explicit_z=True, seed=1, knot_hz=50,
                           pitch_scale=0.85,
                           description='Robot dog: from the street up both storeys, through '
                                       'the rooms of one apartment per floor, and back down'),
        'dog_stairs': dict(items=fast, platform='dog', explicit_z=True, seed=2, knot_hz=50,
                           pitch_scale=0.85,
                           description='Robot dog: fast up and down the dog-leg stair, '
                                       'ground floor to top floor and back, twice'),
    }
