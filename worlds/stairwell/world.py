"""stairwell: a three-storey building (30 x 16 m, 3.5 m floor to floor) with
a U-shaped stairwell (two flights of 10 steps, 17.5 cm risers, 28 cm treads,
32 deg, per storey, plus mid landings), a corridor and furnished rooms on
each floor, slabs with ceilings, and windows. Stresses stairs (fast z
changes with large pitch for a legged carrier), narrow spaces, ceilings and
multi-floor ambiguity (identical floor layouts)."""
import math

import numpy as np

from mvsim_lio import props
from mvsim_lio.traj import P, Stop
from mvsim_lio.world import World

SEED = 1111
X0, X1, Y0, Y1 = -15.0, 15.0, -8.0, 8.0
FLOORS = [0.0, 3.5, 7.0]
HROOF = 10.5
SLAB = 0.25
CW = 1.25  # corridor half width
SX0, SX1, SY = 7.5, 14.4, 2.2  # stairwell: x range, half width
FL0, FL1 = 9.5, 12.3  # flight x range
NSTEP, RISER, TREAD = 10, 0.175, 0.28
FW0, FW1 = 0.2, 2.1  # flight lateral range (|y|)

SCREENSHOTS = [
    dict(point_to=(0, 0, 4), distance=55, azimuth=-60, elevation=35, fov=50),
    dict(point_to=(4, 0, 0.5), distance=26, azimuth=-130, elevation=50, fov=60, cut_z=3.0),
]


def slab(w, z, hole):
    """Slab with top at z, minus an optional hole (x0, x1, y0, y1)."""
    if hole is None:
        w.box(0.0, 0.0, z - SLAB, X1 - X0, Y1 - Y0, SLAB, 0.0, '#b8b4ac', drivable=True,
                  plot=False)
        return
    hx0, hx1, hy0, hy1 = hole
    parts = [(X0, hx0, Y0, Y1), (hx1, X1, Y0, Y1), (hx0, hx1, Y0, hy0), (hx0, hx1, hy1, Y1)]
    for (a, b, c, d) in parts:
        if b - a > 1e-6 and d - c > 1e-6:
            w.box((a + b) / 2, (c + d) / 2, z - SLAB, b - a, d - c, SLAB, 0.0, '#b8b4ac',
                  drivable=True, plot=False)


def make_world():
    rng = np.random.default_rng(SEED)
    w = World('stairwell', __doc__, extent=(-25, 25, -16, 16))
    w.hplane(-30, 30, -20, 20, 0.0, color='#707068')
    hole = (FL0, SX1, -SY, SY)
    for k, z in enumerate(FLOORS):
        if k > 0:
            slab(w, z, hole)
        zc = (FLOORS[k + 1] if k + 1 < len(FLOORS) else HROOF) - SLAB  # ceiling
        H = zc - z
        win = [dict(kind='window', pos=p, width=1.6, z_min=0.9, z_max=2.4)
               for p in np.linspace(0.08, 0.92, 7)]
        w.wall(X0, Y0, X1, Y0, z, H, 0.25, win)
        w.wall(X0, Y1, X1, Y1, z, H, 0.25, win)
        w.wall(X0, Y0, X0, Y1, z, H, 0.25)
        w.wall(X1, Y0, X1, Y1, z, H, 0.25)
        # Corridor walls with three doors per side
        doors = [dict(kind='door', pos=(x - X0) / (SX0 - X0), width=1.1, z_min=0.0, z_max=2.1)
                 for x in (-12.0, -4.5, 3.0)]
        for side in (-1, 1):
            w.wall(X0, side * CW, SX0, side * CW, z, H, 0.12, doors)
            for x in (-7.5, 0.0):
                w.wall(x, side * CW, x, side * (Y1 - 0.15), z, H, 0.1)  # room partitions
            # Stairwell side walls and the rooms east of the corridor
            w.wall(SX0, side * SY, X1, side * SY, z, H, 0.15)
        w.wall(SX0, -Y1, SX0, -SY, z, H, 0.15)
        w.wall(SX0, SY, SX0, Y1, z, H, 0.15)
        w.wall(SX0, -SY, SX0, SY, z, H, 0.15,
               [dict(kind='door', pos=0.5, width=1.6, z_min=0.0, z_max=2.2)])
        # Furniture in the rooms
        for xr in (-11.25, -3.75, 3.75):
            for side in (-1, 1):
                yr = side * 4.8
                w.box(xr, yr, 0.72 + z, 1.8, 0.9, 0.04, 0.0, '#c8b898')
                for dx in (-0.8, 0.8):
                    for dy in (-0.38, 0.38):
                        w.box(xr + dx, yr + dy, z, 0.05, 0.05, 0.72, 0.0, '#606060')
                w.box(xr + 2.6, side * (Y1 - 0.5), z, 1.0, 0.4, 2.0, 0.0, '#e0e0d8')
                w.cylinder(xr - 1.5, yr - side * 1.2, z, 0.25, 0.48, color='#304060')
                if rng.uniform() < 0.5:
                    props.bush(w, rng, xr - 2.8, side * (Y1 - 0.8), z, size=0.5)
        if k + 1 < len(FLOORS):
            # Stairs to the next floor: flight up along +x (south half),
            # mid landing, flight up along -x (north half).
            zm = z + NSTEP * RISER
            yc1, yc2 = -(FW0 + FW1) / 2, (FW0 + FW1) / 2
            props.stairs_flight(w, FL0, yc1, z, 0.0, NSTEP, RISER, TREAD, FW1 - FW0,
                                thickness=0.3)
            w.box((FL1 + SX1) / 2, 0.0, zm - 0.2, SX1 - FL1, 2 * SY, 0.2, 0.0,
                  props.C_CONCRETE, drivable=True)  # mid landing
            props.stairs_flight(w, FL1, yc2, zm, math.pi, NSTEP, RISER, TREAD, FW1 - FW0,
                                thickness=0.3)
            w.box((FL0 + FL1) / 2, 0.0, z, FL1 - FL0, 2 * FW0, 2 * NSTEP * RISER - SLAB, 0.0,
                  '#c8c4bc')  # stair core wall between the flights
    slab(w, HROOF, None)
    return w


def climb(z):
    """Waypoints from the stairwell landing of the floor at z to the next one."""
    zm, zt = z + NSTEP * RISER, z + 2 * NSTEP * RISER
    yc1, yc2 = -(FW0 + FW1) / 2, (FW0 + FW1) / 2
    return [P(8.6, -0.3, z), P(FL0 - 0.3, yc1, z), P(FL0, yc1, z, v=0.5), P(FL1, yc1, zm),
            P(13.3, yc1, zm), P(13.3, yc2, zm), P(FL1, yc2, zm), P(FL0, yc2, zt),
            P(FL0 - 0.4, yc2, zt, v=0.9), P(8.6, 0.3, zt)]


def descend(z):
    """Reverse of climb(z - 3.5): from the landing at z to the floor below."""
    pts = climb(z - 2 * NSTEP * RISER)[::-1]
    out = []
    for i, p in enumerate(pts):
        v = 0.5 if i in (2,) else (0.9 if i == len(pts) - 3 else None)
        out.append(P(p.x, p.y, p.z, v=v))
    return out


def trajectories(world):
    z0, z1, z2 = FLOORS

    def to_stairs(z):
        return [P(6.0, 0.0, z), P(SX0, 0.0, z)]

    def from_stairs(z, x_end):
        return [P(SX0, 0.0, z), P(6.0, 0.0, z), P(x_end, 0.0, z)]

    def room(z, x_door, side, x_in):
        # Through the door at x_door into a room, and back to the corridor.
        return [P(x_door, 0.0, z), P(x_door, side * 1.8, z), P(x_in, side * 3.0, z),
                P(x_in, side * 5.6, z), Stop(1.0), P(x_in, side * 3.0, z),
                P(x_door, side * 1.8, z), P(x_door, 0.0, z)]

    legged = ([P(-13.0, 0.0, z0, v=0.9)] + to_stairs(z0) + climb(z0) + climb(z1) +
              from_stairs(z2, -13.0) + [Stop(1.0)] + room(z2, -12.0, -1, -13.6) +
              to_stairs(z2) + descend(z2) + from_stairs(z1, -13.0) + [Stop(1.0)] +
              room(z1, -4.5, 1, -6.0) + to_stairs(z1) + descend(z1) +
              from_stairs(z0, -12.0))
    hand = ([P(-13.0, 0.0, z0, v=1.1)] + to_stairs(z0) + climb(z0) + from_stairs(z1, -10.0) +
            [Stop(1.0)] + to_stairs(z1) + descend(z1) + from_stairs(z0, 3.0) +
            room(z0, 3.0, -1, 1.6) + [P(-12.0, 0.0, z0)])
    return {
        'legged_floors': dict(items=legged, platform='legged', explicit_z=True, seed=1,
                              description='Legged robot: up two storeys by the stairs, rooms, '
                                          'and back down, body pitched with the stairs'),
        'handheld_stairs': dict(items=hand, platform='handheld', explicit_z=True, seed=2,
                                pitch_scale=0.2, look_around_deg=10.0,
                                description='Handheld rig, upright: up one storey and back down'),
    }
