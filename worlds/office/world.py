"""office: one office floor, 60 x 30 m with a 2.8 m ceiling: a 50 m long,
2.2 m wide corridor, 16 rooms behind 1.2 m doors (desks with legs, chairs,
shelves, a meeting room), an open lobby with columns, and windows to an
outside with a few trees and cars. Stresses the long-corridor degeneracy,
very short ranges in rooms, door transitions and fast in-place rotations."""
import math

import numpy as np

from mvsim_lio import props
from mvsim_lio.traj import P, Spin, Stop
from mvsim_lio.world import World

SEED = 505
X0, X1, Y0, Y1 = -30.0, 30.0, -15.0, 15.0
HC = 2.8  # ceiling height
CW = 1.1  # corridor half width
XL = 20.0  # corridor east end (lobby beyond)
ROOM_W = 6.25

SCREENSHOTS = [
    dict(point_to=(-2, 0, 0), distance=55, azimuth=-60, elevation=60, fov=55, cut_z=2.6),
    dict(point_to=(-10, 0, 0.5), distance=16, azimuth=-130, elevation=40, fov=60, cut_z=2.6),
]


def room_x(k):
    return X0 + k * ROOM_W


def desk(w, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    w.box(x, y, 0.72, 1.6, 0.8, 0.04, yaw, '#c8b898')
    for dx in (-0.75, 0.75):
        for dy in (-0.35, 0.35):
            w.box(x + c * dx - s * dy, y + s * dx + c * dy, 0.0, 0.05, 0.05, 0.72, yaw, '#606060')
    # Chair in front of it
    cx, cy = x - s * 0.75, y + c * 0.75
    w.cylinder(cx, cy, 0.0, 0.04, 0.45, color='#303030')
    w.cylinder(cx, cy, 0.45, 0.25, 0.06, color='#303030')
    w.box(cx - s * 0.22, cy + c * 0.22, 0.5, 0.45, 0.05, 0.5, yaw, '#303030')
    # Monitor
    w.box(x + s * 0.2, y - c * 0.2, 0.76, 0.55, 0.05, 0.35, yaw, '#101010')


def make_world():
    rng = np.random.default_rng(SEED)
    w = World('office', __doc__, extent=(-40, 40, -25, 25))
    w.hplane(-45, 45, -30, 30, 0.0, color='#8c8c84')
    # Outer walls with windows every 3 m
    def windows(L):
        n = int(L / 3.0)
        return [dict(kind='window', pos=(i + 0.5) / n, width=1.8, z_min=0.9, z_max=2.3)
                for i in range(n)]
    w.wall(X0, Y0, X1, Y0, 0.0, HC, 0.3, windows(X1 - X0))
    w.wall(X0, Y1, X1, Y1, 0.0, HC, 0.3, windows(X1 - X0))
    w.wall(X0, Y0, X0, Y1, 0.0, HC, 0.3)
    w.wall(X1, Y0, X1, Y1, 0.0, HC, 0.3,
           [dict(kind='door', pos=0.5, width=2.4, z_min=0.0, z_max=2.4)])
    # Corridor walls with one door per room
    n_rooms = int((XL - X0) / ROOM_W)
    for side in (1, -1):
        doors = []
        for k in range(n_rooms):
            xc = room_x(k) + 1.4 if (k % 2 == 0) else room_x(k) + ROOM_W - 1.4
            doors.append(dict(kind='door', pos=(xc - X0) / (XL - X0), width=1.2, z_min=0.0,
                              z_max=2.1))
        w.wall(X0, side * CW, XL, side * CW, 0.0, HC, 0.12, doors)
        # Partitions between rooms (some removed: open-plan rooms)
        for k in range(1, n_rooms + 1):
            if side == 1 and k in (3, 6):
                continue
            w.wall(room_x(k), side * CW, room_x(k), side * (Y1 - 0.15), 0.0, HC, 0.1)
    # Furniture
    for side in (1, -1):
        for k in range(n_rooms):
            x0 = room_x(k)
            if side == -1 and k == 4:
                # Meeting room: one big table and chairs around it
                w.box(x0 + ROOM_W / 2, -8.0, 0.72, 3.6, 1.4, 0.05, 0.0, '#a08060')
                for dx in (-1.6, 1.6):
                    for dy in (-0.5, 0.5):
                        w.box(x0 + ROOM_W / 2 + dx, -8.0 + dy, 0.0, 0.06, 0.06, 0.72, 0.0, '#505050')
                for j in range(4):
                    for dy in (-1.2, 1.2):
                        w.cylinder(x0 + 1.7 + j * 0.95, -8.0 + dy, 0.0, 0.25, 0.48, color='#304060')
                continue
            for j in range(2):
                for i in range(2):
                    yy = side * (5.0 + 5.0 * j)
                    desk(w, x0 + 1.6 + 3.0 * i, yy, 0.0 if side > 0 else math.pi)
            # Shelves along the outer wall
            for i in range(3):
                w.box(x0 + 1.2 + 1.9 * i, side * (Y1 - 0.5), 0.0, 1.0, 0.4, 2.0, 0.0, '#e0e0d8')
            if rng.uniform() < 0.6:
                px = x0 + ROOM_W - 0.7 if k % 2 == 0 else x0 + 0.7  # away from the door
                props.bush(w, rng, px, side * (CW + 0.8), 0.0, size=0.5)
    # Ceiling lamps along the corridor (periodic)
    for x in np.arange(X0 + 1.5, XL, 3.0):
        w.box(x, 0.0, HC - 0.1, 1.2, 0.3, 0.08, 0.0, '#f0f0e8')
    # Lobby
    for x in (24.0, 28.0):
        for y in (-7.5, 7.5):
            w.box(x, y, 0.0, 0.5, 0.5, HC, 0.0, '#c8c8c0')
    w.box(26.0, -11.0, 0.0, 3.5, 0.8, 1.1, 0.0, '#906040')  # reception desk
    for y in (9.0, 11.5):
        w.box(27.0, y, 0.0, 2.0, 0.9, 0.45, 0.0, '#405080')  # sofas
    for (x, y) in ((22.0, 13.5), (29.0, -13.5), (22.0, -13.5)):
        props.bush(w, rng, x, y, 0.0, size=0.7)
    # Outside, seen through the windows
    for x in np.arange(-25.0, 26.0, 10.0):
        props.tree(w, rng, x, 20.0, 0.0)
        props.car(w, rng, x + 3.0, -19.5, 0.0, 0.0)
    w.hplane(X0, X1, Y0, Y1, HC, color='#e8e8e0')
    return w


def door_x(k):
    return room_x(k) + 1.4 if (k % 2 == 0) else room_x(k) + ROOM_W - 1.4


def trajectories(world):
    # Handheld: from the lobby along the corridor, into a north room through
    # its door, around its desks, out, then into a south room, to the west
    # end and back to the lobby.
    def visit(k, side):
        # Into room k through its door, down the gap between its desks, back.
        g = room_x(k) + 3.1
        return [P(door_x(k), 0.0), P(door_x(k), side * 2.4), P(g, side * 2.9),
                P(g, side * 12.3), Stop(1.0), P(g, side * 2.9), P(door_x(k), side * 2.4),
                P(door_x(k), 0.0)]

    walk = ([P(26.0, 0.0, v=1.2), P(XL - 1.0, 0.0)] + visit(7, -1) + visit(5, 1) + visit(2, -1) +
            visit(0, 1) +
            [P(X0 + 1.0, 0.0), Stop(1.0), P(XL + 1.5, 0.3), P(25.0, 4.0), Stop(1.0), P(25.0, 0.0)])
    spins = [P(26.0, -3.0, v=1.0), P(26.0, 0.0), P(10.0, 0.0), Spin(360, 4.0), P(-5.0, 0.0),
             Spin(-360, 4.0), P(-20.0, 0.0), Spin(720, 8.0), P(-27.0, 0.0),
             Spin(180, 3.0), P(XL + 1.5, 0.0), P(26.0, 4.0), P(26.0, -4.0),
             Spin(360, 4.0), P(23.0, -3.0)]
    return {
        'walk': dict(items=walk, platform='handheld', explicit_z=True, seed=1,
                     look_around_deg=20.0,
                     description='Handheld rig: corridor, two rooms through their doors, lobby'),
        'spins': dict(items=spins, platform='ugv', seed=2,
                      description='UGV along the corridor with fast in-place spins (90 deg/s)'),
    }
