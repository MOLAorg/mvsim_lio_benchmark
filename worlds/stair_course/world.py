"""stair_course: an urban courtyard (60 x 50 m, facades all around) with two
stair types for a legged robot: a straight, 2 m wide outdoor flight of 14
steps (17 cm risers, 30 cm treads) up to a 2.38 m terrace with parapets, and
a spiral stair (20 steps of 18 cm, one full turn around a central column,
floating treads and railing posts) from the terrace up to a 6 m platform on
columns. Stresses straight and helical stair climbing and descending, with
large, continuous yaw on the spiral."""
import math

import numpy as np

from mvsim_lio import props
from mvsim_lio.traj import P, Stop
from mvsim_lio.world import World

SEED = 1515
ZT = 14 * 0.17  # terrace top (2.38 m)
TX0, TX1, TY0, TY1 = -6.0, 8.0, 2.0, 12.0  # terrace footprint
SC = (4.0, 8.0)  # spiral center
N_SP, R_SP = 20, (6.0 - ZT) / 20  # spiral steps and riser
ZP = ZT + N_SP * R_SP  # upper platform (6.0 m)
A0 = math.radians(270.0)  # angle of the spiral start (south of the column)
RWALK = 1.0  # walking radius on the spiral

SCREENSHOTS = [
    dict(point_to=(2, 4, 2), distance=45, azimuth=-60, elevation=30, fov=50),
    dict(point_to=(4, 8, 3.5), distance=12, azimuth=-120, elevation=20, fov=70),
]


def make_world():
    rng = np.random.default_rng(SEED)
    w = World('stair_course', __doc__, extent=(-32, 32, -27, 27))
    w.hplane(-35, 35, -30, 30, 0.0, color='#9a948a')
    # Courtyard facades
    for (x, y, lx, ly, h) in ((0, 23, 60, 6, 15), (0, -23, 60, 6, 12), (-28, 0, 6, 40, 18),
                              (28, 0, 6, 40, 10)):
        props.building(w, rng, x, y, lx, ly, h, pilaster=4.0)
    for (x, y) in ((-15, -10), (15, -12), (-18, 10), (18, 12)):
        props.tree(w, rng, x, y, 0.0)
    for x in (-12.0, 12.0):
        props.lamp_post(w, x, -4.0, 0.0, 0.0, height=5.0, arm=0.5)
    for k in range(4):
        props.bench(w, -14.0 + 6.0 * k, -16.0, 0.0, 0.0)
    # Terrace (solid) with parapets, and the straight flight up to it
    w.box((TX0 + TX1) / 2, (TY0 + TY1) / 2, 0.0, TX1 - TX0, TY1 - TY0, ZT, 0.0, '#b8b0a4',
          drivable=True)
    props.stairs_flight(w, 0.0, TY0 - 14 * 0.30, 0.0, math.pi / 2, 14, 0.17, 0.30, 2.0)
    for side in (-1.0, 1.0):
        for k in range(15):
            yy = TY0 - 14 * 0.30 + 0.3 * k
            zs = min(14, k) * 0.17
            w.cylinder(side * 1.1, yy, zs, 0.03, 1.0, color=props.C_METAL)  # handrail posts
    par = '#a8a094'
    w.box_between(TX0, TY0, -1.2, TY0, ZT, 0.2, 1.0, par)
    w.box_between(1.2, TY0, TX1, TY0, ZT, 0.2, 1.0, par)
    w.box_between(TX0, TY1, TX1, TY1, ZT, 0.2, 1.0, par)
    w.box_between(TX0, TY0, TX0, TY1, ZT, 0.2, 1.0, par)
    w.box_between(TX1, TY0, TX1, TY1, ZT, 0.2, 1.0, par)
    for (x, y) in ((-5.0, 11.0), (-5.0, 3.0), (7.0, 11.0), (2.0, 3.0)):
        w.box(x, y, ZT, 1.0, 1.0, 0.8, 0.0, '#806850')  # planters
    # Spiral stair: column, floating treads, railing posts
    cx, cy = SC
    w.cylinder(cx, cy, ZT, 0.25, ZP + 1.2 - ZT, color='#909090')
    for i in range(N_SP):
        a = A0 + math.radians(18.0 * (i + 0.5))
        rc = 0.25 + 1.25 / 2
        top = ZT + (i + 1) * R_SP
        w.box(cx + rc * math.cos(a), cy + rc * math.sin(a), top - 0.08, 1.25, 0.45, 0.08, a,
              props.C_CONCRETE, drivable=True)
        w.cylinder(cx + 1.55 * math.cos(a), cy + 1.55 * math.sin(a), top, 0.025, 1.0,
                   color=props.C_METAL)
    # Upper platform on two columns, with parapets, entered from the spiral
    px0, px1, py0, py1 = cx - 0.5, TX1, cy - 2.7, cy - 0.6
    w.box((px0 + px1) / 2, (py0 + py1) / 2, ZP - 0.2, px1 - px0, py1 - py0, 0.2, 0.0,
          '#b0aca4', drivable=True, plot=False)
    for (x, y) in ((px1 - 0.3, py0 + 0.3), (px1 - 0.3, py1 - 0.3)):
        w.box(x, y, ZT, 0.3, 0.3, ZP - 0.2 - ZT, 0.0, '#b0aca4')
    w.box_between(px0, py0, px1, py0, ZP, 0.1, 1.0, par)
    w.box_between(px1, py0, px1, py1, ZP, 0.1, 1.0, par)
    w.box_between(cx + 1.6, py1, px1, py1, ZP, 0.1, 1.0, par)
    return w


def spiral(up=True, v=0.35):
    """Points along the walking helix (one per tread), from the terrace to the
    platform (up=True) or back down."""
    cx, cy = SC
    pts = []
    for k in range(N_SP + 1):
        a = A0 + math.radians(18.0 * k)
        pts.append((cx + RWALK * math.cos(a), cy + RWALK * math.sin(a), ZT + k * R_SP))
    if not up:
        pts = pts[::-1]
    out = [P(x, y, z) for x, y, z in pts]
    out[1].v = v  # callers drop the first point (it repeats their last one)
    return out


def trajectories(world):
    cx, cy = SC
    ys = TY0 - 14 * 0.30  # foot of the straight flight
    start = [P(-11.0, -6.5, 0.0, v=1.1), P(-6.0, -10.0, 0.0), P(6.0, -10.0, 0.0),
             P(6.0, -6.0, 0.0), P(0.0, -4.0, 0.0), P(0.0, ys - 0.5, 0.0),
             P(0.0, ys, 0.0, v=0.45), P(0.0, TY0, ZT), P(0.0, TY0 + 0.6, ZT, v=0.9),
             P(-3.0, 4.5, ZT), P(-1.0, cy - RWALK, ZT), P(cx - 0.6, cy - RWALK, ZT)]
    up = spiral(True)
    top = [P(cx + 0.6, cy - RWALK, ZP, v=0.8), P(6.5, cy - 1.6, ZP), P(7.3, cy - 1.6, ZP),
           Stop(1.0), P(6.5, cy - 1.6, ZP), P(cx + 0.6, cy - RWALK, ZP), P(cx, cy - RWALK, ZP)]
    down = spiral(False, 0.3)
    back = [P(cx - 0.6, cy - RWALK, ZT, v=0.9), P(-1.0, cy - RWALK, ZT), P(-3.0, 9.5, ZT),
            P(2.0, 10.5, ZT), P(1.5, 4.5, ZT), P(0.0, TY0 + 0.8, ZT), P(0.0, TY0, ZT, v=0.4),
            P(0.0, ys, 0.0), P(0.0, ys - 0.6, 0.0, v=1.1), P(0.0, -6.0, 0.0),
            P(-8.0, -6.0, 0.0), P(-11.0, -6.5, 0.0)]
    course = start + up[1:] + top + down[1:] + back
    return {
        'dog_course': dict(items=course, platform='dog', explicit_z=True, seed=1, knot_hz=50,
                           pitch_scale=0.85,
                           description='Robot dog: courtyard, straight flight up, spiral up '
                                       'to the platform, spiral and flight back down'),
    }
