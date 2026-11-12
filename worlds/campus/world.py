"""campus: a 500 x 400 m university campus on gently rolling ground (+-3 m):
nine buildings (one with a courtyard, a 25 m tower), a central plaza with a
fountain and benches, a tree-lined ring road with lamp posts, about 300
trees (most in a park), a car park, and a fenced sports field with goals and bleachers.
Mixed structure and vegetation at large scale; long sequences with loops."""
import math

import numpy as np

from mvsim_lio import props
from mvsim_lio.geom import dist_to_polyline
from mvsim_lio.roads import lane_path
from mvsim_lio.traj import P
from mvsim_lio.world import Terrain, World

SEED = 1313
RING = [(-200, -150), (200, -150), (200, 150), (-200, 150)]
BUILDINGS = [  # cx, cy, lx, ly, height, entrance side
    (-120, 80, 70, 25, 18, 0), (-20, 110, 60, 20, 22, 0), (90, 95, 50, 30, 15, 0),
    (160, 20, 30, 70, 12, 3), (-150, -30, 34, 34, 25, 1), (-52, 62, 28, 28, 20, 0),
    (140, -100, 35, 25, 10, 2), (-150, 20, 20, 30, 9, 1)]
TOUR = [(0, -30), (25, -10), (25, 30), (0, 50), (-25, 30), (-25, -10), (0, -30), (40, -40),
        (80, -45), (125, -50), (128, 0), (125, 45), (60, 60), (45, 15), (60, -15), (100, -20),
        (110, 20), (130, 60), (124, 100), (118, 120), (80, 126), (40, 135), (-60, 135), (-75, 100), (-75, 60),
        (-80, 0), (-110, -25), (-129, -42), (-130, -80), (-112, -80), (-50, -72), (-50, -90),
        (-112, -90), (-130, -92), (-130, -112), (-80, -114), (-20, -110), (20, -75),
        (0, -30), (25, -10),
        (25, 30), (0, 50), (-25, 30), (-25, -10), (0, -30)]


def ground(x, y):
    return 2.0 * np.sin(x / 90.0) * np.cos(y / 70.0) + 1.0 * np.sin((x + y) / 60.0)


def make_world():
    rng = np.random.default_rng(SEED)
    w = World('campus', __doc__, extent=(-250, 250, -200, 200))
    w.set_terrain(Terrain(ground, -250, 250, -200, 200, 1.0, color='#86a866'))
    g = lambda x, y: float(ground(x, y))  # noqa: E731

    def zmin(cx, cy, lx, ly):
        xs = np.linspace(cx - lx / 2, cx + lx / 2, 9)
        ys = np.linspace(cy - ly / 2, cy + ly / 2, 9)
        X, Y = np.meshgrid(xs, ys)
        return float(ground(X, Y).min())

    for (cx, cy, lx, ly, h, e) in BUILDINGS:
        props.building(w, rng, cx, cy, lx, ly, h, z=zmin(cx, cy, lx, ly), entrance_side=e)
    # Building with a courtyard (four wings)
    cx, cy, L, d, h = 60.0, -90.0, 46.0, 12.0, 14.0
    z0 = zmin(cx, cy, L, L)
    props.building(w, rng, cx, cy - L / 2 + d / 2, L, d, h, z=z0, entrance_side=0)
    props.building(w, rng, cx, cy + L / 2 - d / 2, L, d, h, z=z0)
    props.building(w, rng, cx - L / 2 + d / 2, cy, d, L - 2 * d - 1, h, z=z0)
    props.building(w, rng, cx + L / 2 - d / 2, cy, d, L - 2 * d - 1, h, z=z0)
    # Central plaza: paving, fountain, benches, lamps, a sculpture
    zp = zmin(0, 10, 60, 60)
    with w.semantic('other-ground'):
        w.box(0.0, 10.0, zp - 0.3, 56.0, 56.0, 0.35, 0.0, '#c8c0b0')
    zp += 0.05
    with w.instance(), w.semantic('other-structure'):
        w.cylinder(0.0, 10.0, zp, 5.0, 0.5, color='#a0a0a0')
        w.cylinder(0.0, 10.0, zp, 0.5, 3.0, color='#a0a0a0')
    for a in np.arange(0, 2 * math.pi, math.pi / 4):
        props.bench(w, 9.0 * math.cos(a), 10.0 + 9.0 * math.sin(a), zp, a + math.pi / 2)
    for x, y in ((-20, -10), (20, -10), (20, 30), (-20, 30), (-20, 10), (20, 10)):
        props.lamp_post(w, x, y, zp, 0.0, height=5.0, arm=0.4)
    with w.instance(), w.semantic('other-structure'):
        w.box(-12.0, 24.0, zp, 2.0, 2.0, 1.2, 0.3, '#707070')
        w.sphere(-12.0, 24.0, zp + 2.2, 1.0, color='#b08040')
    # Ring road: lamps and an avenue of trees
    ring = RING + RING[:1]
    for (ax, ay), (bx, by) in zip(ring[:-1], ring[1:]):
        L = math.hypot(bx - ax, by - ay)
        ux, uy = (bx - ax) / L, (by - ay) / L
        for s in np.arange(15.0, L - 10, 30.0):
            for side in (-1, 1):
                px, py = ax + ux * s - uy * side * 7.0, ay + uy * s + ux * side * 7.0
                props.lamp_post(w, px, py, g(px, py), math.atan2(uy * side, -ux * side),
                                height=7.0, arm=1.0)
                tx, ty = ax + ux * (s + 15) - uy * side * 10.0, ay + uy * (s + 15) + ux * side * 10.0
                if s + 15 < L - 10:
                    props.tree(w, rng, tx, ty, g(tx, ty), leaves=30)
    # Park (east): trees away from the tour path
    tour = TOUR
    for _ in range(260):
        x, y = rng.uniform(35, 135), rng.uniform(-60, 75)
        if dist_to_polyline(x, y, tour) < 4.0 or (130 < x and -20 < y < 60):
            continue
        props.tree(w, rng, x, y, g(x, y), leaves=30)
    for _ in range(60):
        x, y = rng.uniform(-240, 240), rng.uniform(-190, 190)
        if dist_to_polyline(x, y, tour) < 5.0 or dist_to_polyline(x, y, ring) < 12.0:
            continue
        if any(abs(x - b[0]) < b[2] / 2 + 6 and abs(y - b[1]) < b[3] / 2 + 6 for b in BUILDINGS):
            continue
        props.tree(w, rng, x, y, g(x, y), leaves=30)
    # Car park (north-west)
    for x in np.arange(-195.0, -130.0, 2.6):
        for y, yaw in ((110.0, math.pi / 2), (125.0, -math.pi / 2)):
            if rng.uniform() < 0.75:
                props.car(w, rng, x, y, g(x, y), yaw)
    # Sports field (south-west) with a fence, goals and bleachers
    fx0, fx1, fy0, fy1 = -125.0, -35.0, -100.0, -45.0
    for (ax, ay, bx, by) in ((fx0, fy0, fx1, fy0), (fx1, fy0, fx1, fy1), (fx1, fy1, fx0, fy1)):
        props.fence(w, ax, ay, bx, by, g, spacing=3.0, height=2.0, rails=3)
    props.fence(w, fx0, fy1, fx0, fy0 + 35.0, g, spacing=3.0, height=2.0, rails=3)  # gate gap
    for x in (fx0 + 4.0, fx1 - 4.0):
        with w.instance(), w.semantic('other-structure'):  # goal
            for dy in (-3.66, 3.66):
                w.cylinder(x, (fy0 + fy1) / 2 + dy, g(x, (fy0 + fy1) / 2) - 0.2, 0.06, 2.64,
                           color='#f0f0f0')
            w.cylinder(x, (fy0 + fy1) / 2 - 3.66, g(x, (fy0 + fy1) / 2) + 2.4, 0.06, 7.32,
                       yaw=math.pi / 2, pitch=math.pi / 2, color='#f0f0f0')
    zb = g(-80.0, fy1 + 6.0)
    with w.instance(), w.semantic('other-structure'):  # bleachers
        for k in range(4):
            w.box(-80.0, fy1 + 5.0 + 0.8 * k, zb - 0.2, 40.0, 0.8, 0.6 + 0.45 * k, 0.0,
                  '#9090a0')
    return w


def trajectories(world):
    tour = [P(*TOUR[0], v=2.0)] + [P(*p) for p in TOUR[1:]]
    nodes = [(-200, -120), (-200, -150), (200, -150), (200, 150), (-200, 150), (-200, -150),
             (200, -150), (200, 150), (-200, 150), (-200, -100)]
    ring = [P(x, y, v=8.0) for x, y in lane_path(nodes, 2.0)]
    return {
        'long_tour': dict(items=tour, platform='ugv', seed=1,
                          description='UGV, 2 m/s, plaza, park, buildings, sports field and '
                                      'the plaza again (loop closures)'),
        'ring_drive': dict(items=ring, platform='car', seed=2,
                           description='Car, 8 m/s, two laps of the ring road'),
    }
