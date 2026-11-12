"""urban_street: a 3 x 3 grid of city blocks (~370 x 370 m) with 16 m wide
streets, raised sidewalks with curbs, buildings 8-45 m tall with facade
relief, a courtyard block, a park block, parked cars, street trees, lamp
posts, benches, bins, bus stops, sign posts and traffic lights.
Well-constrained, structured scene; the car route revisits streets (loops)."""
import math

import numpy as np

from mvsim_lio import props
from mvsim_lio.roads import lane_path
from mvsim_lio.traj import P
from mvsim_lio.world import World

SEED = 303
PITCH = 100.0  # block pitch [m]
HALF = 42.0  # half size of a block including its sidewalk [m]
SW = 4.0  # sidewalk width [m]
CURB = 0.15  # sidewalk height [m]
STREETS = (-150.0, -50.0, 50.0, 150.0)

SCREENSHOTS = [
    dict(point_to=(0, 0, 0), distance=330, azimuth=-60, elevation=40, fov=50),
    dict(point_to=(50, -10, 2), distance=45, azimuth=-150, elevation=18, fov=60),
]


def block_buildings(w, rng, cx, cy, kind):
    inner = HALF - SW  # 38 m
    m = 0.6  # facade relief margin
    if kind == 'quad':
        for sx in (-1, 1):
            for sy in (-1, 1):
                L = (2 * inner - 6) / 2 - 2 * m
                props.building(w, rng, cx + sx * (3 + L / 2 + m), cy + sy * (3 + L / 2 + m), L, L,
                               rng.uniform(8, 45), entrance_side=rng.integers(4))
    elif kind == 'court':
        H = rng.uniform(15, 25)
        L = 2 * inner - 2 * m
        d = 14.0
        props.building(w, rng, cx, cy - inner + m + d / 2, L, d, H, entrance_side=0)
        props.building(w, rng, cx, cy + inner - m - d / 2, L, d, H, entrance_side=2)
        props.building(w, rng, cx - inner + m + d / 2, cy, d, L - 2 * d - 2 * m, H)
        props.building(w, rng, cx + inner - m - d / 2, cy, d, L - 2 * d - 2 * m, H)
        for _ in range(8):
            props.tree(w, rng, cx + rng.uniform(-15, 15), cy + rng.uniform(-15, 15), 0.0)
    elif kind == 'slabs':
        L = 2 * inner - 2 * m
        props.building(w, rng, cx, cy - inner + m + 9, L, 18, rng.uniform(25, 45), entrance_side=0)
        props.building(w, rng, cx - inner + m + 12, cy + 15, 24, 40, rng.uniform(8, 15))
        for k in range(6):
            props.car(w, rng, cx + 5 + 3 * k, cy + 10, 0.0, math.pi / 2)
            props.car(w, rng, cx + 5 + 3 * k, cy + 24, 0.0, -math.pi / 2)
    elif kind == 'park':
        for _ in range(45):
            x, y = cx + rng.uniform(-34, 34), cy + rng.uniform(-34, 34)
            if abs(x - cx) < 3 or abs(y - cy) < 3:
                continue  # paths
            props.tree(w, rng, x, y, 0.0, leaves=35)
        for k in range(6):
            props.bench(w, cx + 5, cy - 30 + 12 * k, 0.0, math.pi / 2)
        with w.instance(), w.semantic('other-structure'):
            w.box(cx + 12, cy + 12, 0.0, 4.0, 4.0, 3.0, 0.0, '#b07050')  # kiosk
        with w.instance(), w.semantic('other-structure'):
            w.cylinder(cx, cy, 0.0, 3.0, 0.6, color='#909090')  # fountain basin
            w.cylinder(cx, cy, 0.0, 0.4, 2.5, color='#909090')


def make_world():
    rng = np.random.default_rng(SEED)
    E = 185.0
    w = World('urban_street', __doc__, extent=(-E, E, -E, E))
    with w.semantic('road'):
        w.hplane(-E, E, -E, E, 0.0, color='#505050')
    kinds = {(0, 0): 'quad', (1, 0): 'court', (2, 0): 'quad', (0, 1): 'slabs', (1, 1): 'quad',
             (2, 1): 'park', (0, 2): 'quad', (1, 2): 'slabs', (2, 2): 'court'}
    for i in range(3):
        for j in range(3):
            cx, cy = (i - 1) * PITCH, (j - 1) * PITCH
            with w.semantic('sidewalk'):
                w.box(cx, cy, 0.0, 2 * HALF, 2 * HALF, CURB, 0.0, '#a8a8a0')
            block_buildings(w, rng, cx, cy, kinds[(i, j)])
            # Street furniture along the four curbs:
            for side in range(4):
                ang = side * math.pi / 2
                ux, uy = math.cos(ang), math.sin(ang)  # outward normal of this curb
                tx, ty = -uy, ux
                for k, u in enumerate(np.arange(-35.0, 35.1, 12.5)):
                    bx = cx + ux * (HALF - 0.5) + tx * u
                    by = cy + uy * (HALF - 0.5) + ty * u
                    if k % 2 == 0:
                        props.lamp_post(w, bx, by, CURB, ang + math.pi, height=7.0, arm=1.2)
                        if k % 4 == 0:
                            props.bin_(w, bx - tx * 1.2, by - ty * 1.2, CURB)
                    elif kinds[(i, j)] != 'park':
                        tpx = cx + ux * (HALF - 1.0) + tx * u
                        tpy = cy + uy * (HALF - 1.0) + ty * u
                        props.tree(w, rng, tpx, tpy, CURB, height=rng.uniform(6, 8), crown=1.8,
                                   leaves=30)
                # Sign posts at the corners
                sx = cx + ux * (HALF - 0.4) + tx * (HALF - 0.4)
                sy = cy + uy * (HALF - 0.4) + ty * (HALF - 0.4)
                props.sign_post(w, sx, sy, CURB, ang)
    # Traffic lights at the inner intersections
    for x in STREETS[1:3]:
        for y in STREETS[1:3]:
            for dx, dy in ((1, 1), (-1, -1)):
                px, py = x + dx * (8 + 0.6), y + dy * (8 + 0.6)
                with w.instance(), w.semantic('pole'):
                    w.cylinder(px, py, CURB, 0.1, 4.5, color='#404040')
                    with w.semantic('traffic-sign'):
                        w.box(px, py, CURB + 3.2, 0.35, 0.35, 1.0, 0.0, '#202020')
    # Parked cars (parallel, in the curb lanes) on two east-west streets
    for y in (-50.0, 50.0):
        for side in (-1, 1):
            for x in np.arange(-140.0, 140.0, 6.5):
                if abs(x - 50) < 12 or abs(x + 50) < 12 or rng.uniform() > 0.7:
                    continue
                props.car(w, rng, x, y + side * 6.8, 0.0, 0.0 if side < 0 else math.pi)
    # Bus stops on the x = 50 street
    for y in (-100.0, 100.0):
        props.bus_stop(w, 50 + 8 + 1.6, y + 10, CURB, -math.pi / 2)
    # Boundary buildings around the outer ring road
    for k in np.arange(-150.0, 150.1, 37.5):
        for (x, y, lx, ly) in ((k, 172, 30, 14), (k, -172, 30, 14), (172, k, 14, 30),
                               (-172, k, 14, 30)):
            props.building(w, rng, x, y, lx, ly, rng.uniform(10, 30), pilaster=0)
    return w


def trajectories(world):
    S = STREETS
    nodes = [(S[0], S[0]), (S[3], S[0]), (S[3], S[3]), (S[0], S[3]), (S[0], S[1]), (S[2], S[1]),
             (S[2], S[3]), (S[1], S[3]), (S[1], S[0]), (S[3], S[0]), (S[3], S[2]), (S[0], S[2]),
             (S[0], S[0]), (S[1], S[0])]
    car = [P(x, y, v=10.0) for x, y in lane_path(nodes, 2.75)]
    # Pedestrian: around the central block on the sidewalk, across the
    # street to the east block, around it, and back.
    L = HALF - 2.2
    z = CURB
    walk = [P(-L, -L, z, v=1.3), P(L, -L, z), P(L, -8, z), P(50 + 8 - 0.0, -8, 0.0),
            P(100 - L, -8, z), P(100 - L, -L, z), P(100 + L, -L, z), P(100 + L, L, z),
            P(100 - L, L, z), P(100 - L, 8, z), P(50, 8, 0.0), P(L, 8, z), P(L, L, z),
            P(-L, L, z), P(-L, -L + 1, z)]
    return {
        'car_loops': dict(items=car, platform='car', seed=1,
                          description='Car, 10 m/s, outer ring plus two inner loops (revisits)'),
        'pedestrian': dict(items=walk, platform='handheld', explicit_z=True, seed=2,
                           look_around_deg=12.0,
                           description='Handheld rig walking the sidewalks of two blocks'),
    }
