"""tunnel: a 735 m road tunnel (straight, a 300 m radius 45 deg bend,
straight), 10 m wide and 6.5 m high with a stepped arch, raised walkways,
lamps every 10 m, signs every 100 m, emergency niches every 150 m and jet
fan pairs every 200 m, with portals and a turnaround area outside. The
classic degenerate case: almost nothing constrains motion along the axis."""
import math

import numpy as np

from mvsim_lio import props
from mvsim_lio.traj import P
from mvsim_lio.world import World

SEED = 909
L1, R, ANG, L2 = 250.0, 300.0, math.radians(45.0), 250.0
X_START = -300.0
HW = 5.0  # half width (wall inner faces)
HWALL, HCEIL = 5.5, 6.5

SCREENSHOTS = [
    dict(point_to=(-20, 60, 0), distance=520, azimuth=-90, elevation=75, fov=55),
    dict(point_to=(-250, 0, 1.0), distance=30, azimuth=-150, elevation=35, fov=60, cut_z=5.4),
    dict(point_to=(-200, 0, 2.0), distance=10, azimuth=180, elevation=8, fov=80),
]


def centerline(s):
    """Point and heading of the tunnel axis at arc length s (0 = west portal)."""
    if s <= L1:
        return X_START + s, 0.0, 0.0
    if s <= L1 + R * ANG:
        a = (s - L1) / R
        return X_START + L1 + R * math.sin(a), R * (1 - math.cos(a)), a
    x1, y1 = X_START + L1 + R * math.sin(ANG), R * (1 - math.cos(ANG))
    d = s - L1 - R * ANG
    return x1 + d * math.cos(ANG), y1 + d * math.sin(ANG), ANG


LENGTH = L1 + R * ANG + L2


def at(s, lateral):
    x, y, th = centerline(s)
    return x - math.sin(th) * lateral, y + math.cos(th) * lateral, th


def make_world():
    rng = np.random.default_rng(SEED)
    xe, ye, the = centerline(LENGTH)
    w = World('tunnel', __doc__, extent=(-380, xe + 115, -60, ye + 115))
    w.hplane(-400, xe + 120, -80, ye + 120, 0.0, color='#505050')
    seg = 5.0
    niches = [150.0 * k for k in range(1, int(LENGTH / 150) + 1) if 150.0 * k < LENGTH - 10]
    for s in np.arange(0.0, LENGTH, seg):
        sm = s + seg / 2
        x, y, th = centerline(sm)
        for side in (-1, 1):
            in_niche = side < 0 and any(n <= sm <= n + 3.5 for n in niches)
            if not in_niche:
                wx, wy, _ = at(sm, side * (HW + 0.15))
                w.box(wx, wy, 0.0, seg + 0.1, 0.3, HWALL + 0.6, th, '#b8b4a8')
            # Walkway (raised curb) along each wall
            kx, ky, _ = at(sm, side * (HW - 0.5))
            w.box(kx, ky, 0.0, seg + 0.05, 1.0, 0.25, th, '#9a9890')
            # Haunch (stepped arch) and the ceiling
            hx, hy, _ = at(sm, side * (HW - 0.7))
            w.box(hx, hy, HWALL, seg + 0.1, 1.6, HCEIL - HWALL, th, '#b8b4a8', plot=False)
        cx, cy, _ = at(sm, 0.0)
        w.box(cx, cy, HCEIL, seg + 0.1, 2 * HW + 0.6, 0.4, th, '#a8a49c', plot=False)
    for n in niches:
        # Emergency niche: 3.5 m long, 1.5 m deep recess in the right wall
        bx, by, th = at(n + 1.75, -(HW + 1.5 + 0.15))
        w.box(bx, by, 0.0, 3.5, 0.3, HWALL, th, '#c8c4b8')
        for e in (n, n + 3.5):
            sx, sy, th2 = at(e, -(HW + 0.9))
            w.box(sx, sy, 0.0, 0.3, 1.5, HWALL, th2, '#c8c4b8')
        px, py, th = at(n + 1.75, -(HW + 1.2))
        w.box(px, py, 0.9, 0.5, 0.3, 0.6, th, '#e04020')  # emergency phone cabinet
    for s in np.arange(5.0, LENGTH, 10.0):
        for side in (-1, 1):
            lx, ly, th = at(s, side * (HW - 0.1))
            w.box(lx, ly, 4.8, 0.6, 0.2, 0.15, th, '#f0f0d0')
    for s in np.arange(50.0, LENGTH, 100.0):
        sx, sy, th = at(s, HW - 0.08)
        w.box(sx, sy, 3.2, 2.0, 0.1, 0.8, th, '#20a040')
    for s in np.arange(100.0, LENGTH - 50, 200.0):
        for lat in (-2.0, 2.0):
            fx, fy, th = at(s, lat)
            w.cylinder(fx, fy, 5.7, 0.6, 4.0, yaw=th, pitch=math.pi / 2, color='#707070')
    # Portals: facade walls around both openings
    for s_end, sign in ((0.0, -1), (LENGTH, 1)):
        x, y, th = centerline(s_end)
        for side in (-1, 1):
            fx, fy, _ = at(s_end + sign * 1.0, side * (HW + 15.3))
            w.box(fx, fy, 0.0, 2.0, 30.0, 14.0, th, '#8a8678')
        lx, ly, _ = at(s_end + sign * 1.0, 0.0)
        w.box(lx, ly, HCEIL + 0.4, 2.0, 2 * HW + 0.6, 14.0 - HCEIL - 0.4, th, '#8a8678')
        # Approach road guard rails, poles and trees
        for d in np.arange(4.0, 70.0, 4.0):
            for side in (-1, 1):
                gx, gy, _ = at(s_end + sign * d, side * 6.5)
                w.box(gx, gy, 0.5, 4.05, 0.1, 0.3, th, '#a0a0a0')
                if int(d) % 12 == 0:
                    w.cylinder(gx, gy, 0.0, 0.08, 0.8, color='#a0a0a0')
        for _ in range(12):
            d = rng.uniform(10, 70)
            tx, ty, _ = at(s_end + sign * d, rng.choice([-1, 1]) * rng.uniform(10, 25))
            props.tree(w, rng, tx, ty, 0.0)
    # Turnaround (roundabout) beyond the east portal
    rx, ry, _ = at(LENGTH + 85.0, 0.0)
    w.cylinder(rx, ry, 0.0, 7.0, 0.3, color='#7a9a60')
    props.tree(w, rng, rx, ry, 0.3, height=8.0, crown=3.0)
    return w


def trajectories(world):
    def lane(s0, s1, lateral, step=10.0):
        ss = np.arange(s0, s1, step if s1 > s0 else -step)
        return [at(float(s), lateral)[:2] for s in ss] + [at(s1, lateral)[:2]]

    rx, ry, the = at(LENGTH + 85.0, 0.0)
    out = lane(-50.0, LENGTH + 70.0, -2.0)
    # Counter-clockwise roundabout (right-hand traffic): enter on the right
    # lane, go around ~340 deg, leave on the opposite lane.
    loop = []
    d = math.asin(2.0 / 12.0)
    a_in = the + math.pi + d
    sweep = 2 * math.pi - 2 * d
    for k in range(13):
        a = a_in + sweep * k / 12
        loop.append((rx + 12.0 * math.cos(a), ry + 12.0 * math.sin(a)))
    back = lane(LENGTH + 70.0, -50.0, 2.0)
    car = [P(x, y, v=15.0) for x, y in out]
    car += [P(x, y, v=6.0) for x, y in loop]
    car += [P(back[0][0], back[0][1], v=15.0)] + [P(x, y) for x, y in back[1:]]
    ugv = [P(x, y, v=2.5) for x, y in lane(-30.0, LENGTH + 30.0, -2.8)]
    return {
        'car_through': dict(items=car, platform='car', seed=1,
                            description='Car, 15 m/s, through the tunnel, turnaround, and back'),
        'ugv_inspection': dict(items=ugv, platform='ugv', seed=2,
                               description='UGV, 2.5 m/s, once through the tunnel near the right wall'),
    }
