"""Reusable props made of ray-traceable primitives (see world.py).

Foliage is modeled as clusters of small spheres rather than one solid ball,
so that the LiDAR sees a porous, irregular canopy as it does with real trees.
All randomness comes from the `rng` argument (numpy Generator).
Each call of a prop is one object (instance) of a semantic class (labels.py).
"""
import functools
import math

C_TRUNK = '#6b4f32'
C_LEAF = '#3f8f3a'
C_LEAF2 = '#2f7030'
C_CAR = ('#b03030', '#3050b0', '#d0d0d0', '#303030', '#e0c040', '#40a0a0')
C_GLASS = '#202830'
C_TIRE = '#151515'
C_METAL = '#8890a0'
C_CONCRETE = '#b8b4ac'
C_FACADE = ('#d8cfc0', '#c0b8b0', '#a8b0b8', '#d0c0a0', '#b0a090')
C_RACK = '#2050c0'
C_BEAM = '#e07020'
C_PALLET = '#c8a070'


def _object(cls):
    """Decorator: the primitives of each call are one instance of class `cls`."""
    def deco(fn):
        @functools.wraps(fn)
        def wrapper(w, *args, **kwargs):
            with w.instance(), w.semantic(cls):
                return fn(w, *args, **kwargs)
        return wrapper
    return deco


@_object('vegetation')
def tree(w, rng, x, y, z, height=None, crown=None, trunk_r=None, leaves=40, branches=4,
         color=C_LEAF):
    """Deciduous tree: trunk, a few tilted branches and a porous crown of
    `leaves` small spheres inside an ellipsoid."""
    H = height if height is not None else rng.uniform(5.0, 9.0)
    R = crown if crown is not None else rng.uniform(1.5, 3.0)
    tr = trunk_r if trunk_r is not None else rng.uniform(0.12, 0.30)
    h_trunk = H - 1.6 * R
    with w.semantic('trunk'):
        w.cylinder(x, y, z - 0.2, tr, h_trunk + 0.2 + 0.5 * R, color=C_TRUNK)
        for _ in range(branches):
            yaw = rng.uniform(-math.pi, math.pi)
            w.cylinder(x, y, z + h_trunk * rng.uniform(0.7, 1.0), tr * 0.35,
                       R * rng.uniform(0.7, 1.1), yaw=yaw,
                       pitch=math.radians(rng.uniform(35, 60)), color=C_TRUNK)
    cz = z + h_trunk + R * 0.8
    for _ in range(leaves):
        # Uniform in an ellipsoid shell-ish volume (denser outside, as foliage).
        u = rng.normal(size=3)
        u /= math.sqrt((u ** 2).sum())
        r = R * rng.uniform(0.45, 1.0) ** 0.5
        w.sphere(x + u[0] * r, y + u[1] * r, cz + u[2] * r * 0.8, rng.uniform(0.25, 0.55),
                 color=color)


@_object('vegetation')
def conifer(w, rng, x, y, z, height=None, color=C_LEAF2):
    """Conifer: trunk plus stacked, shrinking rings of spheres."""
    H = height if height is not None else rng.uniform(8.0, 16.0)
    tr = rng.uniform(0.15, 0.35)
    with w.semantic('trunk'):
        w.cylinder(x, y, z - 0.2, tr, H + 0.2, color=C_TRUNK)
    z_start = z + rng.uniform(1.5, 3.0)
    levels = int((H - (z_start - z)) / 0.9)
    for k in range(levels):
        zk = z_start + k * 0.9
        rk = max(0.3, 2.5 * (1.0 - (zk - z) / H))
        n = max(3, int(2 * math.pi * rk / 0.7))
        a0 = rng.uniform(0, 2 * math.pi)
        for j in range(n):
            a = a0 + 2 * math.pi * j / n
            w.sphere(x + rk * math.cos(a), y + rk * math.sin(a), zk, 0.35, color=color)


@_object('vegetation')
def bush(w, rng, x, y, z, size=None, color=C_LEAF):
    S = size if size is not None else rng.uniform(0.6, 1.4)
    for _ in range(int(6 + 8 * S)):
        w.sphere(x + rng.uniform(-S, S) * 0.7, y + rng.uniform(-S, S) * 0.7,
                 z + rng.uniform(0.1, S * 0.8), rng.uniform(0.2, 0.4) * S, color=color)


@_object('trunk')
def log(w, rng, x, y, z, length=None, radius=None):
    L = length if length is not None else rng.uniform(3, 7)
    r = radius if radius is not None else rng.uniform(0.15, 0.3)
    w.cylinder(x, y, z + r * 0.8, r, L, yaw=rng.uniform(-math.pi, math.pi), pitch=math.pi / 2,
               color=C_TRUNK)


@_object('car')
def car(w, rng, x, y, z, yaw, kind=None):
    """Parked car centered at (x, y) on ground height z, heading `yaw`."""
    kinds = {'sedan': (4.5, 1.8, 1.45), 'hatch': (3.9, 1.75, 1.5), 'suv': (4.7, 1.9, 1.75),
             'van': (5.2, 2.0, 2.1)}
    k = kind or rng.choice(list(kinds))
    L, W, H = kinds[k]
    col = C_CAR[int(rng.integers(len(C_CAR)))]
    c, s = math.cos(yaw), math.sin(yaw)
    body_z0, body_h = 0.30, (H - 0.30) * (0.55 if k != 'van' else 0.9)
    w.box(x, y, z + body_z0, L, W, body_h, yaw, col)
    if k != 'van':
        cab_l = L * (0.5 if k == 'sedan' else 0.6)
        off = -0.1 * L
        w.box(x + c * off, y + s * off, z + body_z0 + body_h, cab_l, W - 0.12,
              H - body_z0 - body_h, yaw, C_GLASS)
    for fx in (0.36 * L, -0.33 * L):
        for fy in (0.5 * W - 0.11, -(0.5 * W - 0.11)):
            wx = x + c * fx - s * (fy + 0.11 * math.copysign(1, fy))
            wy = y + s * fx + c * (fy + 0.11 * math.copysign(1, fy))
            # Wheel: axis lateral, starting at the outer face, pointing inwards.
            w.cylinder(wx, wy, z + 0.33, 0.33, 0.22, yaw=yaw - math.copysign(math.pi / 2, fy),
                       pitch=math.pi / 2, color=C_TIRE)


@_object('pole')
def lamp_post(w, x, y, z, yaw, height=6.0, arm=1.5):
    w.cylinder(x, y, z, 0.09, height, color=C_METAL)
    w.cylinder(x, y, z + height - 0.1, 0.05, arm, yaw=yaw, pitch=math.pi / 2, color=C_METAL)
    hx, hy = x + arm * math.cos(yaw), y + arm * math.sin(yaw)
    w.box(hx, hy, z + height - 0.35, 0.7, 0.3, 0.25, yaw, '#e8e8d0')


@_object('other-object')
def bench(w, x, y, z, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    w.box(x, y, z + 0.42, 1.8, 0.45, 0.06, yaw, '#8a6a40')
    w.box(x - s * 0.22, y + c * 0.22, z + 0.5, 1.8, 0.06, 0.45, yaw, '#8a6a40')
    for d in (-0.75, 0.75):
        w.box(x + c * d, y + s * d, z, 0.08, 0.45, 0.42, yaw, C_METAL)


@_object('other-object')
def bin_(w, x, y, z):
    w.cylinder(x, y, z, 0.3, 0.95, color='#305030')


@_object('pole')
def sign_post(w, x, y, z, yaw):
    w.cylinder(x, y, z, 0.04, 2.6, color=C_METAL)
    with w.semantic('traffic-sign'):
        w.box(x, y, z + 2.0, 0.05, 0.7, 0.7, yaw, '#e0e0e0')


@_object('pole')
def bollard(w, x, y, z):
    w.cylinder(x, y, z, 0.1, 0.9, color='#404040')


@_object('other-structure')
def bus_stop(w, x, y, z, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    w.box(x - s * 0.7, y + c * 0.7, z, 4.0, 0.08, 2.3, yaw, '#a0b0c0')  # back panel
    w.box(x, y, z + 2.3, 4.2, 1.6, 0.1, yaw, '#606060')  # roof
    for d in (-1.9, 1.9):
        w.box(x + c * d, y + s * d, z, 0.08, 1.5, 2.3, yaw, '#a0b0c0')
    bench(w, x - s * 0.4, y + c * 0.4, z, yaw)


@_object('fence')
def fence(w, x0, y0, x1, y1, z_fn, spacing=2.5, height=1.2, rails=2):
    L = math.hypot(x1 - x0, y1 - y0)
    n = max(1, int(L / spacing))
    for i in range(n + 1):
        u = i / n
        px, py = x0 + u * (x1 - x0), y0 + u * (y1 - y0)
        w.cylinder(px, py, z_fn(px, py) - 0.2, 0.06, height + 0.2, color='#7a6040')
    for k in range(rails):
        zr = height * (0.45 + 0.5 * k / max(1, rails - 1))
        zm = z_fn((x0 + x1) / 2, (y0 + y1) / 2)
        w.box_between(x0, y0, x1, y1, zm + zr - 0.05, 0.05, 0.1, '#7a6040')


@_object('building')
def building(w, rng, cx, cy, lx, ly, height, yaw=0.0, z=0.0, pilaster=4.0, color=None,
             entrance_side=None):
    """Block building with facade relief: pilasters every `pilaster` m, a
    plinth, a roof parapet and an optional entrance canopy."""
    col = color or C_FACADE[int(rng.integers(len(C_FACADE)))]
    w.box(cx, cy, z - 0.3, lx, ly, height + 0.3, yaw, col)
    c, s = math.cos(yaw), math.sin(yaw)

    def P(u, v):
        return cx + c * u - s * v, cy + s * u + c * v

    # Plinth and parapet bands
    w.box(cx, cy, z - 0.3, lx + 0.3, ly + 0.3, 1.0, yaw, '#8a8478')
    w.box(cx, cy, z + height, lx + 0.2, ly + 0.2, 0.9, yaw, '#8a8478')
    if pilaster:
        for side, (L, nrm) in enumerate(((lx, ly), (ly, lx), (lx, ly), (ly, lx))):
            n = int(L / pilaster)
            for i in range(1, n):
                u = -L / 2 + i * L / n
                if side == 0:
                    px, py = P(u, -nrm / 2 - 0.15)
                elif side == 1:
                    px, py = P(nrm / 2 + 0.15, u)
                elif side == 2:
                    px, py = P(u, nrm / 2 + 0.15)
                else:
                    px, py = P(-nrm / 2 - 0.15, u)
                if side in (0, 2):
                    w.box(px, py, z, 0.4, 0.3, height, yaw, col)
                else:
                    w.box(px, py, z, 0.3, 0.4, height, yaw, col)
    if entrance_side is not None:
        # Cantilevered entrance canopy (no columns, so it never blocks a sidewalk)
        u, v = {0: (0, -ly / 2 - 1.0), 1: (lx / 2 + 1.0, 0), 2: (0, ly / 2 + 1.0),
                3: (-lx / 2 - 1.0, 0)}[int(entrance_side)]
        ex, ey = P(u, v)
        dims = (4.0, 2.0) if int(entrance_side) in (0, 2) else (2.0, 4.0)
        w.box(ex, ey, z + 3.0, dims[0], dims[1], 0.2, yaw, '#505860')


@_object('shelf')
def rack_row(w, rng, x0, y0, length, depth, levels, level_h, bay=2.7, axis='x', fill=0.75,
             z=0.0):
    """Pallet rack row starting at (x0, y0) along +x (or +y): uprights,
    beams per level and randomly filled pallet loads."""
    n = int(length / bay)
    H = levels * level_h + 0.3
    for i in range(n + 1):
        u = i * bay
        for v in (0.0, depth):
            px, py = (x0 + u, y0 + v) if axis == 'x' else (x0 + v, y0 + u)
            w.box(px, py, z, 0.1, 0.1, H, 0.0, C_RACK)
    for lv in range(1, levels + 1):
        zl = z + lv * level_h - 0.12
        for v in (0.0, depth):
            if axis == 'x':
                w.box(x0 + length / 2 * (n * bay / length), y0 + v, zl, n * bay, 0.1, 0.12, 0.0,
                      C_BEAM)
            else:
                w.box(x0 + v, y0 + n * bay / 2, zl, 0.1, n * bay, 0.12, 0.0, C_BEAM)
    for i in range(n):
        for lv in range(levels):
            zl = z + lv * level_h + (0.0 if lv == 0 else 0.0)
            for k in range(2):
                if rng.uniform() > fill:
                    continue
                u = i * bay + 0.15 + k * 1.25 + 0.6
                hgt = rng.uniform(0.4, level_h - 0.35)
                with w.instance(), w.semantic('goods'):
                    if axis == 'x':
                        w.box(x0 + u, y0 + depth / 2, zl + 0.15, 1.1, depth - 0.1, hgt, 0.0,
                              C_PALLET)
                    else:
                        w.box(x0 + depth / 2, y0 + u, zl + 0.15, depth - 0.1, 1.1, hgt, 0.0,
                              C_PALLET)


@_object('stairs')
def stairs_flight(w, x0, y0, z0, yaw, n_steps, riser, tread, width, color=C_CONCRETE,
                  thickness=None):
    """Stair flight: step i rises (i+1)*riser above z0, starting at (x0, y0)
    along `yaw`. Steps are solid down to z0, or `thickness` m deep (a
    stepped soffit, leaving headroom under the flight). Returns the (x, y, z)
    of the top edge."""
    c, s = math.cos(yaw), math.sin(yaw)
    for i in range(n_steps):
        u = (i + 0.5) * tread
        top = z0 + (i + 1) * riser
        bot = z0 - 0.05 if thickness is None else max(z0 - 0.05, top - thickness)
        w.box(x0 + c * u, y0 + s * u, bot, tread, width, top - bot, yaw, color, drivable=True)
    L = n_steps * tread
    return x0 + c * L, y0 + s * L, z0 + n_steps * riser
