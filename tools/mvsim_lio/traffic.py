"""Moving traffic for road worlds.

Vehicles are sets of boxes. The world holds a pool of them as named blocks
parked below the terrain (invisible to every ray); tools/generate.py gives
mvsim-dataset-gen one trajectory per block (--moving-block) for the
sequences whose trajectory asks for traffic, and the blocks of the vehicles a
sequence does not use stay parked.

All motion is derived from the sensor car's own along-road profile X(t), so
that traffic stops and starts with it:

- Same lane (and lanes in 'platoon' mode): car following with a time headway
  tau and a standstill distance d. The n-th vehicle ahead is where the car
  will be n headways later, plus n standstill distances:
  x = X(t + sum(tau)) + sum(d); behind, the mirror image. Vehicles ahead move
  off first and the start propagates backwards, as in a queue.
- Lanes in 'flow' mode: a stream that moves (1 + kappa) times as fast as the
  car, so it passes the car (kappa > 0) or is passed by it (kappa < 0), and
  stands when the car stands.
- Oncoming lanes: a stream at constant speed against the driving direction.

Streams live in a window of +-WINDOW/2 around the car (beyond the LiDAR
range); a vehicle leaving it at one end is moved to the other end below the
terrain, so it never jumps in view.
"""
import math
import os
import zlib

import numpy as np
from scipy.spatial.transform import Rotation

# Boxes of each kind in the vehicle frame (origin on the road under the
# center of the footprint, x forward): (x center, lx, ly, z0, lz) [m].
KINDS = {
    'car': [(0.0, 4.4, 1.8, 0.30, 0.75), (-0.3, 2.2, 1.6, 1.05, 0.45)],
    'van': [(0.0, 5.4, 2.0, 0.35, 1.95)],
    'truck': [(-1.2, 11.0, 2.5, 0.90, 3.10), (5.5, 2.2, 2.5, 0.50, 2.60)],
    'boxtruck': [(-0.9, 6.0, 2.4, 0.90, 2.80), (3.0, 1.9, 2.3, 0.50, 2.10)],
}
COLORS = {'car': '#4060a0', 'van': '#d0d0d0', 'truck': '#a0a0a0', 'boxtruck': '#c0b090'}
EGO_HALF = 2.3  # half length of the sensor car's body [m]
PARK_Z = -40.0  # blocks not in use, and stream vehicles changing end
WINDOW = 300.0  # length of the window of a stream around the car [m]
KNOT_HZ = 20.0


def half_lengths(kind):
    """(rear, front) extent of a vehicle from its frame origin."""
    return (max(lx / 2 - xc for xc, lx, _, _, _ in KINDS[kind]),
            max(lx / 2 + xc for xc, lx, _, _, _ in KINDS[kind]))


def block_name(i, j):
    return f'traffic{i:02d}_{j}'


def add_pool(world, pool, x, y):
    """Adds the blocks of the vehicles of `pool` (a list of kinds) to the
    world, parked below the terrain at (x, y). Each vehicle is one instance."""
    for i, kind in enumerate(pool):
        with world.instance(), world.semantic('car' if kind == 'car' else 'other-vehicle'):
            for j, (xc, lx, ly, z0, lz) in enumerate(KINDS[kind]):
                world.box(x + xc, y + 3.0 * i, PARK_Z + z0, lx, ly, lz, 0.0, COLORS[kind], plot=False,
                          name=block_name(i, j))


def _assign(pool, wanted, used):
    """Index of a free vehicle of the wanted kind in the pool."""
    for i, kind in enumerate(pool):
        if kind == wanted and i not in used:
            used.add(i)
            return i
    raise ValueError(f'traffic pool has no free {wanted}')


def tracks(pool, spec, road, tname, t_ego, x_ego):
    """Block trajectories of one sequence.

    pool: the world's list of vehicle kinds. spec: the trajectory's traffic
    (see below). road: dict(road_y=f(x), ground=f(x, y), lanes={name: lateral
    offset from the road axis}, x_range=(x0, x1)). t_ego, x_ego: the sensor
    car's knots (time, along-road x).

    spec = dict(
        ahead=[kinds, nearest first], behind=[kinds, nearest first],   # the car's own lane
        headway=(lo, hi) [s], gap=(lo, hi) [m] at standstill,
        # an entry of a list of kinds may be (kind, headway, gap) with its own values
        lanes={lane: dict(mode='platoon', kinds=[...], spread=m)       # around the car
                     | dict(mode='flow', kinds=[...], kappa=k)
                     | dict(mode='oncoming', kinds=[...], v=m/s)})
    Returns {block name: (t, xyz (N, 3), quat (N, 4))} with knots at KNOT_HZ.
    """
    rng = np.random.default_rng(zlib.crc32(f'traffic/{tname}'.encode()) + int(spec.get('seed', 0)))
    n = int(math.floor((t_ego[-1] - t_ego[0]) * KNOT_HZ)) + 1
    t = t_ego[0] + np.arange(n) / KNOT_HZ
    X = lambda tt: np.interp(tt, t_ego, x_ego)  # noqa: E731  (clamped at both ends)
    x0 = X(t)
    v = np.gradient(x0, 1.0 / KNOT_HZ)
    moving = np.clip(v / 5.0, 0.0, 1.0)
    hw = spec.get('headway', (1.3, 2.2))
    gp = spec.get('gap', (2.0, 3.5))
    used = set()
    vehicles = []  # (pool index, lane offset, x(t), hidden(t), heading sign)

    def chain(kinds, sign, lane, first_half, first_shift=None, wander=None):
        """Vehicles one behind the other (sign +1: ahead of the car, -1:
        behind it). first_half: distance from the car's center to the point
        the first gap is measured from. first_shift: headway of the first
        vehicle to the car's position (a lane beside the car has no vehicle
        there); None: a full headway."""
        shift = 0.0
        dist = 0.0
        prev = first_half
        for k, item in enumerate(kinds):
            kind, tau, d = item if isinstance(item, tuple) else (item, None, None)
            rear, front = half_lengths(kind)
            dist += prev + (rng.uniform(*gp) if d is None else d) + (rear if sign > 0 else front)
            if k == 0 and first_shift is not None:
                shift += first_shift
            else:
                shift += rng.uniform(*hw) if tau is None else tau
            x = X(t + sign * shift) + sign * dist
            if wander is not None:
                x = x + wander
            vehicles.append((_assign(pool, kind, used), lane, x, np.zeros(n, dtype=bool), 1.0))
            prev = front if sign > 0 else rear

    lanes = road['lanes']
    chain(spec.get('ahead', []), +1, lanes['ego'], EGO_HALF)
    chain(spec.get('behind', []), -1, lanes['ego'], EGO_HALF)
    for lane, ls in spec.get('lanes', {}).items():
        kinds = ls['kinds']
        if ls['mode'] == 'platoon':
            # The lane moves as a queue of its own, drifting slowly against the car's.
            w = rng.uniform(2.0, 5.0) * np.sin(2 * np.pi * (t - t[0]) / rng.uniform(30.0, 60.0)
                                               + rng.uniform(0, 2 * np.pi))
            w = np.cumsum(np.gradient(w) * moving)
            # The headway between the first vehicle ahead and the first one
            # behind is split at the car's position.
            k_ahead = (len(kinds) + 1) // 2
            h = rng.uniform(*hw)
            u = rng.uniform(0.1, 0.9)
            ph = rng.uniform(0.0, 4.0)
            chain(kinds[:k_ahead], +1, lanes[lane], -ph, first_shift=u * h, wander=w)
            chain(kinds[k_ahead:], -1, lanes[lane], ph, first_shift=(1.0 - u) * h, wander=w)
            continue
        if ls['mode'] == 'flow':
            rel = ls['kappa'] * (x0 - x0[0])
            heading = 1.0
        else:  # oncoming
            rel = -(ls['v'] * (t - t[0]) + (x0 - x0[0]))
            heading = -1.0
        slot = WINDOW / len(kinds)
        for k, kind in enumerate(kinds):
            r0 = k * slot + rng.uniform(-0.25, 0.25) * (slot - 16.0)
            r = np.mod(r0 + rel + WINDOW / 2, WINDOW) - WINDOW / 2
            vehicles.append((_assign(pool, kind, used), lanes[lane], x0 + r,
                             np.abs(r) > WINDOW / 2 - 8.0, heading))

    out = {}
    xa, xb = road['x_range']
    for i, lane, x, hidden, heading in vehicles:
        hidden = hidden | (x < xa) | (x > xb)
        y = road['road_y'](x) + lane
        dy = (road['road_y'](x + 0.5) - road['road_y'](x - 0.5))
        yaw = np.arctan2(dy, 1.0)
        z = road['ground'](x, y)
        c, s = np.cos(yaw), np.sin(yaw)
        dz = road['ground'](x + 1.5 * c, y + 1.5 * s) - road['ground'](x - 1.5 * c, y - 1.5 * s)
        pitch = -np.arctan2(dz, 3.0)
        if heading < 0:
            yaw = yaw + math.pi
            pitch = -pitch
        rot = Rotation.from_euler('ZYX', np.column_stack([yaw, pitch, np.zeros(n)]))
        q = rot.as_quat()
        for k in range(1, n):
            if np.dot(q[k], q[k - 1]) < 0:
                q[k] = -q[k]
        base = np.column_stack([x, y, z + np.where(hidden, PARK_Z, 0.0)])
        for j, (xc, lx, ly, z0, lz) in enumerate(KINDS[pool[i]]):
            # A box block's frame is its corner.
            corner = np.array([xc - lx / 2, -ly / 2, z0])
            out[block_name(i, j)] = (t, base + rot.apply(corner), q)
    return out


def write(tracks_by_block, out_dir):
    """Writes one TUM file per block; returns the --moving-block arguments."""
    out_dir.mkdir(parents=True, exist_ok=True)
    args = []
    for name, (t, p, q) in sorted(tracks_by_block.items()):
        f = out_dir / f'{name}.tum'
        # Written aside and renamed: another render of the same trajectory may
        # be reading the file.
        tmp = out_dir / f'{name}.tum.{os.getpid()}.tmp'
        with open(tmp, 'w') as fo:
            for k in range(len(t)):
                fo.write(f'{t[k]:.4f} {p[k, 0]:.6f} {p[k, 1]:.6f} {p[k, 2]:.6f} '
                         f'{q[k, 0]:.9f} {q[k, 1]:.9f} {q[k, 2]:.9f} {q[k, 3]:.9f}\n')
        os.replace(tmp, f)
        args += ['--moving-block', f'{name}={f}']
    return args
