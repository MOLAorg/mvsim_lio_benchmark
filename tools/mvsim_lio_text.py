#!/usr/bin/env python3
"""Reader for the plain-text copy of MVSim-LIO. Needs only Python 3 and NumPy.

Each sequence variant is a folder `<world>/<trajectory>/<variant>/` with:

  lidar.txt.xz      t_ns x y z ring label instance  (one point per line, sweep after sweep)
  sweeps.txt.xz     sweep stamp_s num_points
  imu.txt.xz        stamp_s wx wy wz ax ay az
  odometry.txt.xz   stamp_s x y yaw vx vy omega
  extrinsics.yaml   LiDAR and IMU poses in base_link

Usage as a script: mvsim_lio_text.py <variant folder>  (prints a summary)
"""
import bz2
import gzip
import io
import itertools
import lzma
import sys
from pathlib import Path

import numpy as np

LIDAR_DTYPE = np.dtype([('t_ns', np.int64), ('x', np.float32),
                        ('y', np.float32), ('z', np.float32), ('ring', np.uint16),
                        ('label', np.uint16), ('instance', np.uint16)])
SWEEP_DTYPE = np.dtype([('sweep', np.int32), ('stamp', np.float64), ('num_points', np.int64)])
IMU_DTYPE = np.dtype([('stamp', np.float64), ('wx', np.float64), ('wy', np.float64),
                      ('wz', np.float64), ('ax', np.float64), ('ay', np.float64),
                      ('az', np.float64)])
ODOM_DTYPE = np.dtype([('stamp', np.float64), ('x', np.float64), ('y', np.float64),
                       ('yaw', np.float64), ('vx', np.float64), ('vy', np.float64),
                       ('omega', np.float64)])


def _open(folder, name):
    """Opens `name` in `folder`, compressed (.xz, .bz2, .gz) or not, as text."""
    folder = Path(folder)
    for ext, opener in (('.xz', lzma.open), ('.bz2', bz2.open), ('.gz', gzip.open),
                        ('', open)):
        p = folder / (name + ext)
        if p.exists():
            return io.TextIOWrapper(opener(p, 'rb'), encoding='ascii') if ext else open(p)
    raise FileNotFoundError(folder / name)


def _table(folder, name, dtype, max_rows=None):
    with _open(folder, name) as f:
        a = np.loadtxt(f, dtype=dtype, comments='#', max_rows=max_rows, ndmin=1)
    return a


def read_sweeps(folder):
    return _table(folder, 'sweeps.txt', SWEEP_DTYPE)


def read_imu(folder):
    return _table(folder, 'imu.txt', IMU_DTYPE)


def read_odometry(folder):
    return _table(folder, 'odometry.txt', ODOM_DTYPE)


def read_lidar(folder, max_points=None):
    """All points as one structured array (t_ns, x, y, z, ring, label, instance), and the
    sweep index of each point."""
    pts = _table(folder, 'lidar.txt', LIDAR_DTYPE, max_rows=max_points)
    sw = read_sweeps(folder)
    sweep = np.repeat(sw['sweep'], sw['num_points'])[:len(pts)]
    return pts, sweep


def iter_sweeps(folder):
    """Yields (sweep index, stamp [s], points) one sweep at a time, in bounded memory.

    Point `i` was measured at stamp + t_ns[i] * 1e-9 and is expressed in the LiDAR frame
    at that instant.
    """
    sweeps = read_sweeps(folder)
    with _open(folder, 'lidar.txt') as f:
        header = f.readline()
        if not header.startswith('#'):
            raise ValueError('lidar.txt: missing header line')
        for s in sweeps:
            n = int(s['num_points'])
            # Exactly n lines: loadtxt alone may read ahead from a shared handle.
            pts = np.loadtxt(itertools.islice(f, n), dtype=LIDAR_DTYPE, ndmin=1)
            if len(pts) != n:
                raise ValueError(f'sweep {s["sweep"]}: {len(pts)} points, expected {n}')
            yield int(s['sweep']), float(s['stamp']), pts


def read_extrinsics(folder):
    """The two sensor poses: {'lidar': {...}, 'imu': {...}} (a minimal YAML subset parser)."""
    out = {}
    cur = None
    for line in (Path(folder) / 'extrinsics.yaml').read_text().splitlines():
        if not line.strip():
            continue
        if not line.startswith(' '):
            cur = out.setdefault(line.rstrip(':'), {})
            continue
        key, val = (s.strip() for s in line.split(':', 1))
        if val.startswith('['):
            cur[key] = [float(v) for v in val.strip('[]').split(',')]
        else:
            cur[key] = val
    return out


def read_tum(path):
    """A TUM trajectory: stamp_s x y z qx qy qz qw, as an (N, 8) array."""
    return np.loadtxt(path, comments='#', ndmin=2)


def main():
    folder = Path(sys.argv[1])
    print('extrinsics:', read_extrinsics(folder))
    sw = read_sweeps(folder)
    print(f'{len(sw)} sweeps, {sw["num_points"].sum()} points, '
          f'{sw["stamp"][0]:.3f} to {sw["stamp"][-1]:.3f} s')
    imu = read_imu(folder)
    print(f'{len(imu)} IMU readings, mean acc {imu["az"].mean():.3f} m/s^2 (z)')
    _, stamp, pts = next(iter_sweeps(folder))
    print(f'first sweep at {stamp:.3f} s: {len(pts)} points, '
          f'range {np.hypot(pts["x"], pts["y"]).max():.1f} m')


if __name__ == '__main__':
    main()
