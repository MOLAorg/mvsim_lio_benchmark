"""Densify a committed trajectory (TUM, 20 or 50 Hz knots) with a C2 spline.

mvsim-dataset-gen interpolates trajectories linearly (positions) and with
slerp (rotations), and computes IMU readings by finite differences with a
step h of one tenth of the IMU period (0.5 ms at 200 Hz). Accelerations are
only right if the knot spacing divides h, so the dense file has knots every
DENSE_DT = 0.5 ms, sampled from a cubic spline (positions, clamped: the
trajectory starts and ends at rest) and a rotation spline (continuous
angular velocity and acceleration).
"""
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation, RotationSpline

DENSE_DT = 0.0005


def load_tum(path):
    d = np.loadtxt(path, comments='#')
    return d[:, 0], d[:, 1:4], d[:, 4:8]


def spline(path):
    t, p, q = load_tum(path)
    pos = CubicSpline(t, p, axis=0, bc_type='clamped')
    rot = RotationSpline(t, Rotation.from_quat(q))
    return t, pos, rot


def _write(f, t, p, q, tfmt, pfmt):
    for i in range(len(t)):
        f.write(f'{t[i]:{tfmt}} {p[i, 0]:{pfmt}} {p[i, 1]:{pfmt}} {p[i, 2]:{pfmt}} '
                f'{q[i, 0]:{pfmt}} {q[i, 1]:{pfmt}} {q[i, 2]:{pfmt}} {q[i, 3]:{pfmt}}\n')


def sample(path, dt):
    t, pos, rot = spline(path)
    n = int(round((t[-1] - t[0]) / dt)) + 1
    # Integer-step times, so that knots sit exactly on the clock ticks:
    ts = t[0] + dt * np.arange(n)
    ts[-1] = min(ts[-1], t[-1])
    p = pos(ts)
    q = rot(ts).as_quat()
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]
    return ts, p, q


def write_dense(src, dst, dt=DENSE_DT):
    ts, p, q = sample(src, dt)
    with open(dst, 'w') as f:
        # Full precision: the IMU second differences over 0.5 ms turn a
        # 1e-7 m rounding of the positions into ~0.4 m/s^2 of error.
        _write(f, ts, p, q, '.7f', '.12f')
    return len(ts)


def write_gt(src, dst, dt=0.005):
    ts, p, q = sample(src, dt)
    with open(dst, 'w') as f:
        f.write('# timestamp x y z qx qy qz qw  (base_link in the world frame, 200 Hz)\n')
        _write(f, ts, p, q, '.6f', '.9f')
    return len(ts)


if __name__ == '__main__':
    import sys
    write_dense(sys.argv[1], sys.argv[2])
