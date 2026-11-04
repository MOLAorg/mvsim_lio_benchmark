#!/usr/bin/env python3
"""Checks the simulated IMU of a (clean) sequence against the trajectory:
angular velocity and specific force of the imu1 frame, computed independently
from the same C2 spline the generator used, including the lever arm of the
IMU mounting point.

Usage: tools/check_imu.py <world> <traj> [variant]   (default os1_32-clean-global)
Needs rawlog-edit (MRPT) in PATH.
"""
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'tools'))

from mvsim_lio import densify  # noqa: E402

DATA = Path('/mnt/datasets/mvsim_lio_benchmark')
IMU_POS = np.array([0.05, -0.03, 0.45])  # imu1 in base_link (rigs/*.rig.xml)
G = 9.81


def expected(traj_tum, t, dt=1e-3):
    _, pos, rot = densify.spline(traj_tum)
    t = np.clip(t, pos.x[0] + 2 * dt, pos.x[-1] - 2 * dt)

    def p_imu(tt):
        return pos(tt) + rot(tt).apply(IMU_POS)

    acc_w = (p_imu(t + dt) - 2 * p_imu(t) + p_imu(t - dt)) / dt ** 2
    R0 = rot(t)
    f = R0.inv().apply(acc_w + np.array([0, 0, G]))
    rel = rot(t - dt).inv() * rot(t + dt)
    w = rel.as_rotvec() / (2 * dt)
    return w, f


def main():
    world, traj = sys.argv[1], sys.argv[2]
    variant = sys.argv[3] if len(sys.argv) > 3 else 'os1_32-clean-global'
    rawlog = DATA / world / traj / f'{variant}.rawlog'
    with tempfile.TemporaryDirectory() as td:
        link = Path(td) / 'seq.rawlog'
        link.symlink_to(rawlog)
        subprocess.run(['rawlog-edit', '-i', str(link), '--export-imu-txt'], check=True,
                       capture_output=True, cwd=td)
        d = np.loadtxt(next(Path(td).glob('*imu1*.txt')), comments='%')
    t = d[:, 0]
    acc = d[:, 1:4]
    gyr = d[:, [6, 5, 4]]  # file columns: WZ WY WX
    w_exp, f_exp = expected(ROOT / 'worlds' / world / 'trajectories' / f'{traj}.tum', t)
    ea = acc - f_exp
    ew = gyr - w_exp
    print(f'{world}/{traj}/{variant}: {len(t)} IMU samples')
    print(f'  gyro  error: rms {np.sqrt((ew ** 2).mean(0))} max {np.abs(ew).max(0)} rad/s')
    print(f'  accel error: rms {np.sqrt((ea ** 2).mean(0))} max {np.abs(ea).max(0)} m/s^2')
    print(f'  signal rms : gyro {np.sqrt((w_exp ** 2).mean(0))}, accel {np.sqrt((f_exp ** 2).mean(0))}')


if __name__ == '__main__':
    main()
