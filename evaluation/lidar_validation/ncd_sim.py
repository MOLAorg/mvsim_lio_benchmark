#!/usr/bin/env python3
"""Simulated OS1-64 sweeps at the refined Newer College poses, with the error model of
the benchmark at several points of its parameter ranges (and noise-free).

Usage: ncd_sim.py <work_dir> <beam_intrinsics.json> <lidar_tool>
Writes sim_<config>.bin next to the inputs it generates.
"""
import argparse
import json
import os
import subprocess
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

# Ends and middle of the ranges of the realistic error model (paper, tab:realnoise).
CONFIGS = {
    'clean': dict(sigma0=0, k=0, divergence_deg=0, max_incidence_deg=0, p_mix=0,
                  elev_std_deg=0, azim_std_deg=0, bias_std=0),
    'low': dict(sigma0=0.005, k=0, divergence_deg=0.05, max_incidence_deg=88, p_mix=0.1,
                elev_std_deg=0, azim_std_deg=0, bias_std=0),
    'mid': dict(sigma0=0.0125, k=1.5e-4, divergence_deg=0.175, max_incidence_deg=83, p_mix=0.3,
                elev_std_deg=0.02, azim_std_deg=0.02, bias_std=0.005),
    'high': dict(sigma0=0.02, k=3e-4, divergence_deg=0.3, max_incidence_deg=78, p_mix=0.5,
                 elev_std_deg=0.04, azim_std_deg=0.04, bias_std=0.01),
}
# The beam model (sub-rays, echoes merged within the echo separation) with the noise
# fitted to the OS0-32: a sweep of the echo separation at the OS0-32 divergence, and of the
# divergence at 2.5 m; then the ends of the current parameter ranges.
FIT = dict(sigma0=0.00684, k=0.00115, max_incidence_deg=0, p_mix=0,
           elev_std_deg=0, azim_std_deg=0, bias_std=0)
CONFIGS.update({
    'beam_l10': dict(FIT, divergence_deg=0.276, echo_separation=1.0),
    'beam_l20': dict(FIT, divergence_deg=0.276, echo_separation=2.0),
    'beam_l30': dict(FIT, divergence_deg=0.276, echo_separation=3.0),
    'beam_d10_l25': dict(FIT, divergence_deg=0.10, echo_separation=2.5),
    'beam_d15_l25': dict(FIT, divergence_deg=0.15, echo_separation=2.5),
    'beam_d28_l25': dict(FIT, divergence_deg=0.276, echo_separation=2.5),
    'v2_low': dict(sigma0=0.005, k=5e-4, divergence_deg=0.1, max_incidence_deg=0, p_mix=0,
                   echo_separation=1.5, detection_range=250, elev_std_deg=0, azim_std_deg=0, bias_std=0),
    'v2_high': dict(sigma0=0.02, k=1.5e-3, divergence_deg=0.3, max_incidence_deg=0, p_mix=0,
                    echo_separation=3.0, detection_range=110, elev_std_deg=0.04, azim_std_deg=0.04,
                    bias_std=0.01),
})
COMMON = dict(cols=1024, period=0.1, min_range=0.5, max_range=120, seed=1, calib_seed=7)


def tum_line(t, T):
    q = R.from_matrix(T[:3, :3]).as_quat()
    p = T[:3, 3]
    return f'{t:.6f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('work')
    ap.add_argument('intrinsics')
    ap.add_argument('tool')
    ap.add_argument('--configs', nargs='*', default=list(CONFIGS))
    a = ap.parse_args()
    w = Path(a.work)
    P = np.load(w / 'poses.npz')
    poses = P['poses']
    stamps = P['stamps']
    tcol = P['tcol']
    with open(w / 'sim_poses.tum', 'w') as fh:
        for i in range(len(stamps)):
            fh.write(tum_line(stamps[i] + tcol[0], poses[i, 0]))
            fh.write(tum_line(stamps[i] + tcol[-1], poses[i, -1]))
    with open(w / 'sim_sweeps.txt', 'w') as fh:
        for i in range(len(stamps)):
            fh.write(f'{stamps[i] + tcol[0]:.6f}\n')
    alt = json.loads(Path(a.intrinsics).read_text())['beam_altitude_angles']
    (w / 'sim_elev.txt').write_text(''.join(f'{e}\n' for e in alt))
    env = dict(os.environ)
    for name in a.configs:
        opts = dict(COMMON, **CONFIGS[name])
        (w / f'sim_opts_{name}.txt').write_text(''.join(f'{k} {v}\n' for k, v in opts.items()))
        cmd = [a.tool, 'sim', str(w / 'quad_mesh_clean.ply'), str(w / 'sim_poses.tum'),
               str(w / 'sim_sweeps.txt'), str(w / 'sim_elev.txt'), str(w / f'sim_opts_{name}.txt'),
               str(w / f'sim_{name}.bin')]
        print(name, flush=True)
        subprocess.run(cmd, check=True, env=env)


if __name__ == '__main__':
    main()
