#!/usr/bin/env python3
"""Sanity validation: runs MOLA LiDAR odometry (mola-lo-cli-rawlog, default
pipeline) on generated sequences and scores them against the ground truth.

Metrics (translation, base_link, at the LiDAR sweep times):
  ape_se3   APE RMSE after SE(3) Umeyama alignment [m]
  ape_orig  APE RMSE aligning only the first pose (no fitting) [m]
  drift     end-point error with origin alignment, % of the distance traveled
  rpe10     RMSE of the translation error over 10 m segments [m]

Usage: tools/validate_mola.py --seqs world/traj/variant,... [--jobs 4] [--cores-per-job 8]
       tools/validate_mola.py --variant os1_32-noisy-rolling   (every trajectory)
"""
import argparse
import json
import os
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from queue import Queue

import numpy as np
import sys

from evo.core import metrics
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'tools'))
from mvsim_lio import densify  # noqa: E402

DATA = Path('/mnt/datasets/mvsim_lio_benchmark')


def gt_at(traj_tum, timestamps):
    """Ground truth at arbitrary times, from the same C2 spline the generator
    used (estimators may stamp scans at their mid-sweep time, not the start)."""
    t, pos, rot = densify.spline(traj_tum)
    ts = np.clip(timestamps, t[0], t[-1])
    T = np.tile(np.eye(4), (len(ts), 1, 1))
    T[:, :3, :3] = rot(ts).as_matrix()
    T[:, :3, 3] = pos(ts)
    return PoseTrajectory3D(poses_se3=list(T), timestamps=np.asarray(timestamps))


def score(traj_tum, est_path):
    est = file_interface.read_tum_trajectory_file(str(est_path))
    gt = gt_at(traj_tum, est.timestamps)
    out = {'n_poses': int(est.num_poses), 'length_m': float(gt.path_length)}
    # Origin alignment (no fitting): est expressed starting at the GT's first pose
    e0 = PoseTrajectory3D(poses_se3=[p.copy() for p in est.poses_se3],
                          timestamps=est.timestamps.copy())
    e0.align_origin(gt)
    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((gt, e0))
    out['ape_orig'] = float(ape.get_statistic(metrics.StatisticsType.rmse))
    out['drift_pct'] = float(100.0 * np.linalg.norm(gt.positions_xyz[-1] - e0.positions_xyz[-1])
                             / max(gt.path_length, 1e-6))
    e1 = PoseTrajectory3D(poses_se3=[p.copy() for p in est.poses_se3],
                          timestamps=est.timestamps.copy())
    e1.align(gt, correct_scale=False)
    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((gt, e1))
    out['ape_se3'] = float(ape.get_statistic(metrics.StatisticsType.rmse))
    try:
        rpe = metrics.RPE(metrics.PoseRelation.translation_part, delta=10.0,
                          delta_unit=metrics.Unit.meters, all_pairs=False)
        rpe.process_data((gt, est))
        out['rpe10'] = float(rpe.get_statistic(metrics.StatisticsType.rmse))
    except Exception:  # noqa: BLE001  (too short for a 10 m segment)
        out['rpe10'] = float('nan')
    return out


def run(seq, out_root, cores, force):
    world, traj, variant = seq.split('/')
    d = DATA / world / traj
    rawlog = d / f'{variant}.rawlog'
    odir = out_root / world / traj / variant
    odir.mkdir(parents=True, exist_ok=True)
    est = odir / 'lo.tum'
    log = odir / 'mola.log'
    t0 = time.time()
    if force or not est.exists():
        ncores = len(range(int(cores.split('-')[0]), int(cores.split('-')[1]) + 1))
        env = dict(os.environ, MOLA_MAX_THREADS=str(ncores), MOLA_LIDAR_NAME='lidar1',
                   MOLA_IMU_NAME='imu1')
        cmd = ['taskset', '-c', cores, 'mola-lo-cli-rawlog', str(rawlog),
               '--lidar-sensor-label', 'lidar1', '--imu-sensor-label', 'imu1',
               '--output-tum-path', str(est)]
        with open(log, 'w') as f:
            r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, env=env, cwd=odir)
        if r.returncode != 0 or not est.exists():
            return seq, {'error': f'mola exit {r.returncode}'}
    res = score(ROOT / 'worlds' / world / 'trajectories' / f'{traj}.tum', est)
    res['mola_wall_s'] = round(time.time() - t0, 1)
    (odir / 'metrics.json').write_text(json.dumps(res, indent=1) + '\n')
    return seq, res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seqs', default='')
    ap.add_argument('--variant', default='')
    ap.add_argument('--out', default=str(DATA / 'validation'))
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--cores-per-job', type=int, default=8)
    ap.add_argument('--first-core', type=int, default=0)
    ap.add_argument('--force', action='store_true')
    a = ap.parse_args()
    seqs = [s for s in a.seqs.split(',') if s]
    if a.variant:
        seqs += sorted(f'{p.parent.parent.name}/{p.parent.name}/{a.variant}'
                       for p in DATA.glob(f'*/*/{a.variant}.rawlog'))
    out_root = Path(a.out)
    pool = Queue()
    for j in range(a.jobs):
        c0 = a.first_core + j * a.cores_per_job
        pool.put(f'{c0}-{c0 + a.cores_per_job - 1}')

    def task(seq):
        cores = pool.get()
        try:
            s, r = run(seq, out_root, cores, a.force)
        finally:
            pool.put(cores)
        print(s, json.dumps({k: (round(v, 3) if isinstance(v, float) else v)
                             for k, v in r.items()}), flush=True)
        return s, r

    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        results = dict(ex.map(task, seqs))
    summary = out_root / 'summary.json'
    old = json.loads(summary.read_text()) if summary.exists() else {}
    old.update(results)
    summary.write_text(json.dumps(old, indent=1, sort_keys=True) + '\n')


if __name__ == '__main__':
    main()
