#!/usr/bin/env python3
"""Renders every (world, trajectory, variant) sequence with mvsim-dataset-gen.

Output layout (under --out, default /mnt/datasets/mvsim_lio_benchmark):
  <world>/<traj>/<rig>-<noise>-<shutter>.rawlog       sensor data (MRPT rawlog)
  <world>/<traj>/<rig>-<noise>-<shutter>.gt.tum       base_link GT at sweep start times
  <world>/<traj>/<rig>-<noise>-<shutter>.lidar1.gt.tum  LiDAR GT at sweep start times
  <world>/<traj>/<rig>-<noise>-<shutter>.gen.log      generator log (with timing)
  <world>/<traj>/gt_200hz.tum                         base_link GT at 200 Hz

The noise RNG seed depends only on (world, trajectory), so clean/noisy and
global/rolling variants of a sequence are paired. Needs mvsim-dataset-gen in
PATH (see README: it must include the timestamp rounding fix).

Usage: tools/generate.py [--jobs 5] [--cores-per-job 8] [--first-core 0]
                         [--worlds a,b] [--variants os1_32-noisy-rolling,...] [--force]
                         [--labels]
"""
import argparse
import importlib.util
import json
import os
import random
import shutil
import subprocess
import sys
import time
import zlib
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from pathlib import Path
from queue import Queue

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'tools'))

from mvsim_lio import densify, traffic  # noqa: E402
from mvsim_lio.labels import OUTLIER  # noqa: E402

# Default variant set: all six (noise x shutter) combinations for the main
# rig, the noisy rolling-shutter one for the second rig, and the 64-ring
# density arm at the noisy and real levels (rolling shutter).
VARIANTS = ['os1_32-clean-global', 'os1_32-clean-rolling', 'os1_32-noisy-global',
            'os1_32-noisy-rolling', 'os1_32-real-global', 'os1_32-real-rolling',
            'vlp16-noisy-rolling', 'os1_64-noisy-rolling', 'os1_64-real-rolling']


# The generator: the one in PATH, or another build (MVSIM_LIO_DATASET_GEN).
GEN = os.environ.get('MVSIM_LIO_DATASET_GEN', 'mvsim-dataset-gen')


def seed_for(world, traj):
    return zlib.crc32(f'{world}/{traj}'.encode()) & 0x7fffffff


# The "real" noise level: a realistic LiDAR error model, with its parameters drawn per
# sequence and rig from ranges that bracket two real Ouster sensors (a parked OS0-32 and
# the OS1-64 of Newer College against its TLS map), so that no single sensor's error
# profile is learned: range noise growing with range, a beam of finite divergence sampled
# by sub-rays (the footprint spreads the return on oblique surfaces; at depth edges, the
# echoes of surfaces closer than the echo separation merge into one return between them),
# a detection range (weak returns at long range and grazing incidence are lost) and
# per-beam calibration errors. The first release (2026-11) used ranges and a model with a
# grazing-angle cutoff and random mixed pixels instead (see the README).
REAL_ERROR_RANGES = {
    'MVSIM_LIDAR_RANGE_STD': (0.005, 0.02),
    'MVSIM_LIDAR_RANGE_STD_PER_M': (5e-4, 1.5e-3),
    'MVSIM_LIDAR_BEAM_DIVERGENCE_DEG': (0.1, 0.3),
    'MVSIM_LIDAR_ECHO_SEPARATION': (1.5, 3.0),
    'MVSIM_LIDAR_DETECTION_RANGE': (110.0, 250.0),
    'MVSIM_LIDAR_BEAM_ELEVATION_STD_DEG': (0.0, 0.04),
    'MVSIM_LIDAR_BEAM_AZIMUTH_STD_DEG': (0.0, 0.04),
    'MVSIM_LIDAR_BEAM_RANGE_BIAS_STD': (0.0, 0.01),
}


# Rigs that share the error model draws of another rig: the density
# comparison arm (OS1-64 vs OS1-32) differs only in the ring count.
ERROR_MODEL_RIG = {'os1_64': 'os1_32'}


def real_error_env(world, traj, rig):
    """Error model parameters of a "real" sequence, reproducible from its name."""
    rig = ERROR_MODEL_RIG.get(rig, rig)
    r = random.Random(zlib.crc32(f'{world}/{traj}/{rig}'.encode()))
    env = {k: f'{r.uniform(lo, hi):.6g}' for k, (lo, hi) in REAL_ERROR_RANGES.items()}
    env['MVSIM_LIDAR_CALIB_SEED'] = str((seed_for(world, traj) ^ zlib.crc32(rig.encode())) & 0x7fffffff)
    return env


def list_sequences(worlds):
    out = []
    for w in worlds:
        for t in sorted((ROOT / 'worlds' / w / 'trajectories').glob('*.tum')):
            out.append((w, t.stem))
    return out


_WORLD_MODULES = {}


def world_module(world):
    if world not in _WORLD_MODULES:
        spec = importlib.util.spec_from_file_location(f'world_{world}', ROOT / 'worlds' / world / 'world.py')
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _WORLD_MODULES[world] = mod
    return _WORLD_MODULES[world]


def moving_block_args(world, traj, work_dir):
    """--moving-block arguments of a sequence: the trajectories of the blocks
    that move in it (traffic), if its world.py defines moving_blocks()."""
    fn = getattr(world_module(world), 'moving_blocks', None)
    if fn is None:
        return []
    t, p, _ = densify.load_tum(ROOT / 'worlds' / world / 'trajectories' / f'{traj}.tum')
    tracks = fn(traj, t, p[:, 0])
    if not tracks:
        return []
    return traffic.write(tracks, work_dir / world / f'{traj}.traffic')


def run_one(world, traj, variant, out_dir, work_dir, cores, force, labels=False):
    rig, noise, shutter = variant.split('-')
    odir = out_dir / world / traj
    odir.mkdir(parents=True, exist_ok=True)
    rawlog = odir / f'{variant}.rawlog'
    log = odir / f'{variant}.gen.log'
    gt = odir / f'{variant}.gt.tum'
    # Done if the rawlog and its ground truth exist, unless a log shows an
    # incomplete run (first-release sequences have no log: see the README).
    complete = rawlog.exists() and gt.exists() and (
        not log.exists() or 'Wrote vehicle ground truth' in log.read_text())
    if complete and not force:
        return f'{world}/{traj}/{variant}: exists, skipped'
    dense = work_dir / world / f'{traj}.dense.tum'
    cmd = ['taskset', '-c', cores, GEN, str(ROOT / 'worlds' / world / 'world.xml'),
           '--trajectory', str(dense), '--trajectory-format', 'tum', '-o', str(rawlog),
           '--seed', str(seed_for(world, traj)), '--shutter', shutter]
    if noise == 'clean':
        cmd.append('--noiseless')
    if labels:
        # Per-point semantic and instance labels; mixed returns are outliers.
        cmd += ['--labels', '--mixed-return-label', str(OUTLIER)]
    cmd += moving_block_args(world, traj, work_dir)
    env = dict(os.environ, MVSIM_LIO_RIG=str(ROOT / 'rigs' / f'{rig}.rig.xml'),
               TBB_NUM_THREADS=str(len(expand(cores))))
    if noise == 'real':
        errors = real_error_env(world, traj, rig)
        env.update(errors)
        (odir / f'{variant}.errors.json').write_text(json.dumps(errors, indent=1) + '\n')
    t0 = time.time()
    with open(log, 'w') as f:
        f.write('# ' + ' '.join(cmd) + '\n')
        f.flush()
        r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, env=env)
        dt = time.time() - t0
        f.write(f'\n# exit code {r.returncode}, wall time {dt:.1f} s\n')
    if r.returncode != 0:
        return f'{world}/{traj}/{variant}: FAILED (see {log})'
    return f'{world}/{traj}/{variant}: OK in {dt:.0f} s, {rawlog.stat().st_size / 1e6:.0f} MB'


def _call(job):
    fn, *args = job
    return fn(*args)


def expand(cores):
    out = []
    for part in cores.split(','):
        a, _, b = part.partition('-')
        out.extend(range(int(a), int(b or a) + 1))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/mnt/datasets/mvsim_lio_benchmark')
    ap.add_argument('--jobs', type=int, default=5)
    ap.add_argument('--cores-per-job', type=int, default=8)
    ap.add_argument('--first-core', type=int, default=0)
    ap.add_argument('--worlds', default='')
    ap.add_argument('--trajs', default='', help='comma-separated trajectory names')
    ap.add_argument('--variants', default=','.join(VARIANTS))
    ap.add_argument('--force', action='store_true')
    ap.add_argument('--labels', action='store_true',
                    help='store the per-point "label" and "instance" fields (tools/mvsim_lio/labels.py)')
    a = ap.parse_args()

    if not shutil.which(GEN):
        sys.exit(f'{GEN} not found')
    # Without --worlds: the worlds of the hosted set (a world.py with
    # HOSTED = False is rendered only when named).
    worlds = a.worlds.split(',') if a.worlds else [
        w for w in sorted(p.parent.name for p in (ROOT / 'worlds').glob('*/world.xml'))
        if getattr(world_module(w), 'HOSTED', True)]
    seqs = list_sequences(worlds)
    if a.trajs:
        seqs = [s for s in seqs if s[1] in a.trajs.split(',')]
    out_dir = Path(a.out)
    work_dir = out_dir / '.work'
    # Dense trajectories (shared by every variant) and 200 Hz ground truth:
    todo = []
    for w, t in seqs:
        (work_dir / w).mkdir(parents=True, exist_ok=True)
        (out_dir / w / t).mkdir(parents=True, exist_ok=True)
        src = ROOT / 'worlds' / w / 'trajectories' / f'{t}.tum'
        dense = work_dir / w / f'{t}.dense.tum'
        if a.force or not dense.exists() or dense.stat().st_mtime < src.stat().st_mtime:
            todo.append((densify.write_dense, src, dense))
        todo.append((densify.write_gt, src, out_dir / w / t / 'gt_200hz.tum'))
    with ProcessPoolExecutor(max_workers=8) as ex:
        list(ex.map(_call, todo))

    pool = Queue()
    for j in range(a.jobs):
        c0 = a.first_core + j * a.cores_per_job
        pool.put(f'{c0}-{c0 + a.cores_per_job - 1}')

    def task(args):
        cores = pool.get()
        try:
            msg = run_one(*args, out_dir, work_dir, cores, a.force, a.labels)
        finally:
            pool.put(cores)
        print(msg, flush=True)
        return msg

    jobs = [(w, t, v) for w, t in seqs for v in a.variants.split(',')]
    print(f'{len(jobs)} sequences to render with {a.jobs} jobs x {a.cores_per_job} cores',
          flush=True)
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        results = list(ex.map(task, jobs))
    failed = [r for r in results if 'FAILED' in r]
    print(f'DONE: {len(results) - len(failed)} ok, {len(failed)} failed')
    for r in failed:
        print(r)


if __name__ == '__main__':
    main()
